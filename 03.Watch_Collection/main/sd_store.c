#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "sd_store.h"

#define MOUNT_POINT     "/sd"
#define PATH_LEN        40
#define FILE_BUF_LEN    4096
#define WRITER_PRIO     3
#define WRITER_STACK    6144
#define META_PERIOD_US  10000000

#define CSV_HEADER "timestamp_ms,local_ms,frame_idx,sample_seq,sample_idx," \
                   "acc_x_raw,acc_y_raw,acc_z_raw,is_overlap\n"
#define META_HEADER "time_ms,node,frames,lines,lost_samples,restarts,fifo_ovf,ring_drop," \
                    "queue_drops,write_errors,max_frame_ms\n"

typedef enum { ITEM_FRAME, ITEM_CLOSE } item_type_t;

typedef struct {
    item_type_t type;
    void *owner;
    const pkt_data_t *d;
} item_t;

typedef struct {
    bool     has_seq;
    uint32_t next_seq;      /* seq mong đợi của mẫu mới đầu tiên */
} node_track_t;

static const char *TAG = "store";

static sdmmc_card_t *s_card;
static QueueHandle_t s_q;
static SemaphoreHandle_t s_closed;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Thứ tự theo node_id: [0] = node 1 = tay phải, [1] = node 2 = tay trái */
static const char *const FILE_PREFIX[STORE_NODES] = { "right_hand", "left_hand" };

static int   s_index;
static char  s_path[STORE_NODES][PATH_LEN];
static FILE *s_csv[STORE_NODES];
static FILE *s_meta;
static char *s_buf[STORE_NODES];
static node_track_t s_track[STORE_NODES];
static store_stats_t s_stats;
static int64_t s_next_sync, s_next_meta;

static esp_err_t card_mount(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = CONFIG_WF_SD_MOSI_GPIO,
        .miso_io_num = CONFIG_WF_SD_MISO_GPIO,
        .sclk_io_num = CONFIG_WF_SD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "spi bus");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = CONFIG_WF_SD_FREQ_KHZ;

    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.host_id = SPI2_HOST;
    dev.gpio_cs = CONFIG_WF_SD_CS_GPIO;

    esp_vfs_fat_mount_config_t mnt = {
        .format_if_mount_failed = false,    /* không bao giờ tự xóa dữ liệu trên thẻ */
        .max_files = STORE_NODES + 2,
        .allocation_unit_size = 16 * 1024,
    };
    esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &dev, &mnt, &s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount failed: %s (CS=%d SCK=%d MOSI=%d MISO=%d)", esp_err_to_name(err),
                 CONFIG_WF_SD_CS_GPIO, CONFIG_WF_SD_SCK_GPIO,
                 CONFIG_WF_SD_MOSI_GPIO, CONFIG_WF_SD_MISO_GPIO);
        spi_bus_free(SPI2_HOST);
        return err;
    }
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

static void add_write_error(void)
{
    portENTER_CRITICAL(&s_lock);
    s_stats.write_errors++;
    portEXIT_CRITICAL(&s_lock);
}

static void write_meta(void)
{
    store_stats_t st;
    store_get_stats(&st);
    int64_t now_ms = esp_timer_get_time() / 1000;

    for (int i = 0; i < STORE_NODES; i++) {
        const store_node_stats_t *n = &st.node[i];
        int r = fprintf(s_meta, "%" PRId64 ",%d,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
                        ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n",
                        now_ms, i + 1, n->frames, n->lines, n->lost_samples, n->restarts,
                        n->fifo_ovf, n->ring_drop, st.queue_drops, st.write_errors,
                        st.max_frame_ms);
        if (r < 0) {
            add_write_error();
        }
    }
    ESP_LOGI(TAG, "file %d: node1 %" PRIu32 " lines, node2 %" PRIu32 " lines, lost %" PRIu32 "/%" PRIu32
             ", max write %" PRIu32 " ms, queue peak %" PRIu32 ", drops %" PRIu32 ", errors %" PRIu32,
             s_index, st.node[0].lines, st.node[1].lines, st.node[0].lost_samples,
             st.node[1].lost_samples, st.max_frame_ms, st.queue_peak, st.queue_drops,
             st.write_errors);
}

static void sync_file(FILE *f)
{
    if (f == NULL) {
        return;
    }
    if (fflush(f) != 0 || fsync(fileno(f)) != 0) {
        ESP_LOGE(TAG, "sync failed: %s", strerror(errno));
        add_write_error();
    }
}

static void sync_all(void)
{
    for (int i = 0; i < STORE_NODES; i++) {
        sync_file(s_csv[i]);
    }
    sync_file(s_meta);
}

/* Kiểm tra last_seq so với khung trước của node, rồi cập nhật thống kê */
static void track_frame(int idx, const pkt_data_t *d)
{
    node_track_t *t = &s_track[idx];
    uint32_t n_new = d->n_total - d->n_overlap;
    uint32_t first_new = d->last_seq - n_new + 1;
    uint32_t lost = 0, restart = 0, refilled = 0;
    uint32_t missing = s_stats.node[idx].lost_samples;  /* chỉ task này ghi trường đó */

    if (!t->has_seq || first_new >= t->next_seq) {
        lost = t->has_seq ? first_new - t->next_seq : 0;
        t->next_seq = d->last_seq + 1;
    } else if (missing >= n_new) {
        refilled = n_new;   /* khung gửi lại đến muộn: lấp lại phần đã tính là thiếu */
    } else {
        restart = 1;
        t->next_seq = d->last_seq + 1;
    }
    t->has_seq = true;

    portENTER_CRITICAL(&s_lock);
    store_node_stats_t *n = &s_stats.node[idx];
    n->frames++;
    n->lines += d->n_total;
    n->lost_samples += lost;
    n->lost_samples -= refilled;
    n->restarts += restart;
    n->fifo_ovf = d->fifo_ovf;
    n->ring_drop = d->ring_drop;
    portEXIT_CRITICAL(&s_lock);

    if (lost) {
        ESP_LOGW(TAG, "node %d: %" PRIu32 " samples missing before frame %" PRIu32,
                 d->node_id, lost, d->frame_idx);
    }
    if (restart) {
        ESP_LOGW(TAG, "node %d: seq went backwards at frame %" PRIu32 " (node restarted?)",
                 d->node_id, d->frame_idx);
    }
}

static void write_frame(const pkt_data_t *d)
{
    if (d->node_id < 1 || d->node_id > STORE_NODES || d->n_total == 0) {
        ESP_LOGW(TAG, "bad frame: node %u, n_total %u", d->node_id, d->n_total);
        return;
    }
    int idx = d->node_id - 1;
    FILE *f = s_csv[idx];
    if (f == NULL) {
        return;
    }

    int64_t t0 = esp_timer_get_time();
    int64_t span = d->t_last_us - d->t_first_us;
    uint32_t div = d->n_total > 1 ? d->n_total - 1u : 1u;
    uint32_t seq0 = d->last_seq - (d->n_total - 1u);
    bool failed = false;

    for (uint32_t i = 0; i < d->n_total; i++) {
        pkt_sample_t s;
        memcpy(&s, &d->s[i], sizeof(s));
        int64_t rel = span * i / div;
        int64_t t = d->t_first_us + rel;
        int64_t loc = d->t_first_local_us + rel;

        /* In micro-giây dưới dạng ms có 3 số lẻ: làm tròn tới 1 ms sẽ mất độ chính xác */
        int r = fprintf(f, "%" PRId64 ".%03d,%" PRId64 ".%03d,%" PRIu32 ",%" PRIu32 ",%" PRIu32
                        ",%d,%d,%d,%d\n",
                        t / 1000, (int)(t % 1000), loc / 1000, (int)(loc % 1000),
                        d->frame_idx, seq0 + i, i, s.x, s.y, s.z, i < d->n_overlap);
        if (r < 0) {
            failed = true;
            break;
        }
    }
    if (failed) {
        ESP_LOGE(TAG, "write failed: %s", strerror(errno));
        add_write_error();
    }
    track_frame(idx, d);

    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    portENTER_CRITICAL(&s_lock);
    if (ms > s_stats.max_frame_ms) {
        s_stats.max_frame_ms = ms;
    }
    portEXIT_CRITICAL(&s_lock);
}

static void release_files(void)
{
    for (int i = 0; i < STORE_NODES; i++) {
        if (s_csv[i]) {
            fclose(s_csv[i]);
            s_csv[i] = NULL;
        }
        free(s_buf[i]);
        s_buf[i] = NULL;
    }
    if (s_meta) {
        fclose(s_meta);
        s_meta = NULL;
    }
}

static void close_files(void)
{
    write_meta();
    sync_all();
    release_files();
    ESP_LOGI(TAG, "file %d closed", s_index);
}

static void writer_task(void *arg)
{
    for (;;) {
        item_t it;
        bool got = xQueueReceive(s_q, &it, pdMS_TO_TICKS(100)) == pdTRUE;

        if (got && it.type == ITEM_CLOSE) {
            close_files();
            xSemaphoreGive(s_closed);
            continue;
        }
        if (got) {
            write_frame(it.d);
            free(it.owner);
        }

        if (s_meta == NULL) {
            continue;
        }
        int64_t now = esp_timer_get_time();
        if (now >= s_next_meta) {
            s_next_meta = now + META_PERIOD_US;
            write_meta();
        }
        if (now >= s_next_sync) {
            s_next_sync = now + CONFIG_WF_STORE_SYNC_MS * 1000LL;
            sync_all();
        }
    }
}

/* Số kế tiếp = số lớn nhất trên thẻ + 1, nên thứ tự vẫn đúng kể cả khi đã xóa bớt file */
static int next_index(void)
{
    DIR *dir = opendir(MOUNT_POINT);
    if (dir == NULL) {
        ESP_LOGE(TAG, "opendir failed: %s", strerror(errno));
        return -1;
    }
    int max = 0;
    struct dirent *e;
    while ((e = readdir(dir)) != NULL) {
        int n;
        if (sscanf(e->d_name, "right_hand_%d.csv", &n) == 1 ||
            sscanf(e->d_name, "left_hand_%d.csv", &n) == 1 ||
            sscanf(e->d_name, "meta_%d.csv", &n) == 1) {
            max = n > max ? n : max;
        }
    }
    closedir(dir);
    return max + 1;
}

static FILE *open_output(const char *path, const char *header, char **buf)
{
    /* O_EXCL: thà báo lỗi còn hơn ghi đè dữ liệu đã có */
    int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0664);
    FILE *f = fd >= 0 ? fdopen(fd, "w") : NULL;
    if (f == NULL) {
        ESP_LOGE(TAG, "open %s failed: %s", path, strerror(errno));
        if (fd >= 0) {
            close(fd);
        }
        return NULL;
    }
    if (buf) {
        *buf = malloc(FILE_BUF_LEN);
        if (*buf) {
            setvbuf(f, *buf, _IOFBF, FILE_BUF_LEN);
        }
    }
    if (fputs(header, f) < 0) {
        ESP_LOGE(TAG, "write %s failed: %s", path, strerror(errno));
        fclose(f);
        return NULL;
    }
    return f;
}

esp_err_t store_init(void)
{
    ESP_RETURN_ON_ERROR(card_mount(), TAG, "no card");

    s_q = xQueueCreate(CONFIG_WF_STORE_QUEUE_LEN, sizeof(item_t));
    s_closed = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_q && s_closed, ESP_ERR_NO_MEM, TAG, "queue");
    ESP_RETURN_ON_FALSE(xTaskCreate(writer_task, "sd_write", WRITER_STACK, NULL, WRITER_PRIO, NULL)
                        == pdPASS, ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

esp_err_t store_open(void)
{
    int index = next_index();
    ESP_RETURN_ON_FALSE(index > 0, ESP_FAIL, TAG, "cannot read card");
    s_index = index;

    portENTER_CRITICAL(&s_lock);
    memset(&s_stats, 0, sizeof(s_stats));
    portEXIT_CRITICAL(&s_lock);
    memset(s_track, 0, sizeof(s_track));

    char meta_path[PATH_LEN];
    snprintf(meta_path, sizeof(meta_path), MOUNT_POINT "/meta_%d.csv", index);
    for (int i = 0; i < STORE_NODES; i++) {
        snprintf(s_path[i], sizeof(s_path[i]), MOUNT_POINT "/%s_%d.csv", FILE_PREFIX[i], index);
        s_csv[i] = open_output(s_path[i], CSV_HEADER, &s_buf[i]);
    }
    FILE *meta = open_output(meta_path, "# range_g=8 raw_per_g=4096\n" META_HEADER, NULL);
    if (s_csv[0] == NULL || s_csv[1] == NULL || meta == NULL) {
        if (meta) {
            fclose(meta);
        }
        release_files();
        return ESP_FAIL;
    }

    int64_t now = esp_timer_get_time();
    s_next_sync = now + CONFIG_WF_STORE_SYNC_MS * 1000LL;
    s_next_meta = now + META_PERIOD_US;
    s_meta = meta;  /* gán cuối cùng: task ghi dựa vào đây để biết đã mở file */

    ESP_LOGI(TAG, "file %d opened: %s, %s", index, s_path[0], s_path[1]);
    return ESP_OK;
}

bool store_push_frame(void *owner, const pkt_data_t *d)
{
    item_t it = { .type = ITEM_FRAME, .owner = owner, .d = d };
    bool ok = xQueueSend(s_q, &it, 0) == pdTRUE;
    uint32_t used = (uint32_t)uxQueueMessagesWaiting(s_q);

    portENTER_CRITICAL(&s_lock);
    if (!ok) {
        s_stats.queue_drops++;
    }
    if (used > s_stats.queue_peak) {
        s_stats.queue_peak = used;
    }
    portEXIT_CRITICAL(&s_lock);
    return ok;
}

bool store_close_async(void)
{
    item_t it = { .type = ITEM_CLOSE };
    return xQueueSend(s_q, &it, 0) == pdTRUE;
}

bool store_close_done(void)
{
    return xSemaphoreTake(s_closed, 0) == pdTRUE;
}

int store_index(void)
{
    return s_index;
}

const char *store_node_path(int node_id)
{
    return s_path[node_id - 1];
}

void store_get_stats(store_stats_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}
