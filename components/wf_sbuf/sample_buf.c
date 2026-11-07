#include <stdlib.h>
#include "sample_buf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#define CAP CONFIG_WF_SBUF_CAPACITY

static sample_t *s_buf;
static uint32_t s_head;
static SemaphoreHandle_t s_mtx;

static inline uint32_t oldest_locked(void)
{
    return s_head > CAP ? s_head - CAP : 0;
}

esp_err_t sbuf_init(void)
{
    s_buf = calloc(CAP, sizeof(sample_t));
    s_mtx = xSemaphoreCreateMutex();
    if (s_buf == NULL || s_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void sbuf_reset(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_head = 0;
    xSemaphoreGive(s_mtx);
}

void sbuf_push(int64_t t_us, int16_t x, int16_t y, int16_t z)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_buf[s_head % CAP] = (sample_t){ .t_us = t_us, .x = x, .y = y, .z = z };
    s_head++;
    xSemaphoreGive(s_mtx);
}

uint32_t sbuf_head(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint32_t h = s_head;
    xSemaphoreGive(s_mtx);
    return h;
}

uint32_t sbuf_oldest(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint32_t o = oldest_locked();
    xSemaphoreGive(s_mtx);
    return o;
}

bool sbuf_last_time(int64_t *t_us)
{
    bool ok = false;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (s_head > 0) {
        *t_us = s_buf[(s_head - 1) % CAP].t_us;
        ok = true;
    }
    xSemaphoreGive(s_mtx);
    return ok;
}

size_t sbuf_read(uint32_t seq, sample_t *out, size_t n)
{
    size_t cnt = 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (seq >= oldest_locked() && seq < s_head) {
        size_t avail = s_head - seq;
        cnt = n < avail ? n : avail;
        for (size_t i = 0; i < cnt; i++) {
            out[i] = s_buf[(seq + i) % CAP];
        }
    }
    xSemaphoreGive(s_mtx);
    return cnt;
}

bool sbuf_find_time(int64_t t_us, uint32_t *seq)
{
    bool ok = false;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint32_t lo = oldest_locked();
    uint32_t hi = s_head;   /* tìm trong [lo, hi) */
    if (hi > lo && s_buf[(hi - 1) % CAP].t_us >= t_us) {
        while (lo < hi) {
            uint32_t mid = lo + (hi - lo) / 2;
            if (s_buf[mid % CAP].t_us < t_us) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        *seq = lo;
        ok = true;
    }
    xSemaphoreGive(s_mtx);
    return ok;
}
