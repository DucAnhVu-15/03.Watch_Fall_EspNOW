/*
 * Ghi dữ liệu 2 node xuống thẻ SD dưới dạng CSV.
 *
 * Mỗi lần ghi là một bộ file đánh số 1, 2, 3... ở thư mục gốc của thẻ:
 *   right_hand_N.csv  dữ liệu raw của node 1 (Master, tay phải)
 *   left_hand_N.csv   dữ liệu raw của node 2 (Slave, tay trái)
 *   meta_N.csv        thống kê định kỳ, vẫn đọc được nếu mất nguồn giữa chừng
 * Số N tiếp nối số lớn nhất đang có trên thẻ, file cũ không bao giờ bị ghi đè.
 *
 * store_push_frame() không chặn nên gọi được từ task nhận ESP-NOW. Việc ghi nằm ở
 * một task ưu tiên thấp vì thẻ SD đôi lúc khựng hàng trăm ms.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "proto.h"

#define STORE_NODES PROTO_NODES

typedef struct {
    uint32_t frames;
    uint32_t lines;         /* số dòng dữ liệu đã ghi, gồm cả dòng overlap */
    uint32_t lost_samples;  /* số mẫu thiếu, tính từ last_seq              */
    uint32_t restarts;      /* số lần last_seq giảm: node khởi động lại    */
    uint32_t fifo_ovf;
    uint32_t ring_drop;
} store_node_stats_t;

typedef struct {
    store_node_stats_t node[STORE_NODES];
    uint32_t queue_drops;   /* khung bị từ chối vì hàng đợi đầy */
    uint32_t write_errors;
    uint32_t max_frame_ms;  /* thời gian ghi 1 khung lâu nhất   */
    uint32_t queue_peak;
} store_stats_t;

/* Mount thẻ và chạy task ghi. Không bao giờ format thẻ. */
esp_err_t store_init(void);

/* Mở bộ file có số kế tiếp */
esp_err_t store_open(void);

/*
 * Đưa khung vào hàng đợi ghi, không chặn. Trả về true thì store giữ quyền sở hữu
 * owner và sẽ free(owner) sau khi ghi; d phải nằm trong vùng nhớ của owner.
 * Trả về false thì người gọi vẫn giữ owner.
 */
bool store_push_frame(void *owner, const pkt_data_t *d);

/*
 * Xếp lệnh đóng file sau các khung đang chờ, không chặn để beacon không bị trễ.
 * Trả về false nếu hàng đợi đầy, gọi lại sau. Dùng store_close_done() để biết đã xong.
 */
bool store_close_async(void);
bool store_close_done(void);

/* Số của bộ file hiện tại hoặc gần nhất */
int store_index(void);

/* Đường dẫn file CSV của node (1 hoặc 2) trong lần ghi hiện tại hoặc gần nhất */
const char *store_node_path(int node_id);
void store_get_stats(store_stats_t *out);
