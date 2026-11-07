#include <math.h>
#include "timesync.h"

#define WIN_SIZE        32      /* 3.2 giây với beacon 100 ms */
#define MIN_VALID       20
#define MAX_RESIDUAL_US 1500.0  /* lệch hơn mức này coi là điểm nhiễu          */
#define MAX_REJECTS     10      /* bị loại liên tiếp = đồng hồ nhảy, ví dụ C3 reset */
#define MAX_SLOPE_ERR   300e-6  /* tinh thể không thể lệch quá mức này         */

static int64_t s_local[WIN_SIZE], s_common[WIN_SIZE];
static int s_count, s_pos, s_rejects;

/* Điểm tham chiếu để giữ số nhỏ khi tính bằng double */
static int64_t s_ref_local, s_ref_common;
static double s_slope = 1.0, s_icept;

void timesync_reset(void)
{
    s_count = s_pos = s_rejects = 0;
    s_slope = 1.0;
    s_icept = 0.0;
}

static void refit(void)
{
    int newest = (s_pos + WIN_SIZE - 1) % WIN_SIZE;
    s_ref_local = s_local[newest];
    s_ref_common = s_common[newest];

    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < s_count; i++) {
        double x = (double)(s_local[i] - s_ref_local);
        double y = (double)(s_common[i] - s_ref_common);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    double n = s_count;
    double den = n * sxx - sx * sx;
    double slope = den != 0.0 ? (n * sxy - sx * sy) / den : 1.0;
    if (fabs(slope - 1.0) > MAX_SLOPE_ERR) {
        slope = 1.0;
    }
    s_slope = slope;
    s_icept = (sy - slope * sx) / n;
}

void timesync_add(int64_t local_us, int64_t common_us)
{
    if (s_count >= MIN_VALID) {
        double res = (double)(common_us - timesync_to_common(local_us));
        if (fabs(res) > MAX_RESIDUAL_US) {
            if (++s_rejects < MAX_REJECTS) {
                return;
            }
            timesync_reset();
        }
    }
    s_rejects = 0;

    s_local[s_pos] = local_us;
    s_common[s_pos] = common_us;
    s_pos = (s_pos + 1) % WIN_SIZE;
    if (s_count < WIN_SIZE) {
        s_count++;
    }
    refit();
}

bool timesync_valid(void)
{
    return s_count >= MIN_VALID;
}

int64_t timesync_to_common(int64_t local_us)
{
    return s_ref_common + (int64_t)(s_slope * (double)(local_us - s_ref_local) + s_icept);
}

int64_t timesync_to_local(int64_t common_us)
{
    return s_ref_local + (int64_t)(((double)(common_us - s_ref_common) - s_icept) / s_slope);
}

double timesync_ppm(void)
{
    return (s_slope - 1.0) * 1e6;
}
