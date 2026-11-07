#include "timesync.h"
#include "freertos/FreeRTOS.h"

#define WIN_SIZE      8
#define MIN_VALID     3
#define MAX_RTT_US    20000

typedef struct {
    int64_t offset;
    int32_t rtt;
} meas_t;

static meas_t s_win[WIN_SIZE];
static int s_count;
static int s_pos;
static int64_t s_offset;
static int32_t s_rtt = -1;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void timesync_reset(void)
{
    portENTER_CRITICAL(&s_lock);
    s_count = 0;
    s_pos = 0;
    s_offset = 0;
    s_rtt = -1;
    portEXIT_CRITICAL(&s_lock);
}

bool timesync_add(int64_t t1, int64_t t2, int64_t t3, int64_t t4)
{
    int64_t rtt = (t4 - t1) - (t3 - t2);
    if (rtt < 0 || rtt > MAX_RTT_US) {
        return false;
    }
    int64_t offset = ((t2 - t1) + (t3 - t4)) / 2;

    portENTER_CRITICAL(&s_lock);
    s_win[s_pos] = (meas_t){ .offset = offset, .rtt = (int32_t)rtt };
    s_pos = (s_pos + 1) % WIN_SIZE;
    if (s_count < WIN_SIZE) {
        s_count++;
    }
    int best = 0;
    for (int i = 1; i < s_count; i++) {
        if (s_win[i].rtt < s_win[best].rtt) {
            best = i;
        }
    }
    s_offset = s_win[best].offset;
    s_rtt = s_win[best].rtt;
    portEXIT_CRITICAL(&s_lock);
    return true;
}

bool timesync_valid(void)
{
    return s_count >= MIN_VALID;
}

int64_t timesync_to_master(int64_t local_us)
{
    portENTER_CRITICAL(&s_lock);
    int64_t off = s_offset;
    portEXIT_CRITICAL(&s_lock);
    return local_us + off;
}

void timesync_get(int64_t *offset_us, int32_t *rtt_us)
{
    portENTER_CRITICAL(&s_lock);
    *offset_us = s_offset;
    *rtt_us = s_rtt;
    portEXIT_CRITICAL(&s_lock);
}
