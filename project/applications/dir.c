#include "dir.h"
#include "gray_module.h"
#include "motor.h"

extern volatile int Target_Speed;

int Tracking_Error = 0;
volatile int Turn_Speed = 0;
volatile uint8_t Tracking_LineLost = 0;

/* 纯 Kp */
Positional_PID Track_PID = {TRACK_KP_BASE, 0.0f, 0.0f, 0};

/* 粗横线：有效探头路数达到该值视为启停线
 * 判定: 最近3拍窗口内出现≥1拍有效即置位, 并保持整个窗口 (~30ms)。
 * 高速扫过横线可能只有1-2拍有效, 必须"命中即锁存"才不漏检;
 * 锁存≥2个控制周期保证控制线程一定能采到 */
#define STOPLINE_ACTIVE_MIN 4
#define STOPLINE_WIN_TICKS  3
#define STOPLINE_WIN_HIT    1
/* 连续丢线若干拍后停车，防单帧噪声 */
#define LINE_LOST_DEBOUNCE  10   /* ×10ms拍 = 100ms */

/* 转向分档增益: 中心区线性保稳定, 大误差区加陡保跟弯
 *   |err| <= KNEE : u = Kp*err          (直线/小偏差, 与原纯P一致)
 *   |err| >  KNEE : u = Kp*KNEE + KP_BOOST*(|err|-KNEE)  (深弯灌差速) */
#define TRACK_ERR_KNEE   20       /* 分档点 (误差满量程±40) */
#define TRACK_KP_BOOST   0.50f    /* 深弯区增益: 弯中仍外甩就加大 */

static int last_error = 0;
static float err_filter_y = 0.0f;
static uint8_t line_lost_cnt = 0;

void Tracking_Reset(void)
{
    Tracking_Error = 0;
    Turn_Speed = 0;
    Tracking_LineLost = 0;
    last_error = 0;
    err_filter_y = 0.0f;
    line_lost_cnt = 0;
    Track_PID.Last_Error = 0;
}

void Tracking_ClearLineLost(void)
{
    Tracking_LineLost = 0;
    line_lost_cnt = 0;
}

void Tracking_Get_Error(void)
{
    /* G0 在车体右端、G7 在左端：线偏右(G0见线)误差为正 → 右转修正 */
    const int weights[8] = {40, 30, 20, 10, -10, -20, -30, -40};
    int32_t sum_value_weight = 0;
    int32_t sum_value = 0;
    uint8_t valid_count = 0;
    int current_error = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (Gray_Value[i] > GRAY_THRESHOLD) {
            sum_value_weight += (int32_t)Gray_Value[i] * weights[i];
            sum_value += Gray_Value[i];
            valid_count++;
        }
    }

    if (valid_count > 0) {
        current_error = sum_value_weight / sum_value;
        last_error = current_error;
        line_lost_cnt = 0;
        Tracking_LineLost = 0;
    } else {
        /* 丢线：累计后置位停车标志 */
        current_error = 0;
        if (line_lost_cnt < 255) {
            line_lost_cnt++;
        }
        if (line_lost_cnt >= LINE_LOST_DEBOUNCE) {
            Tracking_LineLost = 1;
        }
    }

    Tracking_Error = current_error;
}

uint8_t Tracking_ActiveCount(void)
{
    uint8_t n = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (Gray_Value[i] > GRAY_THRESHOLD) {
            n++;
        }
    }
    return n;
}

static uint8_t stopline_present;
static uint8_t stopline_hist;   /* 最近 STOPLINE_WIN_TICKS 拍的≥4路有效记录 */

void StopLine_Update(void)
{
    uint8_t hit = (Tracking_ActiveCount() >= STOPLINE_ACTIVE_MIN) ? 1u : 0u;
    uint8_t m, bits;

    stopline_hist = (uint8_t)((stopline_hist << 1) | hit) &
                    (uint8_t)((1u << STOPLINE_WIN_TICKS) - 1u);
    for (m = stopline_hist, bits = 0; m != 0u; m >>= 1) {
        bits = (uint8_t)(bits + (m & 1u));
    }
    stopline_present = (bits >= STOPLINE_WIN_HIT) ? 1u : 0u;
}

uint8_t StopLine_IsPresent(void)
{
    return stopline_present;
}

/* 73：y = 0.73*原始 + 0.27*上次滤波 */
static int Error_Filter_73(int raw)
{
    err_filter_y = 0.73f * (float)raw + 0.27f * err_filter_y;

    if (err_filter_y >= 0.0f) {
        return (int)(err_filter_y + 0.5f);
    }
    return (int)(err_filter_y - 0.5f);
}

void Tracking_PID_Compute(void)
{
    float u;
    int turn_speed;
    int turn_limit;
    int err_f;

    if (Tracking_LineLost) {
        Turn_Speed = 0;
        return;
    }

    err_f = Error_Filter_73(Tracking_Error);

    /* 分档 P: 深弯时 Kp*e 已跟不上曲率, 大误差区换陡增益 */
    {
        float ae = (err_f >= 0) ? (float)err_f : -(float)err_f;
        float s  = (err_f >= 0) ? 1.0f : -1.0f;

        if (ae <= (float)TRACK_ERR_KNEE) {
            u = Track_PID.Kp * ae;
        } else {
            u = Track_PID.Kp * (float)TRACK_ERR_KNEE +
                TRACK_KP_BOOST * (ae - (float)TRACK_ERR_KNEE);
        }
        u *= s;
    }
    Track_PID.Last_Error = err_f;

    if (u >= 0.0f) {
        turn_speed = (int)(u + 0.5f);
    } else {
        turn_speed = (int)(u - 0.5f);
    }

    turn_limit = (int)(Target_Speed * TURN_LIMIT_RATIO);
    if (turn_limit < 1) {
        turn_limit = 1;
    }
    turn_speed = LIMIT(turn_speed, -turn_limit, turn_limit);
    Turn_Speed = turn_speed;
}
