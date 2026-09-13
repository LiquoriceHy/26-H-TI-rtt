#ifndef _PID_H_
#define _PID_H_
#include <rtthread.h>

/* 中点 (px, 几何校正后); 任务目标相对此中点 */
#define PID_SETPOINT        426

#define PID_VEL_FRAMES      5      /* 速度差分: 本帧相对前 N 帧 */
#define PID_HIST_LEN        8      /* 位置环形缓冲长度 */

/* ---------- 题3 ORDER=1: 中点→+5(291)→-5(557) ---------- */
/* 阶段A: 去往 291/+5 (较快到位) */
#define PID_A_KP             9.5f  /* 位置比例 */
#define PID_A_KI             0.5f  /* 积分增益 (每拍累加 ki*e) */
#define PID_A_KD             0.0f  /* 位置微分: kd*(e-e_prev) */
#define PID_A_KV           -37.0f  /* 速度阻尼, 负=阻尼 */
#define PID_A_I_BAND        50.0f  /* |e|>=带外清积分 */
#define PID_A_I_DEAD        10.0f  /* |e|<=死区: 不累加, 保持I */
#define PID_A_INT_LIMIT    300.0f  /* 积分限幅 (0.01°) */
#define PID_A_OUT_LIMIT   1300.0f  /* 输出限幅 (0.01°) */

/* 阶段B: 去往 557/-5 并稳住 */
#define PID_B_KP             7.5f
#define PID_B_KI             0.3f
#define PID_B_KD             0.0f
#define PID_B_KV           -37.0f
#define PID_B_I_BAND        50.0f
#define PID_B_I_DEAD        10.0f
#define PID_B_INT_LIMIT    300.0f
#define PID_B_OUT_LIMIT   1300.0f

/* ---------- 题3 ORDER=0: 中点→-5(557)→+5(291), 与 ORDER=1 独立整定 ---------- */
/* 阶段E: 去往 557/-5 (较快到位) */
#define PID_E_KP             11.5f
#define PID_E_KI             0.3f
#define PID_E_KD             0.0f
#define PID_E_KV           -38.0f
#define PID_E_I_BAND        50.0f
#define PID_E_I_DEAD        10.0f
#define PID_E_INT_LIMIT    300.0f
#define PID_E_OUT_LIMIT   1300.0f

/* 阶段F: 去往 291/+5 并稳住 */
#define PID_F_KP             6.0f
#define PID_F_KI             0.2f
#define PID_F_KD             0.0f
#define PID_F_KV           -35.0f
#define PID_F_I_BAND        50.0f
#define PID_F_I_DEAD        10.0f
#define PID_F_INT_LIMIT    300.0f
#define PID_F_OUT_LIMIT   1300.0f

/* ---------- 题4: A→B 行驶, 稳住中点 426 (单独整定) ---------- */
#define PID_C_KP             5.0f  /* 位置比例 */
#define PID_C_KI             0.3f  /* 积分增益 */
#define PID_C_KD             0.0f  /* 位置微分 */
#define PID_C_KV           -17.0f  /* 速度阻尼基准 (负=阻尼) */
#define PID_C_I_BAND        50.0f  /* 积分带 */
#define PID_C_I_DEAD        10.0f  /* 积分死区 */
#define PID_C_INT_LIMIT    100.0f  /* 积分限幅 */

#define PID_C_OUT_LIMIT   1300.0f  /* 输出限幅 */

/* 按 |v| 分阶段 Kv: 低速小、高速大
 *   |v|<=V_LOW → V_LOW_GAIN;  LOW~HIGH 线性;  |v|>=V_HIGH → V_HIGH_GAIN
 * 最终 Kv = 基准 * |v|倍率 */
#define PID_C_KV_V_SCHED     1
#define PID_C_KV_V_LOW      20.0f
#define PID_C_KV_V_HIGH     50.0f
#define PID_C_KV_V_LOW_GAIN  1.0f
#define PID_C_KV_V_HIGH_GAIN 2.0f

/* ---------- 题6 正向目标专用 PID (与题4中点独立整定) ---------- */
#define PID_D_KP             7.0f  /* 位置比例 */
#define PID_D_KI             0.2f  /* 积分增益 */
#define PID_D_KD             0.0f  /* 位置微分 */
#define PID_D_KV           -17.0f  /* 速度阻尼基准 (负=阻尼) */
#define PID_D_I_BAND        50.0f  /* 积分带: |e|>=带外清积分 */
#define PID_D_I_DEAD        10.0f  /* 积分死区: |e|<=死区不累加, 保持I */
#define PID_D_INT_LIMIT    100.0f  /* 积分限幅 (0.01°) */
#define PID_D_OUT_LIMIT   1300.0f  /* 输出限幅 (0.01°) */
/* 按 |v| 分阶段 Kv: 低速小、高速大; 最终 Kv = 基准 * |v|倍率 */
#define PID_D_KV_V_SCHED     1     /* 1=启用按速度调度 */
#define PID_D_KV_V_LOW      20.0f  /* |v|<=此值 → LOW_GAIN */
#define PID_D_KV_V_HIGH     50.0f  /* |v|>=此值 → HIGH_GAIN */
#define PID_D_KV_V_LOW_GAIN  1.0f  /* 低速阻尼倍率 */
#define PID_D_KV_V_HIGH_GAIN 2.0f  /* 高速阻尼倍率 */

/* |e|<=此值时 Ki 乘系数 (不冻结输出, 避免出入死区抖动) */
#define PID_KI_SOFT_DEAD     10.0f
#define PID_KI_SOFT_SCALE    0.6f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kv;
    float i_band;
    float i_dead;
    float int_limit;
    float out_limit;
    /* 按 |v| 分阶段阻尼 */
    uint8_t kv_v_sched;
    float kv_v_low;
    float kv_v_high;
    float kv_v_low_gain;
    float kv_v_high_gain;
} PID_Params_t;

typedef struct
{
    float u;
    float integral;
    float e_prev;
    float setpoint;
    PID_Params_t p;
    int16_t pos_hist[PID_HIST_LEN];
    uint8_t hist_idx;
} PID_t;

extern const PID_Params_t PID_PARAMS_A;  /* 题3 ORDER=1 →291/+5 */
extern const PID_Params_t PID_PARAMS_B;  /* 题3 ORDER=1 →557/-5 */
extern const PID_Params_t PID_PARAMS_E;  /* 题3 ORDER=0 →557/-5 */
extern const PID_Params_t PID_PARAMS_F;  /* 题3 ORDER=0 →291/+5 */
extern const PID_Params_t PID_PARAMS_C;  /* 题4/5/6负 中点 */
extern const PID_Params_t PID_PARAMS_D;  /* 题6 正向目标 */

void PID_Init(PID_t *pid, int16_t pos);
void PID_SetSetpoint(PID_t *pid, float sp);
void PID_SetParams(PID_t *pid, const PID_Params_t *params);
float PID_GetSetpoint(const PID_t *pid);
float PID_Calc(PID_t *pid, int16_t pos);

#endif
