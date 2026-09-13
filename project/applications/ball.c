/*
 * 控球 PID
 * K230 视觉位置 → PID → 几何反解 → PTZ 绝对角度(内部限幅钳制)
 */
#include "ball.h"
#include "comm.h"
#include "ptz.h"
#include <stdio.h>
#include <math.h>

typedef enum
{
    BALL_TASK_IDLE = 0,
    BALL_TASK_TO_POS,    /* 第一目标, 抵达/越过后切折返目标 */
    BALL_TASK_DONE       /* 目标锁定 FINAL, PID继续稳住 */
} BallTask_t;

static PID_t pid;
static float u = 0.0f;
static int16_t pos = 0;
static int16_t last_valid_pos = PID_SETPOINT;
static int32_t ptz_target = PTZ_ANGLE_MID;

static BallTask_t task = BALL_TASK_IDLE;
static uint8_t rod_cmd_en = 0;  /* 题4: PID按杆角u, 几何反解电机 */
static uint8_t kv_boost_on;
static float kv_boost_saved;
/* 仅任务6正向: 角度输出×(1-cm/12); 0/负侧及其他任务=1 */
static float u_pos_scale = 1.0f;
static uint8_t task6_pos_en; /* 任务6且目标>0cm: 用 PID_D + 专用 Kv 增益 */
/* 任务6正向缓启/缓停角度前馈 (0.01°), 叠在闭环输出上 */
static int16_t t6_soft_ff_cdeg;
static uint16_t t6_soft_ff_hold_ms; /* >0: 延时清除倒计时 */

/* +12cm→0 每5mm, 负侧由调用方对称 */
static const uint16_t k_px_pos12[25] = {
    105, 118, 134, 143, 156, 173, 186, 198, 211, 226,
    237, 252, 266, 278, 291, 307, 317, 333, 343, 362,
    372, 387, 397, 413, 426
};

/* 正向像素 → cm; px>=中点返回 0 */
static float Ball_PxToPosCm(int16_t px)
{
    uint8_t i, best;
    int best_d, d;

    if (px >= PID_SETPOINT) {
        return 0.0f;
    }
    if (px <= (int16_t)k_px_pos12[0]) {
        return 12.0f;
    }
    best = 24u;
    best_d = 10000;
    for (i = 0; i <= 24u; i++) {
        d = (int)px - (int)k_px_pos12[i];
        if (d < 0) {
            d = -d;
        }
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    /* idx0=+12.0, idx24=0 */
    return 12.0f - 0.5f * (float)best;
}

/* 仅任务6正向: ×(1-cm/12); 中点/负侧/非任务6不衰减 */
static void Ball_UpdatePosScale(int16_t sp_px)
{
    float cm;

    if (sp_px >= PID_SETPOINT) {
        u_pos_scale = 1.0f;
        return;
    }
    cm = Ball_PxToPosCm(sp_px);
    if (cm <= 0.0f) {
        u_pos_scale = 1.0f;
        return;
    }
    if (cm > 12.0f) {
        cm = 12.0f;
    }
    u_pos_scale = 1.0f - cm / 12.0f;
    if (u_pos_scale < 0.05f) {
        u_pos_scale = 0.05f;
    }
}

/* 杆倾角 θ=atan2(num,den) [deg], 与像素补偿同一套几何
 * MID=212.50 为真正水平; θ(MID) 为模型偏置, 控制用 Δθ=θ-θ(MID) 再反解电机 */
static float Ball_RodTheta_deg(float motor_deg)
{
    float rad = (motor_deg - COMPENSATE_ANG0) * DEG2RAD;
    float num = COMPENSATE_L1 + COMPENSATE_R * sinf(rad);
    float den = COMPENSATE_L2 + COMPENSATE_R * cosf(rad);
    return atan2f(num, den) * (1.0f / DEG2RAD);
}

/* 杆角(0.01°) → 电机角(0.01°), 工作区单调二分反解 */
static int32_t Ball_MotorFromRod_cdeg(int32_t theta_cdeg)
{
    float th_des = (float)theta_cdeg * 0.01f;
    int32_t lo = PTZ_ANGLE_MIN;
    int32_t hi = PTZ_ANGLE_MAX;
    uint8_t i;

    for (i = 0; i < 24u; i++)
    {
        int32_t mid = (lo + hi) / 2;
        float th = Ball_RodTheta_deg((float)mid * 0.01f);
        if (th < th_des) lo = mid;
        else hi = mid;
    }
    return (lo + hi) / 2;
}

static int16_t Ball_GeomCorrect(int16_t x0)
{
#if !COMPENSATE_EN
    return x0;
#else
    float x0f, p1, rad, num, den, hyp;
    if (x0 < 0) return x0;
    x0f = (float)x0;
    p1  = (float)ptz_target * 0.01f;
    rad = (p1 - COMPENSATE_ANG0) * DEG2RAD;
    num = COMPENSATE_L1 + COMPENSATE_R * sinf(rad);
    den = COMPENSATE_L2 + COMPENSATE_R * cosf(rad);
    hyp = sqrtf(num * num + den * den);
    if (den <= 1.0f) return x0;
    return (int16_t)(COMPENSATE_X_REF - (COMPENSATE_X_REF - x0f) * hyp / den);
#endif
}

static void Ball_TaskSetSp(float sp)
{
    PID_SetSetpoint(&pid, sp);
    printf("task sp=%.0f\r\n", sp);
}

/* 第一目标: 进入 ±TURN_BAND 或抵达/越过 → 立刻折返并换 PID */
static void Ball_TaskUpdate(void)
{
    float sp, e, ae;
    uint8_t hit;

    if (task != BALL_TASK_TO_POS) return;

    sp = PID_GetSetpoint(&pid);
    e = sp - (float)last_valid_pos;
    ae = (e < 0.0f) ? -e : e;
    hit = (ae <= (float)BALL_TASK_TURN_BAND) ? 1u : 0u; /* 进带 */
#if BALL_TASK3_ORDER
    if (e >= 0.0f) hit = 1u;   /* →291: 抵达/越过 */
#else
    if (e <= 0.0f) hit = 1u;   /* →557: 抵达/越过 */
#endif
    if (!hit) return;

    task = BALL_TASK_DONE;
#if BALL_TASK3_ORDER
    PID_SetParams(&pid, &PID_PARAMS_B);  /* ORDER=1 折返 →557 */
#else
    PID_SetParams(&pid, &PID_PARAMS_F);  /* ORDER=0 折返 →291 */
#endif
    PID_SetSetpoint(&pid, (float)BALL_TASK_SP_FINAL);
    printf("task turn sp=%.0f e=%.0f\r\n", (float)BALL_TASK_SP_FINAL, e);
}

void Ball_Init(void)
{
    rod_cmd_en = 0;
    u = 0.0f;
    pos = 0;
    last_valid_pos = PID_SETPOINT;
    ptz_target = PTZ_ANGLE_MID;
    task = BALL_TASK_IDLE;
    u_pos_scale = 1.0f;
    task6_pos_en = 0;
    t6_soft_ff_cdeg = 0;
    t6_soft_ff_hold_ms = 0;
    PID_Init(&pid, PID_SETPOINT);
}

void Ball_Start(void)
{
    rod_cmd_en = 0;
    u_pos_scale = 1.0f;
    task6_pos_en = 0;
    t6_soft_ff_cdeg = 0;
    t6_soft_ff_hold_ms = 0;
    K230_GetReady();
    pos = Ball_GeomCorrect(K230_GetPos());
    PID_Init(&pid, pos);
    task = BALL_TASK_TO_POS;
#if BALL_TASK3_ORDER
    PID_SetParams(&pid, &PID_PARAMS_A);  /* ORDER=1 第一段 →291 */
#else
    PID_SetParams(&pid, &PID_PARAMS_E);  /* ORDER=0 第一段 →557 */
#endif
    Ball_TaskSetSp((float)BALL_TASK_SP_FIRST);
}


void Ball_StartHold(void)
{
    Ball_StartHoldAt(PID_SETPOINT);
}

void Ball_StartHoldAt(int16_t sp_px)
{
    rod_cmd_en = 1;  /* 题4/6: 杆角空间对称控制 */
    K230_GetReady();
    pos = Ball_GeomCorrect(K230_GetPos());
    PID_Init(&pid, pos);
    task = BALL_TASK_DONE;
    t6_soft_ff_cdeg = 0;
    t6_soft_ff_hold_ms = 0;
    if (sp_px < PID_SETPOINT) {
        /* 任务6正向: 专用 PID_D + (1-cm/12) 输出衰减 */
        task6_pos_en = 1;
        PID_SetParams(&pid, &PID_PARAMS_D);
        Ball_UpdatePosScale(sp_px);
    } else {
        /* 中点/负侧: 题4/5 参数 */
        task6_pos_en = 0;
        u_pos_scale = 1.0f;
        PID_SetParams(&pid, &PID_PARAMS_C);
    }
    Ball_TaskSetSp((float)sp_px);
}

void Ball_Stop(void)
{
    Ball_SetKvScale(1.0f);
    rod_cmd_en = 0;
    u_pos_scale = 1.0f;
    task6_pos_en = 0;
    t6_soft_ff_cdeg = 0;
    t6_soft_ff_hold_ms = 0;
    PTZ_SetAbsAngle(PTZ_ANGLE_MID);
    ptz_target = PTZ_ANGLE_MID;
    task = BALL_TASK_IDLE;
    PID_SetParams(&pid, &PID_PARAMS_A);
    PID_SetSetpoint(&pid, (float)PID_SETPOINT);
}

void Ball_Control(void)
{
    if (K230_GetReady())
    {
        pos = Ball_GeomCorrect(K230_GetPos());
        if (pos >= 0)
        {
            last_valid_pos = pos;
            u = PID_Calc(&pid, pos);
            /* 任务6正向: 角度×(1-cm/12); 负侧/其他任务不乘 */
            if (u_pos_scale < 0.999f) {
                u *= u_pos_scale;
            }
            /* 缓启/缓停前馈叠在闭环上 (不参与 pos_scale 衰减) */
            if (t6_soft_ff_cdeg != 0) {
                u += (float)t6_soft_ff_cdeg;
            }
            if (rod_cmd_en)
            {
                float th_mid = Ball_RodTheta_deg((float)PTZ_ANGLE_MID * 0.01f);
                int32_t th_cmd = (int32_t)(th_mid * 100.0f + u);
                ptz_target = Ball_MotorFromRod_cdeg(th_cmd);
            }
            else
            {
                ptz_target = PTZ_ANGLE_MID + (int32_t)u;
            }
            if (ptz_target > PTZ_ANGLE_MAX) ptz_target = PTZ_ANGLE_MAX;
            if (ptz_target < PTZ_ANGLE_MIN) ptz_target = PTZ_ANGLE_MIN;
            PTZ_SetAbsAngle(ptz_target);

        }
    }
    else
    {
        pos = Ball_GeomCorrect(K230_GetPos());
    }
    if (task != BALL_TASK_IDLE && task != BALL_TASK_DONE)
    {
        Ball_TaskUpdate();
    }
}

uint8_t Ball_IsT6Pos(void) { return task6_pos_en; }

/* scale: 1.0 恢复标称 Kv; >1 加速/减速段倍率 (调用方保证题6正向用 BALL_T6_KV_*) */
void Ball_SetKvScale(float scale)
{
    if (scale < 1.01f) {
        if (kv_boost_on) {
            pid.p.kv = kv_boost_saved;
            kv_boost_on = 0;
        }
        return;
    }
    if (!kv_boost_on) {
        kv_boost_saved = pid.p.kv;
        kv_boost_on = 1;
    }
    pid.p.kv = kv_boost_saved * scale;
}

/* 仅任务6正向: 缓启 -1°, 缓停 +0.5°; dir=0 再保持 HOLD_MS 后清除 */
void Ball_SetSoftFf(int8_t dir)
{
    if (!task6_pos_en) {
        t6_soft_ff_cdeg = 0;
        t6_soft_ff_hold_ms = 0;
        return;
    }
    if (dir == 0) {
        /* 缓启/缓停结束: 不立刻清零, 再保持一段 */
        if (t6_soft_ff_cdeg != 0 && t6_soft_ff_hold_ms == 0) {
            t6_soft_ff_hold_ms = BALL_T6_SOFT_FF_HOLD_MS;
            printf("t6 soft_ff hold %u ms\r\n",
                   (unsigned)BALL_T6_SOFT_FF_HOLD_MS);
        }
        return;
    }
    t6_soft_ff_hold_ms = 0;
    /* 负u→往+侧推(抗加速); 正u→往-侧推(抗减速) */
    t6_soft_ff_cdeg = (dir < 0) ? (int16_t)(-BALL_T6_SOFT_FF_ACCEL_CDEG)
                                : (int16_t)BALL_T6_SOFT_FF_DECEL_CDEG;
    printf("t6 soft_ff=%d\r\n", (int)t6_soft_ff_cdeg);
}

void Ball_SoftFfTick(uint16_t dt_ms)
{
    if (t6_soft_ff_hold_ms == 0 || dt_ms == 0) {
        return;
    }
    if (t6_soft_ff_hold_ms > dt_ms) {
        t6_soft_ff_hold_ms = (uint16_t)(t6_soft_ff_hold_ms - dt_ms);
        return;
    }
    t6_soft_ff_hold_ms = 0;
    t6_soft_ff_cdeg = 0;
    printf("t6 soft_ff end\r\n");
}

int16_t Ball_GetPos(void)      { return pos; }
int32_t Ball_GetTarget(void)   { return ptz_target; }
int16_t Ball_GetSetpoint(void) { return (int16_t)PID_GetSetpoint(&pid); }
int16_t Ball_GetErr(void)      { return Ball_GetSetpoint() - last_valid_pos; }
