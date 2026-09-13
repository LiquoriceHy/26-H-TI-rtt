#include "pid.h"

const PID_Params_t PID_PARAMS_A = {
    PID_A_KP, PID_A_KI, PID_A_KD, PID_A_KV,
    PID_A_I_BAND, PID_A_I_DEAD, PID_A_INT_LIMIT, PID_A_OUT_LIMIT,
    0, 0.0f, 0.0f, 1.0f, 1.0f
};

const PID_Params_t PID_PARAMS_B = {
    PID_B_KP, PID_B_KI, PID_B_KD, PID_B_KV,
    PID_B_I_BAND, PID_B_I_DEAD, PID_B_INT_LIMIT, PID_B_OUT_LIMIT,
    0, 0.0f, 0.0f, 1.0f, 1.0f
};

const PID_Params_t PID_PARAMS_E = {
    PID_E_KP, PID_E_KI, PID_E_KD, PID_E_KV,
    PID_E_I_BAND, PID_E_I_DEAD, PID_E_INT_LIMIT, PID_E_OUT_LIMIT,
    0, 0.0f, 0.0f, 1.0f, 1.0f
};

const PID_Params_t PID_PARAMS_F = {
    PID_F_KP, PID_F_KI, PID_F_KD, PID_F_KV,
    PID_F_I_BAND, PID_F_I_DEAD, PID_F_INT_LIMIT, PID_F_OUT_LIMIT,
    0, 0.0f, 0.0f, 1.0f, 1.0f
};

const PID_Params_t PID_PARAMS_C = {
    PID_C_KP, PID_C_KI, PID_C_KD, PID_C_KV,
    PID_C_I_BAND, PID_C_I_DEAD, PID_C_INT_LIMIT, PID_C_OUT_LIMIT,
    PID_C_KV_V_SCHED, PID_C_KV_V_LOW, PID_C_KV_V_HIGH,
    PID_C_KV_V_LOW_GAIN, PID_C_KV_V_HIGH_GAIN
};

const PID_Params_t PID_PARAMS_D = {
    PID_D_KP, PID_D_KI, PID_D_KD, PID_D_KV,
    PID_D_I_BAND, PID_D_I_DEAD, PID_D_INT_LIMIT, PID_D_OUT_LIMIT,
    PID_D_KV_V_SCHED, PID_D_KV_V_LOW, PID_D_KV_V_HIGH,
    PID_D_KV_V_LOW_GAIN, PID_D_KV_V_HIGH_GAIN
};

void PID_Init(PID_t *pid, int16_t pos)
{
    pid->u = 0.0f;
    pid->integral = 0.0f;
    pid->e_prev = 0.0f;
    pid->setpoint = (float)PID_SETPOINT;
    pid->p = PID_PARAMS_A;
    for (uint8_t i = 0; i < PID_HIST_LEN; i++) pid->pos_hist[i] = pos;
    pid->hist_idx = 0;
}

void PID_SetSetpoint(PID_t *pid, float sp)
{
    pid->setpoint = sp;
    pid->integral = 0.0f;
    pid->e_prev = 0.0f;
}

void PID_SetParams(PID_t *pid, const PID_Params_t *params)
{
    pid->p = *params;
    pid->integral = 0.0f;
    pid->e_prev = 0.0f;
}

float PID_GetSetpoint(const PID_t *pid)
{
    return pid->setpoint;
}

/* 按 |v|: 低速 low_gain, 高速 high_gain, 中间线性 */
static float PID_KvScaleVel(const PID_Params_t *p, float av)
{
    float lo, hi, glo, ghi, t;

    if (!p->kv_v_sched) return 1.0f;

    lo = p->kv_v_low;
    hi = p->kv_v_high;
    glo = p->kv_v_low_gain;
    ghi = p->kv_v_high_gain;
    if (av <= lo) return glo;
    if (av >= hi || hi <= lo) return ghi;
    t = (av - lo) / (hi - lo);
    return glo + (ghi - glo) * t;
}

float PID_Calc(PID_t *pid, int16_t pos)
{
    pid->pos_hist[pid->hist_idx] = pos;
    uint8_t old_idx = (uint8_t)((pid->hist_idx + PID_HIST_LEN - PID_VEL_FRAMES) % PID_HIST_LEN);
    float v = (float)(pos - pid->pos_hist[old_idx]);
    pid->hist_idx = (uint8_t)((pid->hist_idx + 1) % PID_HIST_LEN);

    float e = pid->setpoint - (float)pos;
    float ae = (e < 0.0f) ? -e : e;
    float av = (v < 0.0f) ? -v : v;
    float de = e - pid->e_prev;
    pid->e_prev = e;

    if (ae >= pid->p.i_band)
    {
        pid->integral = 0.0f;
    }
    else if (ae <= PID_KI_SOFT_DEAD)
    {
        /* 近端死区: Ki 减半, 仍积分、仍输出 */
        pid->integral += pid->p.ki * PID_KI_SOFT_SCALE * e;
        if (pid->integral >  pid->p.int_limit) pid->integral =  pid->p.int_limit;
        if (pid->integral < -pid->p.int_limit) pid->integral = -pid->p.int_limit;
    }
    else if (ae > pid->p.i_dead)
    {
        pid->integral += pid->p.ki * e;
        if (pid->integral >  pid->p.int_limit) pid->integral =  pid->p.int_limit;
        if (pid->integral < -pid->p.int_limit) pid->integral = -pid->p.int_limit;
    }
    /* i_soft < |e| <= i_dead: 保持 I */

    float kv = pid->p.kv * PID_KvScaleVel(&pid->p, av);
    float u = pid->p.kp * e + pid->integral + pid->p.kd * de + kv * v;
    if (u >  pid->p.out_limit) u =  pid->p.out_limit;
    if (u < -pid->p.out_limit) u = -pid->p.out_limit;
    pid->u = u;
    return u;
}
