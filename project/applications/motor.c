#include "motor.h"
#include "board.h"

/* 增量式 PI 参数：100Hz */
PID_Control PID_A = {VEL_KP_NOM, VEL_KI_NOM, 0, 0, 0};
PID_Control PID_B = {VEL_KP_NOM, VEL_KI_NOM, 0, 0, 0};

/* TB6612 方向引脚 */
#define AIN1_PIN    GET_PIN(G, 1)
#define AIN2_PIN    GET_PIN(G, 3)
#define BIN1_PIN    GET_PIN(G, 5)
#define BIN2_PIN    GET_PIN(G, 6)

/* TIM1 双通道 PWM：PA8=CH1(pwmA), PE11=CH2(pwmB) */
#define TRACK_PWM_DEV       "pwm1"
#define TRACK_PWM_CH_A      1
#define TRACK_PWM_CH_B      2
/* TIM1 168MHz / 8000 计数 ≈ 21kHz，满量程对应速度环输出 7999 */
#define TRACK_PWM_FULL_CNT  8000
#define TRACK_PWM_PERIOD_NS 47619

static struct rt_device_pwm *track_pwm_dev = RT_NULL;

void Motor_Init(void)
{
    rt_pin_mode(AIN1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AIN2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BIN1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BIN2_PIN, PIN_MODE_OUTPUT);

    track_pwm_dev = (struct rt_device_pwm *)rt_device_find(TRACK_PWM_DEV);
    if (track_pwm_dev == RT_NULL)
    {
        rt_kprintf("motor: find %s failed!\n", TRACK_PWM_DEV);
        return;
    }

    /* 使能两路 PWM 输出（TIM1 高级定时器需启动输出），之后只改占空比 */
    rt_pwm_enable(track_pwm_dev, TRACK_PWM_CH_A);
    rt_pwm_enable(track_pwm_dev, TRACK_PWM_CH_B);
}

void PID_Reset(void) {
    PID_A.Last_Bias = 0; PID_A.Output = 0;
    PID_B.Last_Bias = 0; PID_B.Output = 0;
}

/*
 * 设置电机 PWM 输出（pwmA=右轮, pwmB=左轮；正=前进）
 * pwm=0 时方向引脚清零，避免过零抖向
 */
void Set_PWM(int pwmA, int pwmB)
{
    if (pwmA > 0) {
        rt_pin_write(AIN1_PIN, PIN_HIGH);
        rt_pin_write(AIN2_PIN, PIN_LOW);
    } else if (pwmA < 0) {
        rt_pin_write(AIN1_PIN, PIN_LOW);
        rt_pin_write(AIN2_PIN, PIN_HIGH);
    } else {
        rt_pin_write(AIN1_PIN, PIN_LOW);
        rt_pin_write(AIN2_PIN, PIN_LOW);
    }

    if (pwmB > 0) {
        rt_pin_write(BIN1_PIN, PIN_HIGH);
        rt_pin_write(BIN2_PIN, PIN_LOW);
    } else if (pwmB < 0) {
        rt_pin_write(BIN1_PIN, PIN_LOW);
        rt_pin_write(BIN2_PIN, PIN_HIGH);
    } else {
        rt_pin_write(BIN1_PIN, PIN_LOW);
        rt_pin_write(BIN2_PIN, PIN_LOW);
    }

    if (track_pwm_dev != RT_NULL)
    {
        rt_pwm_set(track_pwm_dev, TRACK_PWM_CH_A, TRACK_PWM_PERIOD_NS,
                   (rt_uint32_t)((rt_int64_t)ABS(pwmA) * TRACK_PWM_PERIOD_NS / TRACK_PWM_FULL_CNT));
        rt_pwm_set(track_pwm_dev, TRACK_PWM_CH_B, TRACK_PWM_PERIOD_NS,
                   (rt_uint32_t)((rt_int64_t)ABS(pwmB) * TRACK_PWM_PERIOD_NS / TRACK_PWM_FULL_CNT));
    }
}

/* 标准增量式 PI: Out += Kp*(e[k]-e[k-1]) + Ki*e[k] */
int Velocity_PID_Compute(int Target, int Current, PID_Control *PID, int MaxOutput) {
    int Bias = Target - Current;

    PID->Output += (int)(PID->Kp * (Bias - PID->Last_Bias) + PID->Ki * Bias);

    PID->Last_Bias = Bias;
    PID->Output = LIMIT(PID->Output, -MaxOutput, MaxOutput);

    return PID->Output;
}

int Velocity_A(int Target, int Current) {
    return Velocity_PID_Compute(Target, Current, &PID_A, TRACK_PWM_FULL_CNT - 1);
}

int Velocity_B(int Target, int Current) {
    return Velocity_PID_Compute(Target, Current, &PID_B, TRACK_PWM_FULL_CNT - 1);
}
