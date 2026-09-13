#ifndef _MOTOR_H
#define _MOTOR_H
#include <rtthread.h>
#include <rtdevice.h>

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* 轮速PI标称增益: 纯循迹(1.Track)与行驶+控球(Drv Mid/Here)独立整定
 * 缓启/缓停段(基速<目标)由 tracking.c 按基速比例衰减 */
#define VEL_KP_NOM  600   /* 1.Track */
#define VEL_KI_NOM  180
#define VEL_KP_BALL 500   /* 行驶+控球, 与 Track 分开调 */
#define VEL_KI_BALL 180

typedef struct {
    int Kp;
    int Ki;
    int Kd;
    int   Last_Bias;
    int   Output;
} PID_Control;

void Motor_Init(void);
void PID_Reset(void);
void Set_PWM(int pwmA,int pwmB);
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);

extern PID_Control PID_A;
extern PID_Control PID_B;

#endif /* _MOTOR_H */
