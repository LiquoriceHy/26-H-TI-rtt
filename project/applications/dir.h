#ifndef _DIR_H
#define _DIR_H
#include <rtthread.h>
#include <rtdevice.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    int   Last_Error;
} Positional_PID;

/* 黑线判定阈值（0~255），UI 显示与循迹判定共用 */
#define GRAY_THRESHOLD 17

/* 转向共享参数: dir.c 初值/限幅 与 tracking.c 任务参数共用, 单一来源防失配 */
#define TRACK_KP_BASE     (0.25f * 2.0f / 3.0f)  /* 循迹转向标称 Kp */
#define TURN_LIMIT_RATIO  0.85f                   /* 转向输出限幅 = 基速×此比例 */

extern int Tracking_Error;
extern volatile int Turn_Speed;
extern volatile uint8_t Tracking_LineLost;
extern Positional_PID Track_PID;

void Tracking_Get_Error(void);
void Tracking_PID_Compute(void);
void Tracking_Reset(void);
void Tracking_ClearLineLost(void);

/* 启停线：多路同时有效 + 防抖 */
uint8_t Tracking_ActiveCount(void);
void StopLine_Update(void);
uint8_t StopLine_IsPresent(void);

#endif /* _DIR_H */
