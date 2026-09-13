#ifndef _BALL_H_
#define _BALL_H_
#include <rtthread.h>
#include "pid.h"
#define COMPENSATE_EN       1
#define COMPENSATE_X_REF    800.0f
#define COMPENSATE_ANG0     183.8f
#define COMPENSATE_L1      (-30.0f)
#define COMPENSATE_R        60.0f
#define COMPENSATE_L2       154.0f
#define DEG2RAD             0.01745329252f
/* 题3 ±5 路径顺序: 1=中点→+5(291)→-5(557); 0=中点→-5(557)→+5(291) */
#define BALL_TASK3_ORDER       0
#if BALL_TASK3_ORDER
#define BALL_TASK_SP_FIRST     291     /* +5cm */
#define BALL_TASK_SP_FINAL     557     /* -5cm 稳住 */
#else
#define BALL_TASK_SP_FIRST     557     /* -5cm */
#define BALL_TASK_SP_FINAL     291     /* +5cm 稳住 */
#endif
#define BALL_TASK_TURN_BAND     10     /* 题3第一目标: |e|<=此值或越过即折返 */

void Ball_Init(void);
void Ball_Start(void);      /* 题3: 由 BALL_TASK3_ORDER 决定 291↔557 */
void Ball_StartHold(void);              /* 题4: 稳住中点 426 */
void Ball_StartHoldAt(int16_t sp_px);   /* 题6: 稳住指定像素 */
void Ball_Stop(void);
void Ball_Control(void);
int16_t Ball_GetPos(void);
int32_t Ball_GetTarget(void);
int16_t Ball_GetErr(void);
int16_t Ball_GetSetpoint(void);   /* 当前设定位置目标 (px) */
uint8_t Ball_IsT6Pos(void);        /* 题6正向目标 (PID_D + 输出衰减生效中) */
/* Kv 倍率: 1.0 恢复标称; >1 加速/减速段增益, 恢复前再次设置会基于标称值 */
void Ball_SetKvScale(float scale);
/* dir: -1缓启补偿, +1缓停补偿, 0请求清除(再保持 HOLD_MS); 仅任务6正向 */
void Ball_SetSoftFf(int8_t dir);
void Ball_SoftFfTick(uint16_t dt_ms); /* 节拍调用, 处理清除延时 */
/* 任务6正向专用: 缓启/出弯、缓停/入弯 Kv 倍率 */
#define BALL_T6_KV_ACCEL  1.3f
#define BALL_T6_KV_DECEL  1.2f
/* 任务6正向缓启/缓停杆角前馈 (0.01°); 负=往+侧推, 正=往-侧推 */
#define BALL_T6_SOFT_FF_ACCEL_CDEG  100  /* 缓启 1.00° */
#define BALL_T6_SOFT_FF_DECEL_CDEG   50  /* 缓停 0.50° */
#define BALL_T6_SOFT_FF_HOLD_MS     400u /* 缓启/缓停结束后再保持 */

#endif  /* _BALL_H_ */
