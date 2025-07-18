#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "pid_control.h"


/* 初级函数声明 */
void AngleControl_Init(void);
void AngleControl_Reset(void);


/* 功能接口函数声明 */
void SetStraightDrive(float speed, float target_yaw);
void SelfTurnTarget(float target_yaw);


#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_CONTROL_H */
