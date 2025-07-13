#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "pid_control.h"

/* 角度控制模式枚举（仅支持Yaw轴） */
typedef enum
{
    ANGLE_CONTROL_DISABLED = 0,  // 角度控制关闭
    ANGLE_CONTROL_YAW = 1        // Yaw轴控制启用
} AngleControlMode_TypeDef;

/* 角度控制结构体（仅支持Yaw轴） */
typedef struct
{
    AngleControlMode_TypeDef mode;     // 控制模式
    uint8_t enabled;                   // 是否启用角度控制
    float target_yaw;                  // 目标Yaw角度
    float current_yaw;                 // 当前Yaw角度
    float yaw_sensitivity;             // 转向灵敏度 (0.1 - 3.0)
} AngleControl_TypeDef;

/* 函数声明（仅支持Yaw轴） */
void AngleControl_Init(void);
void AngleControl_Enable(void);
void AngleControl_Disable(void);
void AngleControl_SetSensitivity(float yaw_sensitivity);
void AngleControl_Reset(void);
void AngleControl_SelfTurnTarget(float target_yaw);
AngleControl_TypeDef* AngleControl_GetStatus(void);

/* 便捷接口函数声明 */
void AngleControl_SetStraightDrive(float speed, float target_yaw);
void AngleControl_SetStraightDriveZero(float speed);
uint8_t AngleControl_IsTargetReached(float tolerance);

/* 外部变量声明 */
extern AngleControl_TypeDef angle_control;

#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_CONTROL_H */
