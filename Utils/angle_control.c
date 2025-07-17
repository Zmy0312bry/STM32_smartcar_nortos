#include "angle_control.h"
#include "pid_control.h"
#include "jy61p.h"
#include <math.h>

/* 角度控制全局实例 */
AngleControl_TypeDef angle_control;

/**
 * @brief 初始化角度控制系统（仅支持Yaw轴）
 * @retval None
 */
void AngleControl_Init(void)
{
    /* 初始化角度控制结构体 */
    angle_control.mode = ANGLE_CONTROL_DISABLED;
    angle_control.enabled = 0;
    angle_control.target_yaw = 0.0f;
    angle_control.current_yaw = 0.0f;
    angle_control.yaw_sensitivity = 5.0f;
    
    /* 初始化角度PID控制器 */
    AnglePID_Init();
    AngleControl_Enable();
    /* 设置初始目标值 */
    AnglePID_SetTarget(0.0f);
}

/**
 * @brief 启用角度控制（仅Yaw轴）
 * @retval None
 */
void AngleControl_Enable(void)
{
    angle_control.enabled = 1;
    angle_control.mode = ANGLE_CONTROL_YAW;
    /* 启用角度控制 */
    angle_pid_yaw.enabled = 1;
    pid_control_state.angle_control_active = 1;
}

/**
 * @brief 禁用角度控制（仅Yaw轴）
 * @retval None
 */
void AngleControl_Disable(void)
{
    angle_control.enabled = 0;
    angle_control.mode = ANGLE_CONTROL_DISABLED;
    
    /* 禁用并重置PID状态 */
    angle_pid_yaw.enabled = 0;
    AnglePID_Reset();
}

/**
 * @brief 设置目标偏航角
 * @param target_yaw: 目标Yaw角度(度)
 * @retval None
 */
void AngleControl_SelfTurnTarget(float target_yaw)
{
    /* 设置为原地转向模式 */
    PID_SetTurnInPlace(target_yaw);
}

/**
 * @brief 设置控制灵敏度
 * @param yaw_sensitivity: Yaw轴控制灵敏度 (0.1 - 3.0)
 * @retval None
 */
void AngleControl_SetSensitivity(float yaw_sensitivity)
{
    /* 限制灵敏度范围 */
    if (yaw_sensitivity < 0.1f) yaw_sensitivity = 0.1f;
    if (yaw_sensitivity > 3.0f) yaw_sensitivity = 3.0f;
    
    angle_control.yaw_sensitivity = yaw_sensitivity;
    
    /* 动态调整PID参数 */
    AnglePID_SetSensitivity(yaw_sensitivity);
}

/**
 * @brief 重置角度控制系统（仅Yaw轴）
 * @retval None
 */
void AngleControl_Reset(void)
{
    /* 重置目标角度为当前角度 */
    angle_control.target_yaw = MPU6050.Yaw_Custom;
    angle_control.current_yaw = MPU6050.Yaw_Custom;
    
    /* 更新PID目标值 */
    AnglePID_SetTarget(MPU6050.Yaw_Custom);
    
    /* 重置PID状态 */
    AnglePID_Reset();
}

/**
 * @brief 获取角度控制状态（仅Yaw轴）
 * @retval 角度控制结构体指针
 */
AngleControl_TypeDef* AngleControl_GetStatus(void)
{
    /* 更新当前角度 */
    angle_control.current_yaw = MPU6050.Yaw_Custom;
    return &angle_control;
}

/**
 * @brief 设置直行模式（保持偏航角为0或指定角度）
 * @param speed: 直行速度(RPM)
 * @param target_yaw: 目标偏航角(度)，默认0为直行
 * @retval None
 */
void AngleControl_SetStraightDrive(float speed, float target_yaw)
{
    angle_control.target_yaw = target_yaw;
    angle_control.enabled = 1;
    angle_control.mode = ANGLE_CONTROL_YAW;
    
    /* 使用PID系统的直行驱动模式 */
    PID_SetStraightDrive(speed, target_yaw);
}

/**
 * @brief 设置直行模式（偏航角保持为0）
 * @param speed: 直行速度(RPM)
 * @retval None
 */
void AngleControl_SetStraightDriveZero(float speed)
{
    AngleControl_SetStraightDrive(speed, 0.0f);
}

/**
 * @brief 检查角度是否已到达目标（容差范围内）
 * @param tolerance: 角度容差(度)
 * @retval 1: 已到达, 0: 未到达
 */
uint8_t AngleControl_IsTargetReached(float tolerance)
{
    if (!angle_control.enabled) return 0;
    
    float angle_error = fabsf(angle_control.target_yaw - MPU6050.Yaw_Custom);
    
    /* 处理角度环绕问题 */
    if (angle_error > 180.0f) {
        angle_error = 360.0f - angle_error;
    }
    
    return (angle_error <= tolerance) ? 1 : 0;
}
