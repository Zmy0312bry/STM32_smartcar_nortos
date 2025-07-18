#include "angle_control.h"
#include "pid_control.h"
#include "jy61p.h"
#include <math.h>

/**
 * @brief 初始化角度控制系统（仅支持Yaw轴）
 * @retval None
 */
void AngleControl_Init(void)
{
    /* 初始化角度PID控制器 */
    AnglePID_Init();
}

/**
 * @brief 重置角度控制系统（仅Yaw轴）
 * @retval None
 */
void AngleControl_Reset(void)
{
    /* 重置PID状态 */
    AnglePID_Reset();
    /* 同时更新PID目标值与单纯初始化不同，设置当前角度，防止突然行动 */
    AnglePID_SetTarget(My_YawAngle);
}

/**
 * @brief 设置直行模式（保持偏航角为0或指定角度）
 * @param speed: 直行速度(RPM)
 * @param target_yaw: 目标偏航角(度)，默认0为直行
 * @retval None
 */
void SetStraightDrive(float speed, float target_yaw)
{
    AnglePID_Reset(); // 重置PID状态
    /* 更新控制状态 */
    angle_pid_yaw.angle_mode = PID_MODE_STRAIGHT_DRIVE; // 设置角度模式为直行驱动
    angle_pid_yaw.base_speed = speed;
    /* 角度PID设置目标值 */
    AnglePID_SetTarget(target_yaw);
}

/**
 * @brief 设置目标偏航角
 * @param target_yaw: 目标Yaw角度(度)
 * @retval None
 */
void SelfTurnTarget(float target_yaw)
{
    AnglePID_Reset();
    /* 更新控制状态 */
    angle_pid_yaw.angle_mode = PID_MODE_TURN_IN_PLACE; // 设置角度模式为原地转向
    angle_pid_yaw.base_speed = 0.0f;                   // 原地转向，基础速度为0
    /* 角度PID设置目标值 */
    AnglePID_SetTarget(target_yaw);
}