#include "pid_control.h"
#include "encoder.h"
#include "motor.h"
#include "jy61p.h"
#include <math.h>

/* 全局PID参数，可用于调整电机A的PID控制器 */
float pid_kp_a = PID_KP_A;
float pid_ki_a = PID_KI_A;
float pid_kd_a = PID_KD_A;

/* 全局PID参数，可用于调整电机B的PID控制器 */
float pid_kp_b = PID_KP_B;
float pid_ki_b = PID_KI_B;
float pid_kd_b = PID_KD_B;

/* 定义电机PID控制器实例 */
PID_TypeDef pid_motor_a; // 电机A的PID控制器
PID_TypeDef pid_motor_b; // 电机B的PID控制器

/* 定义角度环PID控制器实例（使用专用结构体） */
AnglePID_TypeDef angle_pid_yaw; // Yaw轴角度PID控制器

/* 外部变量 */
extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;

/**
 * @brief 初始化PID控制器
 * @param pid: PID控制器结构体指针
 * @retval None
 */
void PID_Init(PID_TypeDef *pid)
{
    /* 根据控制器类型设置PID参数 */
    if (pid == &pid_motor_a)
    {
        pid->Kp = pid_kp_a;
        pid->Ki = pid_ki_a;
        pid->Kd = pid_kd_a;
    }
    else if (pid == &pid_motor_b)
    {
        pid->Kp = pid_kp_b;
        pid->Ki = pid_ki_b;
        pid->Kd = pid_kd_b;
    }

    /* 重置PID状态 */
    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;

    /* 设置输出限制 */
    pid->output_min = PID_OUTPUT_MIN;
    pid->output_max = PID_OUTPUT_MAX;

    /* 初始状态为停止，确保安全启动 */
    pid->direction = MOTOR_STOP;
}

/**
 * @brief 计算PID控制器的输出
 * @param pid: PID控制器结构体指针
 * @retval PID输出值
 */
float PID_Compute(PID_TypeDef *pid)
{
    /* 计算当前误差 */
    pid->error = pid->setpoint - pid->feedback;

    /* 计算积分项 */
    pid->integral += pid->error;

    /* 积分项限幅，防止积分饱和 */
    if (pid->integral > (pid->output_max / pid->Ki))
        pid->integral = pid->output_max / pid->Ki;
    else if (pid->integral < (pid->output_min / pid->Ki))
        pid->integral = pid->output_min / pid->Ki;

    /* 计算微分项 (使用误差的变化率) */
    pid->derivative = pid->error - pid->last_error;

    /* 计算PID输出 */
    pid->output = pid->Kp * pid->error +
                  pid->Ki * pid->integral +
                  pid->Kd * pid->derivative;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    /* 更新误差记录 */
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    /* 方向信息现在由PID_Update函数根据输出值确定 */
    /* 这里不再设置direction，让PID_Update根据output符号来处理 */

    return pid->output;
}

/**
 * @brief 设置目标电机转速
 * @param motor_id: 电机ID (1-电机A, 2-电机B)
 * @param target_rpm: 目标转速(RPM) - 支持正负值
 * @retval None
 */
void PID_SetSpeed(PID_Motor_ID motor_id, float target_rpm)
{
    float target_rpp = target_rpm * (ENCODER_RESOLUTION * GEAR_RATIO)/60.0f;
    if (motor_id == PID_MOTOR_A)
    {
        pid_motor_a.setpoint = target_rpp;

        /* 如果设置为0，直接停止电机，避免不必要的抖动 */
        if (fabsf(target_rpp) < 0.1f)
        {
            Motor_Stop(1);
            pid_motor_a.integral = 0;           // 重置积分项
            pid_motor_a.direction = MOTOR_STOP; // 更新方向状态
        }
        else
        {
            /* 根据目标转速的符号预设方向（实际方向由PID输出决定） */
            pid_motor_a.direction = (target_rpm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        }
    }
    else if (motor_id == PID_MOTOR_B)
    {
        pid_motor_b.setpoint = target_rpp;

        /* 如果设置为0，直接停止电机，避免不必要的抖动 */
        if (fabsf(target_rpp) < 0.1f)
        {
            Motor_Stop(2);
            pid_motor_b.integral = 0;           // 重置积分项
            pid_motor_b.direction = MOTOR_STOP; // 更新方向状态
        }
        else
        {
            /* 根据目标转速的符号预设方向（实际方向由PID输出决定） */
            pid_motor_b.direction = (target_rpm > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        }
    }
}

/**
 * @brief 更新PID控制器并控制电机
 * @note 此函数应定期调用，与编码器更新频率一致
 * @retval None
 */
void PID_Update(void)
{
    uint16_t pwm_value;

    /* 更新PID控制器的反馈值（保持原始符号，包括负值） */
    pid_motor_a.feedback = encoderA.speed_rpp; // 为编码器转过的格数
    pid_motor_b.feedback = encoderB.speed_rpp; // 为编码器转过格数

    /* 更新PID控制器参数（以防被动态修改） */
    pid_motor_a.Kp = pid_kp_a;
    pid_motor_a.Ki = pid_ki_a;
    pid_motor_a.Kd = pid_kd_a;

    pid_motor_b.Kp = pid_kp_b;
    pid_motor_b.Ki = pid_ki_b;
    pid_motor_b.Kd = pid_kd_b;

    /* 计算电机A的PID控制值并应用 */
    if (fabsf(pid_motor_a.setpoint) > 0.1f)
    {
        PID_Compute(&pid_motor_a);

        /* 根据PID输出的符号确定方向和PWM值 */
        if (pid_motor_a.output >= 0)
        {
            pwm_value = (uint16_t)pid_motor_a.output;
        }
        else
        {
            pwm_value = (uint16_t)(-pid_motor_a.output); // 取绝对值作为PWM
        }

        /* 限制PWM值范围 */
        if (pwm_value > 999)
            pwm_value = 999;

        Motor_SetSpeed(1, pwm_value, pid_motor_a.direction);
    }
    else
    {
        /* 目标速度为0时停止电机 */
        Motor_Stop(1);
        pid_motor_a.direction = MOTOR_STOP; // 更新PID结构体中的方向信息
    }

    /* 计算电机B的PID控制值并应用 */
    if (fabsf(pid_motor_b.setpoint) > 0.1f)
    {
        PID_Compute(&pid_motor_b);

        /* 根据PID输出的符号确定方向和PWM值 */
        if (pid_motor_b.output >= 0)
        {
            pwm_value = (uint16_t)pid_motor_b.output;
        }
        else
        {
            pwm_value = (uint16_t)(-pid_motor_b.output); // 取绝对值作为PWM
        }

        /* 限制PWM值范围 */
        if (pwm_value > 999)
            pwm_value = 999;

        Motor_SetSpeed(2, pwm_value, pid_motor_b.direction);
    }
    else
    {
        /* 目标速度为0时停止电机 */
        Motor_Stop(2);
    }
}

/**
 * @brief 角度差值计算（处理角度环绕问题）
 * @param target: 目标角度
 * @param current: 当前角度
 * @retval 角度差值
 */
static float Angle_Difference(float target, float current)
{
    float diff = target - current;

    /* 处理角度环绕问题，确保差值在-180到180度之间 */
    if (diff > 180.0f)
        diff -= 360.0f;
    else if (diff < -180.0f)
        diff += 360.0f;

    return diff;
}

/**
 * @brief 角度PID控制器计算函数
 * @retval PID输出值
 */
float AnglePID_Compute(void)
{
    /* 计算当前角度误差（处理角度环绕问题） */
    angle_pid_yaw.error = Angle_Difference(angle_pid_yaw.setpoint, angle_pid_yaw.feedback);

    /* 计算积分项 */
    angle_pid_yaw.integral += angle_pid_yaw.error;

    /* 积分项限幅，防止积分饱和 */
    float integral_limit = angle_pid_yaw.output_max / angle_pid_yaw.Ki;
    if (angle_pid_yaw.integral > integral_limit)
        angle_pid_yaw.integral = integral_limit;
    else if (angle_pid_yaw.integral < -integral_limit)
        angle_pid_yaw.integral = -integral_limit;

    /* 计算微分项 */
    angle_pid_yaw.derivative = angle_pid_yaw.error - angle_pid_yaw.last_error;

    /* 计算PID输出 */
    angle_pid_yaw.output = angle_pid_yaw.Kp * angle_pid_yaw.error +
                           angle_pid_yaw.Ki * angle_pid_yaw.integral +
                           angle_pid_yaw.Kd * angle_pid_yaw.derivative;

    /* 输出限幅 */
    if (angle_pid_yaw.output > angle_pid_yaw.output_max)
        angle_pid_yaw.output = angle_pid_yaw.output_max;
    else if (angle_pid_yaw.output < angle_pid_yaw.output_min)
        angle_pid_yaw.output = angle_pid_yaw.output_min;

    /* 更新误差记录 */
    angle_pid_yaw.last_error = angle_pid_yaw.error;

    return angle_pid_yaw.output;
}

/**
 * @brief 初始化角度环PID控制器
 * @retval None
 */
void AnglePID_Init(void)
{
    /* 初始化PID参数 */
    angle_pid_yaw.Kp = ANGLE_PID_KP;
    angle_pid_yaw.Ki = ANGLE_PID_KI;
    angle_pid_yaw.Kd = ANGLE_PID_KD;

    /* 重置PID状态 */
    angle_pid_yaw.setpoint = 0.0f;
    angle_pid_yaw.feedback = 0.0f;
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
    angle_pid_yaw.final_output = 0.0f;

    /* 设置输出限制 */
    angle_pid_yaw.output_min = ANGLE_PID_OUTPUT_MIN;
    angle_pid_yaw.output_max = ANGLE_PID_OUTPUT_MAX;

    angle_pid_yaw.angle_mode = PID_MODE_MANUAL;
    angle_pid_yaw.base_speed = 0.0f; // 初始基础速度为0
}

/**
 * @brief 初始化角度环PID控制器
 * @retval None
 */
void Angle_Self_PID_Set(void)
{
    /* 初始化PID参数 */
    angle_pid_yaw.Kp = ANGLE_SELF_KP;
    angle_pid_yaw.Ki = ANGLE_SELF_KI;
    angle_pid_yaw.Kd = ANGLE_SELF_KD;

    /* 重置PID状态 */
    angle_pid_yaw.setpoint = 0.0f;
    angle_pid_yaw.feedback = 0.0f;
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
    angle_pid_yaw.final_output = 0.0f;

    /* 设置输出限制 */
    angle_pid_yaw.output_min = ANGLE_SELF_OUTPUT_MIN;
    angle_pid_yaw.output_max = ANGLE_SELF_OUTPUT_MAX;

    angle_pid_yaw.angle_mode = PID_MODE_STRAIGHT_DRIVE;
    angle_pid_yaw.base_speed = 0.0f; // 初始基础速度为0
}

/**
 * @brief 设置角度PID目标值
 * @param target_angle: 目标角度(度)
 * @retval None
 */
void AnglePID_SetTarget(float target_angle)
{
    angle_pid_yaw.setpoint = target_angle;
}

/**
 * @brief 重置角度PID状态
 * @retval None
 */
void AnglePID_Reset(void)
{
    angle_pid_yaw.error = 0.0f;
    angle_pid_yaw.last_error = 0.0f;
    angle_pid_yaw.integral = 0.0f;
    angle_pid_yaw.derivative = 0.0f;
    angle_pid_yaw.output = 0.0f;
}

/**
 * @brief 更新角度环PID控制器
 * @note 此函数应在定时器中定期调用，建议调用频率与陀螺仪读取频率一致
 * @retval None
 */
void AnglePID_Update()
{
    float yaw_output;
    float motor_a_speed, motor_b_speed;

    /* 更新反馈值 */
    angle_pid_yaw.feedback = My_YawAngle;

    float angle_error = Angle_Difference(angle_pid_yaw.setpoint, angle_pid_yaw.feedback);

    if (fabsf(angle_error) <= DEAD_ZONE)
    {
        /* 在死区内：强制置零积分项和偏差项，不计算PID */
        angle_pid_yaw.integral = 0.0f;
        angle_pid_yaw.error = 0.0f;
        angle_pid_yaw.last_error = 0.0f;
        angle_pid_yaw.derivative = 0.0f;
        yaw_output = 0.0f;
    }
    else
    {
        yaw_output = AnglePID_Compute();
        /* 直行模式：基础速度 + 偏航修正 */
        motor_a_speed = angle_pid_yaw.base_speed + yaw_output;
        motor_b_speed = angle_pid_yaw.base_speed - yaw_output;
        /* 设置电机目标速度，让底层PID根据正负自动确定方向 */
        angle_pid_yaw.final_output = motor_a_speed; // 保存最终输出值
        PID_SetSpeed(PID_MOTOR_A, motor_a_speed);
        PID_SetSpeed(PID_MOTOR_B, motor_b_speed);
    }
}

/**
 * @brief 停止所有电机
 * @retval None
 */
void PID_StopAll(void)
{
    /* 停止电机 */
    PID_SetSpeed(PID_MOTOR_A, 0.0f);
    PID_SetSpeed(PID_MOTOR_B, 0.0f);

    /* 重置控制状态 */
    angle_pid_yaw.angle_mode = PID_MODE_MANUAL;
    angle_pid_yaw.base_speed = 0.0f;

    /* 禁用并重置角度PID */
    AnglePID_Reset();
}