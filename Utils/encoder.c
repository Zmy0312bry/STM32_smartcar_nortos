#include "encoder.h"
#include "tim.h"

/* 编码器实例 */
Encoder_TypeDef encoderA; // 对应TIM3
Encoder_TypeDef encoderB; // 对应TIM1

/**
 * @brief  编码器初始化
 * @param  None
 * @retval None
 */
void Encoder_Init(void)
{
    // 清除计数器值
    __HAL_TIM_SET_COUNTER(&htim3, 0); // ENCODER_A
    __HAL_TIM_SET_COUNTER(&htim1, 0); // ENCODER_B

    // 初始化编码器A结构体
    encoderA.count = 0;
    encoderA.last_count = 0;
    encoderA.diff = 0;
    encoderA.speed_rpm = 0.0f;
    encoderA.speed_rps = 0.0f;
    encoderA.direction = 0;
    encoderA.id = ENCODER_A;
    encoderA.htim = &htim3;
    encoderA.resolution = ENCODER_RESOLUTION;
    encoderA.gear_ratio = GEAR_RATIO;

    // 初始化编码器B结构体
    encoderB.count = 0;
    encoderB.last_count = 0;
    encoderB.diff = 0;
    encoderB.speed_rpm = 0.0f;
    encoderB.speed_rps = 0.0f;
    encoderB.direction = 0;
    encoderB.id = ENCODER_B;
    encoderB.htim = &htim1;
    encoderB.resolution = ENCODER_RESOLUTION;
    encoderB.gear_ratio = GEAR_RATIO;
}

/**
 * @brief  清除编码器计数值
 * @param  encoder: 编码器结构体指针
 * @retval None
 */
void Encoder_Clear(Encoder_TypeDef *encoder)
{
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->count = 0;
    encoder->last_count = 0;
    encoder->diff = 0;
}

/**
 * @brief  更新编码器状态
 * @param  encoder: 编码器结构体指针
 * @param  sample_time_ms: 采样时间(毫秒)
 * @retval None
 */
void Encoder_Update(Encoder_TypeDef *encoder, uint32_t sample_time_ms)
{
    // 保存上次计数值
    encoder->last_count = encoder->count;

    // 读取当前计数值（根据结构体中的定时器句柄）
    encoder->count = (int16_t)(__HAL_TIM_GET_COUNTER(encoder->htim));

    // 计算差值（考虑计数器溢出情况）
    if (encoder->count < encoder->last_count)
    {
        // 如果当前值小于上次值，说明可能发生了向上溢出
        if (encoder->last_count - encoder->count > ENCODER_TIM_PERIOD / 2)
        {
            encoder->diff = encoder->count + (ENCODER_TIM_PERIOD - encoder->last_count);
        }
        else
        {
            encoder->diff = encoder->count - encoder->last_count;
        }
    }
    else
    {
        // 如果当前值大于上次值，说明可能发生了向下溢出
        if (encoder->count - encoder->last_count > ENCODER_TIM_PERIOD / 2)
        {
            encoder->diff = encoder->count - (ENCODER_TIM_PERIOD + encoder->last_count);
        }
        else
        {
            encoder->diff = encoder->count - encoder->last_count;
        }
    }

    // 判断方向并保持原始符号
    if (encoder->diff > 0)
    {
        encoder->direction = 0; // 正转
    }
    else if (encoder->diff < 0)
    {
        encoder->direction = 1; // 反转
        // 注意：这里不再取绝对值，保持负值
    }
    else
    {
        // diff == 0，保持上次方向
    }

    // 计算转速 (counts/sample_time => RPM)
    // 使用带符号的diff值，这样反转时RPM为负值
    encoder->speed_rpm = (float)encoder->diff * 60.0f * 1000.0f /
                         (encoder->resolution * encoder->gear_ratio * sample_time_ms);

    // 计算RPS (RPM / 60)，同样保持符号
    encoder->speed_rps = encoder->speed_rpm / 60.0f;
}