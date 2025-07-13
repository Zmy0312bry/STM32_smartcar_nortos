#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "tim.h"

/* 编码器相关宏定义 */
#define ENCODER_TIM_PERIOD 65535 // 编码器定时器计数周期
#define ENCODER_RESOLUTION 11    // 编码器分辨率(线数)
#define GEAR_RATIO 20.41f            // 电机减速比


    /* 编码器枚举 */
    typedef enum
    {
        ENCODER_A = 0, // 对应TIM3
        ENCODER_B = 1  // 对应TIM1
    } Encoder_ID;

    /* 编码器结构体定义 */
    typedef struct
    {
        int16_t count;           // 编码器计数值
        int16_t last_count;      // 上一次编码器计数值
        int16_t diff;            // 两次计数差值
        float speed_rpm;         // 电机转速(RPM)
        float speed_rps;         // 电机转速(RPS)
        uint8_t direction;       // 旋转方向: 0-正转, 1-反转
        Encoder_ID id;           // 编码器ID: 0-ENCODER_A, 1-ENCODER_B
        TIM_HandleTypeDef *htim; // 定时器句柄
        uint8_t resolution;      // 编码器分辨率
        float gear_ratio;      // 减速比
    } Encoder_TypeDef;

    /* 函数声明 */
    void Encoder_Init(void);
    void Encoder_Update(Encoder_TypeDef *encoder, uint32_t sample_time_ms);
    void Encoder_Clear(Encoder_TypeDef *encoder);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */