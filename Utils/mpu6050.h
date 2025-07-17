/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */
#include "i2c.h"

#ifndef INC_GY521_H_
#define INC_GY521_H_



#include <stdint.h>




// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    float YawAngle;      // Z轴累计角度(偏航角)
    float YawDiff;       // 偏航角差值
    float PrevYawAngle;  // 上一次的偏航角

    // 添加位移相关数据
    float DisplacementX; // X轴位移
    float DisplacementY; // Y轴位移

    float Yaw_Custom; // 自定义偏航角

} MPU6050_t;


// 陀螺仪角度累计计算结构体
typedef struct {
    float currentAngle;        // 当前累积角度
    float previousAngle;       // 上一次的累积角度
    float angleDiff;           // 两次角度之间的差值
    float gyroZFiltered;       // 滤波后的z轴角速度
    float noiseThreshold;      // 噪声阈值
    uint32_t lastTime;         // 上次更新时间
    uint8_t isInitialized;     // 是否初始化
} GyroIntegrator_t;


// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

typedef struct {
    float accXFiltered;        // X轴滤波后的加速度
    float accYFiltered;        // Y轴滤波后的加速度
    float displacementX;       // X轴位移
    // float displacementY;       // Y轴位移
    float velocityX;           // X轴速度
    // float velocityY;           // Y轴速度
    float prevAccX;            // 上一次X轴加速度
    // float prevAccY;            // 上一次Y轴加速度
    uint32_t lastTime;         // 上次更新时间
    uint8_t isInitialized;     // 是否初始化
    float noiseThreshold;      // 噪声阈值
} DisplacementCalculator_t;

// 在全局变量区域添加以下变量
typedef struct {
    float accXSum;               // 累积的X轴加速度值
    float accYSum;               // 累积的Y轴加速度值
    uint32_t sampleCount;        // 采样计数
    float accXBias;              // 计算得到的X轴偏差
    float accYBias;              // 计算得到的Y轴偏差
    uint8_t calibrationDone;     // 标记是否完成校准
    float g_real;                // 实际重力加速度
} AccelCalibration_t;

extern DisplacementCalculator_t displacementCalculator;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void GyroIntegrator_Init(GyroIntegrator_t *integrator, float noiseThreshold);
void GyroIntegrator_Update(GyroIntegrator_t *integrator, float gyroZ, uint8_t resetFlag);
void MPU6050_Reset_YawAngle(MPU6050_t *DataStruct);


void DisplacementCalculator_Init(DisplacementCalculator_t *calculator, float noiseThreshold);
// 更新DisplacementCalculator_Update函数声明
void DisplacementCalculator_Update(DisplacementCalculator_t *calculator, float accX, float accY, float accZ, float roll, float pitch, float yaw);
void DisplacementCalculator_Reset(DisplacementCalculator_t *calculator);

#endif /* INC_GY521_H_ */