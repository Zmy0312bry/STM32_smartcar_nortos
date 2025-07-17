/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#ifndef M_PI
#define M_PI 3.1415926536
#endif

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

// 在现有全局变量下方添加
GyroIntegrator_t gyroIntegrator = {0};
DisplacementCalculator_t displacementCalculator = {0};
AccelCalibration_t accelCalibration = {0};

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

    // 判断是否初始化了陀螺仪积分器
    if (!gyroIntegrator.isInitialized)
    {
        GyroIntegrator_Init(&gyroIntegrator, 0.5f); // 设置0.5°/s的噪声阈值
    }

    // 更新Z轴角度(偏航角)
    GyroIntegrator_Update(&gyroIntegrator, DataStruct->Gz, 0); // 0表示不重置

    // 更新结构体中的偏航角数据
    DataStruct->PrevYawAngle = DataStruct->YawAngle;
    DataStruct->YawAngle = gyroIntegrator.currentAngle;
    DataStruct->YawDiff = gyroIntegrator.angleDiff;

    if (!displacementCalculator.isInitialized)
    {
        DisplacementCalculator_Init(&displacementCalculator, 0.04f); // 设置0.05g的噪声阈值
    }

    // 更新位移计算
    // 注意：需要考虑姿态角对加速度的影响，可能需要将身体坐标系转换到地面坐标系
    DisplacementCalculator_Update(&displacementCalculator,
                                  DataStruct->Ax,
                                  DataStruct->Ay,
                                  DataStruct->Az,
                                  DataStruct->KalmanAngleX, // 翻滚角roll
                                  DataStruct->KalmanAngleY, // 俯仰角pitch
                                  DataStruct->YawAngle);    // 偏航角yaw

    // 更新结构体中的位移数据（如果需要）
    DataStruct->DisplacementX = displacementCalculator.displacementX;
    
    if (DataStruct->Gz > 0.8f || DataStruct->Gz < -0.8f)
    {
        DataStruct->Yaw_Custom += DataStruct->Gz * dt;
    }
    // DataStruct->DisplacementY = displacementCalculator.displacementY;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

// 在现有函数之后添加
void GyroIntegrator_Init(GyroIntegrator_t *integrator, float noiseThreshold)
{
    integrator->currentAngle = 0.0f;
    integrator->previousAngle = 0.0f;
    integrator->angleDiff = 0.0f;
    integrator->gyroZFiltered = 0.0f;
    integrator->noiseThreshold = noiseThreshold;
    integrator->lastTime = HAL_GetTick();
    integrator->isInitialized = 1;
}

// 在GyroIntegrator_Init函数后添加
void GyroIntegrator_Update(GyroIntegrator_t *integrator, float gyroZ, uint8_t resetFlag)
{
    uint32_t currentTime;
    float dt;
    float alpha = 0.8f; // 低通滤波器系数

    // 检查是否需要重置
    if (resetFlag)
    {
        integrator->currentAngle = 0.0f;
        integrator->previousAngle = 0.0f;
        integrator->angleDiff = 0.0f;
        integrator->gyroZFiltered = 0.0f;
        integrator->lastTime = HAL_GetTick();
        return;
    }

    // 计算时间间隔
    currentTime = HAL_GetTick();
    dt = (float)(currentTime - integrator->lastTime) / 1000.0f; // 转换为秒
    integrator->lastTime = currentTime;

    // 跳过过短或异常的时间间隔
    if (dt <= 0.0f || dt > 0.5f)
    {
        return;
    }

    // 低通滤波以去除噪声
    integrator->gyroZFiltered = alpha * integrator->gyroZFiltered + (1.0f - alpha) * gyroZ;

    // 应用噪声阈值
    if (fabs(integrator->gyroZFiltered) < integrator->noiseThreshold)
    {
        integrator->gyroZFiltered = 0.0f;
    }

    // 保存上一次角度
    integrator->previousAngle = integrator->currentAngle;

    // 积分计算角度变化
    integrator->currentAngle += integrator->gyroZFiltered * dt;

    // 确保角度在0-360度范围内
    while (integrator->currentAngle >= 360.0f)
    {
        integrator->currentAngle -= 360.0f;
    }
    while (integrator->currentAngle < 0.0f)
    {
        integrator->currentAngle += 360.0f;
    }

    // 计算角度差值
    integrator->angleDiff = integrator->currentAngle - integrator->previousAngle;

    // 处理角度跨越0/360度边界的情况
    if (integrator->angleDiff > 180.0f)
    {
        integrator->angleDiff -= 360.0f;
    }
    else if (integrator->angleDiff < -180.0f)
    {
        integrator->angleDiff += 360.0f;
    }
}

// 在GyroIntegrator_Update函数后添加
void MPU6050_Reset_YawAngle(MPU6050_t *DataStruct)
{
    // 重置积分器
    GyroIntegrator_Update(&gyroIntegrator, 0.0f, 1); // 1表示重置

    // 重置结构体中的偏航角数据
    DataStruct->YawAngle = 0.0f;
    DataStruct->PrevYawAngle = 0.0f;
    DataStruct->YawDiff = 0.0f;
}

void DisplacementCalculator_Init(DisplacementCalculator_t *calculator, float noiseThreshold)
{
    calculator->accXFiltered = 0.0f;
    calculator->accYFiltered = 0.0f;
    calculator->displacementX = 0.0f;
    // calculator->displacementY = 0.0f;
    calculator->velocityX = 0.0f;
    // calculator->velocityY = 0.0f;
    calculator->prevAccX = 0.0f;
    // calculator->prevAccY = 0.0f;
    calculator->lastTime = HAL_GetTick();
    calculator->isInitialized = 1;
    calculator->noiseThreshold = noiseThreshold;

    // 初始化校准数据
    accelCalibration.accXSum = 0.0f;
    accelCalibration.accYSum = 0.0f;
    accelCalibration.sampleCount = 0;
    accelCalibration.accXBias = 0.0f;
    accelCalibration.accYBias = 0.0f;
    accelCalibration.calibrationDone = 0;
}
#define STEP 50
void DisplacementCalculator_Update(DisplacementCalculator_t *calculator, float accX, float accY, float accZ, float roll, float pitch, float yaw)
{
    uint32_t currentTime;
    float dt;
    float accX_real;

    // 计算时间间隔
    currentTime = HAL_GetTick();
    dt = (float)(currentTime - calculator->lastTime) / 1000.0f; // 转换为秒
    calculator->lastTime = currentTime;

    // 跳过过短或异常的时间间隔
    if (dt <= 0.0f || dt > 0.5f)
    {
        return;
    }
    // Initialization g_calculated and Bias
    if (accelCalibration.calibrationDone == 0)
    {
        // 计算重力加速度
        if (accelCalibration.sampleCount == 600)
        {
            accelCalibration.g_real = sqrt(accX * accX + accY * accY + accZ * accZ);
        }

        // add ticks
        accelCalibration.sampleCount++;
    }

    // 1. 将roll(翻滚角)、pitch(俯仰角)和yaw(偏航角)转换为弧度
    float roll_rad = roll * M_PI / 180.0f;
    float pitch_rad = pitch * M_PI / 180.0f;
    float yaw_rad = yaw * M_PI / 180.0f;

    // 2. 将传感器坐标系下的加速度转换到重力坐标系下，但保持x轴与传感器前方一致
    // 首先，补偿俯仰和翻滚对加速度的影响
    accX_real = accX - accelCalibration.g_real * sin(pitch_rad);
    // float accY_noRollPitch = accX * sin(roll_rad) * sin(pitch_rad) + accY * cos(roll_rad) - accZ * sin(roll_rad) * cos(pitch_rad);

    // 低通滤波去除高频噪声
    // calculator->accXFiltered = alpha * calculator->accXFiltered + (1.0f - alpha) * accX_world;
    // calculator->accYFiltered = alpha * calculator->accYFiltered + (1.0f - alpha) * accY_world;
    if (accelCalibration.sampleCount <= 1200 && accelCalibration.sampleCount >= 800)
    {
        const float n = 400 / STEP + 1.0f; // 计算滤波系数
        // 校准阶段：累加过滤后的加速度值
        if (accelCalibration.sampleCount % STEP == 0)
        {
            accelCalibration.accXSum += accX_real;
            // accelCalibration.accYSum += accY_world;
        }

        // 当达到2000次采样时，计算平均偏差
        if (accelCalibration.sampleCount == 1200)
        {
            accelCalibration.accXBias = accelCalibration.accXSum / n;
            // accelCalibration.accYBias = accelCalibration.accYSum / n;
            accelCalibration.calibrationDone = 1;
            accelCalibration.sampleCount++;

            // 重置速度和位移，确保从零开始正确计算
            calculator->velocityX = 0.0f;
            // calculator->velocityY = 0.0f;
            calculator->displacementX = 0.0f;
            // calculator->displacementY = 0.0f;
        }
    }

    // 校准完成后，从过滤后的加速度中减去偏差
    float compensatedAccX = accX_real - accelCalibration.accXBias;
    // float compensatedAccY = accY_real - accelCalibration.accYBias;
    // 添加低通滤波器来去除高频噪声分量
    // alpha值越大，滤波效果越强，但响应越慢；alpha值越小，响应越快，但滤波效果越弱
    float aalpha = 0.50f; // 可以根据实际情况调整，范围0-1
    static float filteredAccX = 0.0f;

    // 应用低通滤波器
    filteredAccX = aalpha * filteredAccX + (1.0f - aalpha) * compensatedAccX;
    // filteredAccY = aalpha * filteredAccY + (1.0f - aalpha) * compensatedAccY;

    // 使用滤波后的加速度代替原始补偿加速度
    compensatedAccX = filteredAccX;
    // compensatedAccY = filteredAccY;
    // printf("%.2f\r\n",
    //        compensatedAccX);

    if (fabs(compensatedAccX) < calculator->noiseThreshold)
    {
        compensatedAccX = 0.0f;
    }
    // if (fabs(compensatedAccY) < calculator->noiseThreshold)
    // {
    //     compensatedAccY = 0.0f;
    // }

    // 加速度积分得到速度 (梯形法则)
    calculator->velocityX += (compensatedAccX + calculator->prevAccX) * dt / 2.0f;
    // calculator->velocityY += (compensatedAccY + calculator->prevAccY) * dt / 2.0f;

    // 速度积分得到位移
    calculator->displacementX += calculator->velocityX * dt;
    // calculator->displacementY += calculator->velocityY * dt;

    // 保存当前补偿后加速度用于下次计算
    calculator->prevAccX = compensatedAccX;
    // calculator->prevAccY = compensatedAccY;
    // printf("%.2f,%.2f\r\n", calculator->velocityX,calculator->displacementX);
    // 轻微的速度衰减，防止小误差累积导致速度持续增长
    calculator->velocityX *= 0.995f;
    // calculator->velocityY *= 0.995f;
}

void DisplacementCalculator_Reset(DisplacementCalculator_t *calculator)
{
    calculator->accXFiltered = 0.0f;
    calculator->accYFiltered = 0.0f;
    calculator->displacementX = 0.0f;
    // calculator->displacementY = 0.0f;
    calculator->velocityX = 0.0f;
    // calculator->velocityY = 0.0f;
    calculator->prevAccX = 0.0f;
    // calculator->prevAccY = 0.0f;
    calculator->lastTime = HAL_GetTick();
}