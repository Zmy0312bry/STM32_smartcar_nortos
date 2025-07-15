#include "icm42688.h"
#include "usart.h"
#include <stdio.h>
#if defined(ICM_USE_HARD_SPI)
#include "spi.h"
#elif defined(ICM_USE_I2C)
#include "myiic.h"
#endif

static float accSensitivity   = 0.244f;   //���ٶȵ���С�ֱ��� mg/LSB
static float gyroSensitivity  = 32.8f;    //�����ǵ���С�ֱ���


/*ICM42688ʹ�õ�ms����ʱ����*/
#define ICM42688DelayMs(_nms)  HAL_Delay(_nms)



/*******************************************************************************
* ��    �ƣ� icm42688_read_reg
* ��    �ܣ� ��ȡ�����Ĵ�����ֵ
* ��ڲ����� reg: �Ĵ�����ַ
* ���ڲ����� ��ǰ�Ĵ�����ַ��ֵ
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page51.
*******************************************************************************/
static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;

#if defined(ICM_USE_HARD_SPI)
    SPI_IMU_CS=0;
    reg |= 0x80;
    /* д��Ҫ���ļĴ�����ַ */
    SPI1_ReadWriteByte(reg);
    /* ��ȡ�Ĵ������� */
    regval = SPI1_ReadWriteByte(0x00);
    SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IIC_Read_nByte(ICM42688_ADDRESS, reg, 1, &regval);
#endif

    return regval;
}

/*******************************************************************************
* ��    �ƣ� icm42688_read_regs
* ��    �ܣ� ������ȡ����Ĵ�����ֵ
* ��ڲ����� reg: ��ʼ�Ĵ�����ַ *buf����ָ��,uint16_t len����
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page50.
*******************************************************************************/
static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    SPI_IMU_CS=0;
	//delay_us(100);
    /* д��Ҫ���ļĴ�����ַ */
    SPI1_ReadWriteByte(reg);
    /* ��ȡ�Ĵ������� */
    while(len)
	{
		*buf = SPI1_ReadWriteByte(0x00);
		len--;
		buf++;
	}
	//delay_us(100);
    SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IIC_Read_nByte(ICM42688_ADDRESS, reg, len, buf);
#endif
}


/*******************************************************************************
* ��    �ƣ� icm42688_write_reg
* ��    �ܣ� �򵥸��Ĵ���д����
* ��ڲ����� reg: �Ĵ�����ַ value:����
* ���ڲ����� 0
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page50.
*******************************************************************************/
static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
#if defined(ICM_USE_HARD_SPI)
    SPI_IMU_CS=0;
    /* д��Ҫ���ļĴ�����ַ */
    /* д��Ҫ���ļĴ�����ַ */
    SPI1_ReadWriteByte(reg);
    /* ��ȡ�Ĵ������� */
    SPI1_ReadWriteByte(value);
    SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IIC_Write_nByte(ICM42688_ADDRESS, reg, 1, &value);
#endif
    return 0;
}



float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

/*******************************************************************************
* ��    �ƣ� bsp_Icm42688RegCfg
* ��    �ܣ� Icm42688 �Ĵ�������
* ��ڲ����� ��
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע��
*******************************************************************************/
int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val = 0;
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //����bank 0����Ĵ���
    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01); //����λ������
    ICM42688DelayMs(100);
    /* ��ȡ who am i �Ĵ��� */
    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
	printf("reg_val:%d\n",reg_val);
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //����bank 0����Ĵ���
    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01); //����λ������
    ICM42688DelayMs(100);


    if(reg_val == ICM42688_ID)
    {
        bsp_Icm42688GetAres(AFS_4G);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = (AFS_4G << 5);   //���� ��4g
        reg_val |= (AODR_100Hz);     //������� 100HZ
        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

        bsp_Icm42688GetGres(GFS_1000DPS);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = (GFS_1000DPS << 5);   //���� ��1000dps
        reg_val |= (GODR_100Hz);     //������� 100HZ
        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); //��ȡPWR��MGMT0��ǰ�Ĵ�����ֵ(page72)
        reg_val &= ~(1 << 5);//ʹ���¶Ȳ���
        reg_val |= ((3) << 2);//����GYRO_MODE  0:�ر� 1:���� 2:Ԥ�� 3:������
        reg_val |= (3);//����ACCEL_MODE 0:�ر� 1:�ر� 2:�͹��� 3:������
        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
        ICM42688DelayMs(1); //������PWR��MGMT0�Ĵ����� 200us�ڲ������κζ�д�Ĵ����Ĳ���

        return 0;
    }
    return -1;
}
													 
//��ʼ��SPI ICM��IO��
void ICM426XX_Init(void)
{ 
#if defined(ICM_USE_HARD_SPI)
//    u8 temp;
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();           //ʹ��GPIOBʱ��
    
    //PA4
    GPIO_Initure.Pin=GPIO_PIN_4;            //PA4
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����         
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //��ʼ��
    
	SPI_IMU_CS=1;			                //SPI FLASH��ѡ��
	SPI1_Init();		   			        //��ʼ��SPI
	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_256); 

#elif defined(ICM_USE_I2C)
	IIC_Init();
#endif
}  

/*******************************************************************************
* ��    �ƣ� bsp_Icm42688Init
* ��    �ܣ� Icm42688 ��������ʼ��
* ��ڲ����� ��
* ���ڲ����� 0: ��ʼ���ɹ�  ����ֵ: ��ʼ��ʧ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע��
*******************************************************************************/
int8_t bsp_Icm42688Init(void)
{
	ICM426XX_Init();
    return(bsp_Icm42688RegCfg());

}

/*******************************************************************************
* ��    �ƣ� bsp_IcmGetTemperature
* ��    �ܣ� ��ȡIcm42688 �ڲ��������¶�
* ��ڲ����� ��
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

/*******************************************************************************
* ��    �ƣ� bsp_IcmGetAccelerometer
* ��    �ܣ� ��ȡIcm42688 ���ٶȵ�ֵ
* ��ڲ����� ������ٶȵ�ֵ
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    accData->x = (int16_t)(accData->x * accSensitivity);
//    accData->y = (int16_t)(accData->y * accSensitivity);
//    accData->z = (int16_t)(accData->z * accSensitivity);

    return 0;
}

/*******************************************************************************
* ��    �ƣ� bsp_IcmGetGyroscope
* ��    �ܣ� ��ȡIcm42688 �����ǵ�ֵ
* ��ڲ����� ���������ǵ�ֵ
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� datasheet page63
*******************************************************************************/
int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
    return 0;
}

/*******************************************************************************
* ��    �ƣ� bsp_IcmGetRawData
* ��    �ܣ� ��ȡIcm42688���ٶ�����������
* ��ڲ����� ����
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2024-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� datasheet page62,63
*******************************************************************************/
int8_t bsp_IcmGetRawData(icm42688RealData_t* accData, icm42688RealData_t* GyroData)
{
    uint8_t buffer[12] = {0};
	icm42688RawData_t accRaw;
	icm42688RawData_t gyroRaw;

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    accRaw.x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
    accRaw.y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
    accRaw.z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
    gyroRaw.x = ((uint16_t)buffer[6] << 8)  | buffer[7];
    gyroRaw.y = ((uint16_t)buffer[8] << 8)  | buffer[9];
    gyroRaw.z = ((uint16_t)buffer[10] << 8) | buffer[11];
    
    if ((accRaw.x == -1 && accRaw.y == -1 && accRaw.z == -1) || accRaw.x == -32768)
    {
        return 1;
    }

    accData->x = (float)(accRaw.x * accSensitivity);
    accData->y = (float)(accRaw.y * accSensitivity);
    accData->z = (float)(accRaw.z * accSensitivity);

    GyroData->x = (float)(gyroRaw.x * gyroSensitivity);
    GyroData->y = (float)(gyroRaw.y * gyroSensitivity);
    GyroData->z = (float)(gyroRaw.z * gyroSensitivity);

    return 0;
}

