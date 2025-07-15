#ifndef _MYIIC_H
#define _MYIIC_H

#include <stdint.h>

#define SDA_IN()  {GPIOA->CRL &= 0xFFFFFF0F; GPIOA->CRL |= (0x08<<4);}  
// 配置PA1为输出
#define SDA_OUT() {GPIOA->CRL &= 0xFFFFFF0F; GPIOA->CRL |= (0x03<<4);}

// 使用HAL库函数操作GPIO
#define IIC_SCL(x)    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_SDA(x)    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define READ_SDA      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�



uint8_t IIC_Write_1Byte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
uint8_t IIC_Read_1Byte(uint8_t SlaveAddress, uint8_t REG_Address);
uint8_t IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, const uint8_t *buf);
uint8_t IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf);

#endif

