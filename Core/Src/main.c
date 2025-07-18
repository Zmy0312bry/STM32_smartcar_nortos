/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "flow_decode.h"
#include "motor.h"
#include "pid_control.h"
#include "angle_control.h"
#include <stdio.h>
#include <string.h>
#include "jy61p.h"
#include "icm42688.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050 = {0};
/* The variables about GRYO is declared here */

volatile uint8_t gyro_sample = 0; // How many seconds to read data
uint8_t g_usart2_receivedata;

/* This is the end of declaration */

/* This is the declaration of the variables of optical flow sensor */
volatile uint8_t optical_flow_sample = 0; // How many seconds to read data
/* This is the end of declaration */

/* This is the declaration of the variables of encoder data*/
extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;
volatile uint8_t encoder_sample = 0; // How many seconds to read encoder data
/* This is the end of declaration to the encoder*/

/* This is the declaration of the variables of angle PID control*/
volatile uint8_t angle_pid_sample = 0; // How many ms to update angle PID
//icm42688RealData_t accval;
//icm42688RealData_t gyroval;
/* This is the end of declaration to the angle PID*/

/* This is the sapce for you to declare temp variables */
volatile uint8_t pid_temp = 0;
/* This is the end of temp space */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* This is the space for some function for gyro */
/* This is the end */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Init uart receive
  UART_StartReceive(&huart3);
  HAL_UART_Receive_IT(&huart2, &g_usart2_receivedata, 1);
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  Motor_Init();
  Encoder_Init();

  /* Init the PID control here */
  PID_Init(&pid_motor_a);
  PID_Init(&pid_motor_b);
  /* Finish init PID control */

  /* This is the init for icm42688 */
  // HAL_Delay(500); // Wait for the sensor to stabilize
  // while (MPU6050_Init(&hi2c1) == 1);
  /* This is the end */

  /* Init the Yaw Angle PID control here */
  //AngleControl_Init();
  // SelfTurnTarget(90.0f); // Set initial target angle for Yaw
  PID_SetSpeed(PID_MOTOR_A, 60.0f); // Initialize motor A speed to 0
  PID_SetSpeed(PID_MOTOR_B, 60.0f); // Initialize motor
  /* Finish init Yaw Angle PID control */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* This is the space for you to declare temp variables */
    /* This is the end */

    /* This is the call to the gyroscope code function */
    if (gyro_sample >= 69) // This means that the gyroscope data is read every 100ms
    {
      //MPU6050_Read_All(&hi2c1, &MPU6050);
      gyro_sample = 0; // Reset the sample counter
    }

    /* This is the end of this function */

    /* This is the call to the function of read the data of optical flow sensor */
    if (optical_flow_sample >= 99)
    {
      static int ret;
      static unsigned char ch;
      HAL_UART_Receive(&huart2, &ch, 1, 25);
      ret = upnotof_parse_char(ch);
      if (!ret)
      {
        static int16_t flow_x_integral = 0;
        static int16_t flow_y_integral = 0;
        static uint8_t valid = 0;
        flow_x_integral = up_flow_data.flow_x_integral;
        flow_y_integral = up_flow_data.flow_y_integral;
        valid = up_flow_data.valid;
        printf("flow_x_integral=%d,flow_y_integral=%d,valid=%d\n",
               flow_x_integral, flow_y_integral, valid);
      }
      optical_flow_sample = 0; // Reset the sample counter
    }
    /* This is the end of this function */

    /* This is the function to print encoder data */

    if (encoder_sample >= 69)
    {

      //AnglePID_Update();
      Encoder_Update(&encoderA, 70); // Put into the sample time(ms)
      Encoder_Update(&encoderB, 70);
      PID_Update();
      printf("EncoderA:%.2f,", encoderA.speed_rpp);
      printf("EncoderB:%.2f,", encoderB.speed_rpp);
      printf("EncoderA_rpm:%.2f,", encoderA.speed_rpm);
      printf("EncoderB_rpm:%.2f,", encoderB.speed_rpm);
      printf("targetA:%.2f,", pid_motor_a.setpoint);
      printf("targetB:%.2f,", pid_motor_b.setpoint);
      printf("PID_A:%.2f,", pid_motor_a.output);
      printf("PID_B:%.2f\r\n", pid_motor_b.output);
      //printf("Yaw:%.2f,integral:%.2f,", MPU6050.Yaw_Custom, angle_pid_yaw.integral);
      //printf("PID_Out:%.2f\r\n", angle_pid_yaw.final_output);
      encoder_sample = 0; // Reset the sample counter
    }

    /* This is the end of encoder print function */

    /* This is angle pid ring code */

    if (angle_pid_sample >= 49) // Update angle PID every 20ms (50Hz)
    {
      angle_pid_sample = 0; // Reset the sample counter
    }

    /* This the end of ring control */
    /* This is the space for you to pile up your test code */
    /* This is the end of your garbage space */
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Timer interruption is created here. Don't delete it!!! */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // All Timed Tasks Need to be marked here!!!
  if (htim == &htim2)
  {
    if (optical_flow_sample < 99)
    {
      optical_flow_sample = 0;
    }
    if (gyro_sample < 79)
    {
      gyro_sample++;
    }
    if (encoder_sample < 79)
    {
      encoder_sample++;
    }
    if (angle_pid_sample < 99) // Update angle PID every 20ms (50Hz)
    {
      angle_pid_sample++;
    }
    /* This is the space for your temp variables */
    /* End of temp */
  }
}
/* Don't Move This Code!!! */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
