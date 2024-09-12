/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "Myproject.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern float voltage_power_supply;
extern float voltage_limit;
extern int  pole_pairs;
extern float velocity_limit;
extern MotionControlType controller;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Rx_Buffer[32]; //接收缓存区
uint16_t Rx_Flag = 0;//接收标志位

float target;//电机目标转速
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//USART接收回调函数
{
	if(huart->Instance == USART2)
	{
		static uint16_t index = 0;
		
		index++;
		if(Rx_Flag != 2)//接收未完成
		{
			if(Rx_Flag == 1)//接收到了\r
			{
				if(Rx_Buffer[index-1] != '\n')
				{
					Rx_Flag = 0;//接收错误，重新开始
					index = 0;
				}					
				else
				{
					Rx_Flag = 2;//接收完成  最后一个字节放'0’
					Rx_Buffer[31]='\0';
					index = 0;
				}
			}
			else
			{
				if(Rx_Buffer[index-1] == '\r') Rx_Flag = 1;
			}
			
		}
		HAL_UART_Receive_IT(&huart2,Rx_Buffer+index,1);
	}
}

void commander_run(void)
{
	if(Rx_Flag == 2)
	{
		if(Rx_Buffer[0] == 'T')
		{
			target=atof((const char *)(Rx_Buffer+1));
			printf ("Rx=%.4f\r\n",target);
		}
		else
		{
			printf ("input error\r\n");
		}
		Rx_Flag = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		move(target);
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //电机启动信息
  voltage_power_supply=24;   //V
  voltage_limit=3;         //V，最大值需小于12/1.732=6.9。大功率航模电机设置的小一点0.5-1；小功率云台电机设置的大一点1-3
  velocity_limit=20;         //rad/s angleOpenloop() use it
  controller=Type_velocity_openloop;  //Type_velocity_openloop; //Type_angle_openloop;
  pole_pairs=7;              //极对数
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  
  HAL_TIM_Base_Start_IT(&htim2);
  
  //Motor Enable
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);//EN1
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);//EN2
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);//EN3
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);//DIAG/EN——全桥使能
  
  printf("Motor Ready.\r\n");
  
  HAL_UART_Receive_IT(&huart2,Rx_Buffer,1);
  
  target = 10;//初始转速
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	commander_run();
	//move(target);
	//HAL_Delay(10);//对解决电机抖动效果不好
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
