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
#include "adc.h"
#include "dma.h"
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
//extern float voltage_power_supply;
//extern float voltage_limit;
//extern int  pole_pairs;
//extern float velocity_limit;
//extern float voltage_sensor_align;
//extern MotionControlType controller;
//extern TorqueControlType torque_controller;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Rx_Buffer[32]; //���ջ�����
uint16_t Rx_Flag = 0;//���ձ�־λ
float target;//���Ŀ��ת��
float f3[3];

uint16_t ad_value[2];//���������洢
uint16_t current_offset[10];
uint16_t offset_channel_1=0;
uint16_t offset_channel_2=0;
int16_t current_a_count;
int16_t current_b_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1)
	{
		current_a_count = ad_value[0]-offset_channel_1;
		current_b_count = ad_value[1]-offset_channel_2;
//		printf("[0]:%d\r\n",ad_value[0]-offset_channel_1);
//		printf("[1]:%d\r\n",ad_value[1]-offset_channel_2);
		//printf("value:%d",HAL_ADC_GetValue(&hadc1));
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//USART���ջص�����
{
	if(huart->Instance == USART2)
	{
		static uint16_t index = 0;
		
		index++;
		if(Rx_Flag != 2)//����δ���
		{
			if(Rx_Flag == 1)//���յ���\r
			{
				if(Rx_Buffer[index-1] != '\n')
				{
					Rx_Flag = 0;//���մ������¿�ʼ
					index = 0;
				}					
				else
				{
					Rx_Flag = 2;//�������  ���һ���ֽڷ�'0��
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			updateState();
		}
		
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//for test
//{
//	if(htim->Instance==TIM2)
//	{
//		//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)(ad_value),2);
//		//printf("$%f;",getVelocity());
//		f3[0]=getVelocity();
//		f3[1]=voltage.q;
//		f3[2]=voltage.d;
//		Vofa_JustFloat(f3,3);
//	}
//}
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	voltage_power_supply=24;   //V
	voltage_limit=5;           //V�����ֵС��voltage_power_supply/1.732 
							   //����ģʽ��Uq=voltage_limit;dq��������޷�ֵ;Type_voltageģʽ���ٶ��޷�ֵ
	velocity_limit=20;         //rad/s ����λ��ģʽ��λ��ģʽ��ʹ�ã���ΪPI limit
	current_limit=10;          //A��foc_current��dc_currentģʽ���Ƶ���������Ϊ0���ٶ�ģʽ��λ��ģʽ������
	//voltage_sensor_align=2;    //V  ��Ҫ��������ģ�������0.5-1����̨���С����2-3
	torque_controller = Type_foc_current; //Type_dc_current;//Type_foc_current;//Type_voltage;
	controller=Type_velocity;  //Type_angle; //Type_torque; //Type_velocity;
	pole_pairs=7;              //������
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	//Motor Enable
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);//EN1
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);//EN2
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);//EN3
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);//DIAG_EN����ȫ��ʹ��
	
	HAL_UART_Receive_IT(&huart2,Rx_Buffer,1);
	
	//����TIM2 CC1�жϵ�һ����
	TIM_SlaveConfigTypeDef SlaveConfig;
	SlaveConfig.SlaveMode =  TIM_SLAVEMODE_DISABLE;
	SlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
	SlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_BOTHEDGE;
	HAL_TIM_SlaveConfigSynchronization(&htim2, &SlaveConfig);
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	
	PID_init(); 
	LPF_init();
	
	SysTick_Config(16777215);  // ����SysTick��װ��ֵ,���ֵ16777215
	
	target = 100;//��ʼת��
	for(int i=0;i<5;i++)//ȷ������������ʼֵ,����5�����ֵ
	{
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)(current_offset+2*i),2);
	}
	printf("Motor ready\r\n");//��DMAд���ʱ�䣬ʵ��֤�����������䣬current_offset[9]��ʱ��δд��
	
	offset_channel_1 = (current_offset[0]+current_offset[2]+current_offset[4]+current_offset[6]+current_offset[8])/5;
	offset_channel_2 = (current_offset[1]+current_offset[3]+current_offset[5]+current_offset[7]+current_offset[9])/5;
	printf("HCLK Frequency: %u\n", HAL_RCC_GetHCLKFreq());
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //������HAL_Delay()����
	  move(target);
	  loopFOC();
	  commander_run();
//		f3[0]=getVelocity();
//		f3[1]=voltage.q;
//		f3[2]=voltage.d;
//		Vofa_JustFloat(f3,3);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
