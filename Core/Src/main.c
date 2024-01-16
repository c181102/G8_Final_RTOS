/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dht11.h"
#include "rgb.h"
#include "delay_timer.h"
#include "lcd_i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	DISPLAY_TEMP_HUMI,
	DISPLAY_TEMP,
	DISPLAY_HUMI,
} DisplayMode_t;

typedef enum
{
	TASK_READ_DATA,
	TASK_CONTROL_RGB,
} TaskIndex_t;

typedef struct
{
	TaskIndex_t TaskIndex;
	float buffDHT[2];
	uint8_t buffRGB[3];
} DataQueue_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_RX_LEN 13
#define RGB_COMMAND_LEN	12
#define DISPLAY_COMMAND_LEN 11
#define TIME_COMMAND_LEN 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ReadData */
osThreadId_t ReadDataHandle;
const osThreadAttr_t ReadData_attributes = {
  .name = "ReadData",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SendToCom */
osThreadId_t SendToComHandle;
const osThreadAttr_t SendToCom_attributes = {
  .name = "SendToCom",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlRgb */
osThreadId_t ControlRgbHandle;
const osThreadAttr_t ControlRgb_attributes = {
  .name = "ControlRgb",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for HandleInterrupt */
osThreadId_t HandleInterruptHandle;
const osThreadAttr_t HandleInterrupt_attributes = {
  .name = "HandleInterrupt",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for DataToCom */
osMessageQueueId_t DataToComHandle;
const osMessageQueueAttr_t DataToCom_attributes = {
  .name = "DataToCom"
};
/* Definitions for IRQSem */
osSemaphoreId_t IRQSemHandle;
const osSemaphoreAttr_t IRQSem_attributes = {
  .name = "IRQSem"
};
/* Definitions for DataSem */
osSemaphoreId_t DataSemHandle;
const osSemaphoreAttr_t DataSem_attributes = {
  .name = "DataSem"
};
/* USER CODE BEGIN PV */

RGB_t rgb;
uint32_t rgbInterval = 500;
uint8_t red = 0, green = 0, blue = 0;

DHT11_Sensor dht;
DHT11_Status dhtStatus;
uint32_t dhtInterval = 1500;

LCD_I2C_Name lcd;

uint8_t rxData[MAX_RX_LEN];
uint8_t rxDataIndex = 0;

DisplayMode_t DisplayMode = DISPLAY_TEMP_HUMI;

osStatus_t stat;

DataQueue_t DataQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void vTask_ReadData(void *argument);
void vTask_SendToCom(void *argument);
void vTask_ControlRgb(void *argument);
void vTask_HandleInterrupt(void *argument);
void vTask_Display(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		if (rxData[rxDataIndex] == '@')
		{
			rxData[rxDataIndex] = '\0';
			printf("\nCommand: %s\r\n", rxData);
			rxDataIndex = 0;
			if (osThreadGetPriority(HandleInterruptHandle) != osPriorityHigh)
			{
				osThreadSetPriority(HandleInterruptHandle, osPriorityHigh);
			}
			osSemaphoreRelease(IRQSemHandle);
		}
		else
		{
			rxDataIndex ++;
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData[rxDataIndex], 1);
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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("Start\r\n\n");

  DHT11_Init(&dht, DHT_GPIO_Port, DHT_Pin, &htim4);
  RGB_Init(&rgb, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);
  LCD_Init(&lcd, &hi2c2, LDC_DEFAULT_ADDRESS, 20, 4);

  LCD_SetCursor(&lcd, 0, 0);
  LCD_WriteString(&lcd, "Hello");

  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData[rxDataIndex],  1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of IRQSem */
  IRQSemHandle = osSemaphoreNew(1, 0, &IRQSem_attributes);

  /* creation of DataSem */
  DataSemHandle = osSemaphoreNew(1, 0, &DataSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of DataToCom */
  DataToComHandle = osMessageQueueNew (10, sizeof(uint32_t), &DataToCom_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ReadData */
  ReadDataHandle = osThreadNew(vTask_ReadData, NULL, &ReadData_attributes);

  /* creation of SendToCom */
  SendToComHandle = osThreadNew(vTask_SendToCom, NULL, &SendToCom_attributes);

  /* creation of ControlRgb */
  ControlRgbHandle = osThreadNew(vTask_ControlRgb, NULL, &ControlRgb_attributes);

  /* creation of HandleInterrupt */
  HandleInterruptHandle = osThreadNew(vTask_HandleInterrupt, NULL, &HandleInterrupt_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(vTask_Display, NULL, &Display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 554-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT_Pin */
  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void DisplayRgb(uint8_t r, uint8_t g, uint8_t b)
{
	printf("RED = %d - GREEN = %d - BLUE = %d\r\n", r, g, b);
}

static void DisplayDht(float temperature, float humidity)
{
	switch (DisplayMode)
	{
		case DISPLAY_TEMP:
			printf("Temperature: %.2f\r\n", temperature);
			break;
		case DISPLAY_HUMI:
			printf("Humidity: %.2f\r\n", humidity);
			break;
		default:
			printf("Temperature: %.2f\r\n", temperature);
			printf("Humidity: %.2f\r\n", humidity);
			break;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTask_ReadData */
/**
* @brief Function implementing the ReadData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTask_ReadData */
void vTask_ReadData(void *argument)
{
  /* USER CODE BEGIN vTask_ReadData */
	int32_t tick = osKernelGetTickCount();

  /* Infinite loop */
	for(;;)
	{
		if (osSemaphoreGetCount(DataSemHandle) == 0)
		{
			tick = tick + dhtInterval;
			printf("vTask_ReadData IN: %ld\r\n", osKernelGetTickCount());

			dhtStatus = DHT11_GetData(&dht);

			switch(dhtStatus)
			{
				case DHT11_ERR_CHECKSUM:
					printf("DHT11 ERROR CHECKSUM\r\n");
					break;
				case DHT11_ERR_RESPONSE:
					printf("DHT11 ERROR RESPONSE\r\n");
					break;
				default:
					printf("Get Data from DHT11 successfully\r\n");
					break;
			}

			printf("vTask_ReadData OUT: %ld\r\n\n", osKernelGetTickCount());

			if (dhtStatus == DHT11_OK)
			{
				osSemaphoreRelease(DataSemHandle);
				DataQueue.TaskIndex = TASK_READ_DATA;
				DataQueue.buffDHT[0] = dht.Temp;
				DataQueue.buffDHT[1] = dht.Humi;
				osMessageQueuePut(DataToComHandle, &DataQueue, 0, 0);
			}

			osDelayUntil(tick);
		}
	}
  /* USER CODE END vTask_ReadData */
}

/* USER CODE BEGIN Header_vTask_SendToCom */
/**
* @brief Function implementing the SendToCom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTask_SendToCom */
void vTask_SendToCom(void *argument)
{
  /* USER CODE BEGIN vTask_SendToCom */
  /* Infinite loop */
	for(;;)
	{
		osMessageQueueGet(DataToComHandle, &DataQueue, NULL, osWaitForever);
		printf("vTask_SendToCom IN: %ld\r\n", osKernelGetTickCount());

		switch(DataQueue.TaskIndex)
		{
			case TASK_CONTROL_RGB:
				DisplayRgb(DataQueue.buffRGB[0], DataQueue.buffRGB[1], DataQueue.buffRGB[2]);
				break;
			case TASK_READ_DATA:
				DisplayDht(DataQueue.buffDHT[0], DataQueue.buffDHT[1]);
				break;
			default:
				break;
		}

		printf("vTask_SendToCom OUT: %ld\r\n\n", osKernelGetTickCount());
	}
  /* USER CODE END vTask_SendToCom */
}

/* USER CODE BEGIN Header_vTask_ControlRgb */
/**
* @brief Function implementing the ControlRgb thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTask_ControlRgb */
void vTask_ControlRgb(void *argument)
{
  /* USER CODE BEGIN vTask_ControlRgb */
	int32_t tick = osKernelGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		tick = tick + rgbInterval;

		printf("vTask_ControlRgb IN: %ld\r\n", osKernelGetTickCount());

		RGB_SetValue(&rgb, red++, green++, blue++);

		printf("vTask_ControlRgb OUT: %ld\r\n\n", osKernelGetTickCount());

		if (osThreadGetPriority(ControlRgbHandle) != osPriorityBelowNormal)
		{
			osThreadSetPriority(ControlRgbHandle, osPriorityBelowNormal);
		}

		if (osMessageQueueGetSpace(DataToComHandle) != 0)
		{
			DataQueue.TaskIndex = TASK_CONTROL_RGB;
			DataQueue.buffRGB[0] = rgb.Data.red_value;
			DataQueue.buffRGB[1] = rgb.Data.green_value;
			DataQueue.buffRGB[2] = rgb.Data.blue_value;
			osMessageQueuePut(DataToComHandle, &DataQueue, 0, 0);
		}

		osDelayUntil(tick);
	}
  /* USER CODE END vTask_ControlRgb */
}

/* USER CODE BEGIN Header_vTask_HandleInterrupt */
/**
* @brief Function implementing the HandleInterrupt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTask_HandleInterrupt */
void vTask_HandleInterrupt(void *argument)
{
  /* USER CODE BEGIN vTask_HandleInterrupt */
  /* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(IRQSemHandle, osWaitForever);

		osThreadSuspend(ReadDataHandle);
		osThreadSuspend(SendToComHandle);

		const uint8_t* numPart;
		uint8_t buffer[5];
		printf("\n");
		switch (strlen((const char*)rxData))
		{
			case RGB_COMMAND_LEN:
				if (rxData[0] == 'r' && rxData[1] == 'g' && rxData[2] == 'b')
				{
					numPart = rxData + 3;
					for (int i = 0; i < 3; i++)
					{
						strncpy(buffer, (const char*)(numPart + i * 3), 3);
						buffer[3] = '\0'; // Null-terminate the buffer

						switch (i)
						{
							case 0: red = (uint8_t)atoi((const char*)buffer); break;
							case 1: green = (uint8_t)atoi((const char*)buffer); break;
							case 2: blue = (uint8_t)atoi((const char*)buffer); break;
						}
					}

					printf("Change RGB color:\r\n");
					printf("RED = %d\r\nGREEN = %d\r\nBLUE = %d\r\n\n", red, green, blue);

					osThreadSetPriority(ControlRgbHandle, osThreadGetPriority(HandleInterruptHandle) - 1);
					osThreadSuspend(ControlRgbHandle);
					osThreadResume(ControlRgbHandle);
				}
				else
				{
					printf("Error Command Syntax\r\n\n");
				}
				break;
			case DISPLAY_COMMAND_LEN:
				if (strcmp((const char*)rxData, "displaytemp") == 0)
				{
					DisplayMode = DISPLAY_TEMP;
					printf("Change Display Mode to DISPLAY_TEMP\r\n\n");
				}
				else if (strcmp((const char*)rxData, "displayhumi") == 0)
				{
					DisplayMode = DISPLAY_HUMI;
					printf("Change Display Mode to DISPLAY_HUMI\r\n\n");
				}
				else if (strcmp((const char*)rxData, "displayboth") == 0)
				{
					DisplayMode = DISPLAY_TEMP_HUMI;
					printf("Change Display Mode to DISPLAY_TEMP_HUMI\r\n\n");
				}
				else
				{
					printf("Error Command Syntax\r\n\n");
				}
				break;
			case TIME_COMMAND_LEN:
				if(rxData[0] == 't' && rxData[1] == 'i' && rxData[2] == 'm' && rxData[3] == 'e')
				{
					numPart = rxData + 4;
					strncpy(buffer, (const char*)numPart, 4);
					buffer[4] = '\0'; // Null-terminate the buffer
					uint32_t old_dhtInterval = dhtInterval;
					dhtInterval = (uint32_t)atoi((const char*)buffer);

					if (dhtInterval > 1500 && dhtInterval < 10000)
					{
						printf("Change DHT11 Period from %d to %d\r\n\n", old_dhtInterval, dhtInterval);
					}
					else
					{
						printf("The new Period is invalid! DHT11 Period stays the same: %d\r\n\n", old_dhtInterval);
						dhtInterval = old_dhtInterval;
					}
				}
				else
				{
					printf("Error Command Syntax\r\n\n");
				}
				break;
			default:
				printf("Error Command Syntax\r\n\n");
				break;
		}

		osThreadResume(ReadDataHandle);
		osThreadResume(SendToComHandle);
  }
  /* USER CODE END vTask_HandleInterrupt */
}

/* USER CODE BEGIN Header_vTask_Display */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTask_Display */
void vTask_Display(void *argument)
{
  /* USER CODE BEGIN vTask_Display */
	char temp[15], humi[15];
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(DataSemHandle, osWaitForever);
	printf("vTask_Display IN: %ld\r\n", osKernelGetTickCount());

  	if (dhtStatus == DHT11_OK)
  	{
  	  	sprintf(temp, "Temp: %.2f", dht.Temp);
  	  	sprintf(humi, "Humi: %.2f", dht.Humi);

  	  	LCD_Clear(&lcd);

  	  	switch (DisplayMode)
  	  	{
  	  		case DISPLAY_TEMP:
  	  			LCD_SetCursor(&lcd, 0, 0);
  	  			LCD_WriteString(&lcd, temp);
  	  			break;
  	  		case DISPLAY_HUMI:
  	  			LCD_SetCursor(&lcd, 0, 0);
  	  			LCD_WriteString(&lcd, humi);
  	  			break;
  	  		default:
  	  			LCD_SetCursor(&lcd, 0, 0);
  	  			LCD_WriteString(&lcd, temp);
  	  			LCD_SetCursor(&lcd, 0, 1);
  	  			LCD_WriteString(&lcd, humi);
  	  			break;
  	  	}
  	}

  	printf("vTask_Display OUT: %ld\r\n\n", osKernelGetTickCount());
  }
  /* USER CODE END vTask_Display */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
