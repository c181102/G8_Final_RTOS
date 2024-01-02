
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RGB_H
#define __RGB_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint8_t red_value;
    uint8_t green_value;
    uint8_t blue_value;
} RGB_Data;

typedef struct 
{
    TIM_HandleTypeDef* Timer;
    uint32_t red_channel;
	uint32_t green_channel;
	uint32_t blue_channel;
    RGB_Data Data;
} RGB_t;

typedef enum
{
    RGB_OK,
    RGB_ERR,
} RGB_Status;


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/

void RGB_Init(RGB_t* rgb, TIM_HandleTypeDef* htim, uint32_t red_channel, uint32_t green_channel, uint32_t blue_channel);
void RGB_SetValue(RGB_t *rgb, uint8_t red, uint8_t green, uint8_t blue);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#endif /* __RGB_H */
