/* Private includes ----------------------------------------------------------*/
#include "rgb.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/

static void assign_value(RGB_t *rgb)
{
	switch (rgb->red_channel)
	{
		case TIM_CHANNEL_1:
			rgb->Timer->Instance->CCR1 = rgb->Data.red_value;
			break;
		case TIM_CHANNEL_2:
			rgb->Timer->Instance->CCR2 = rgb->Data.red_value;
			break;
		case TIM_CHANNEL_3:
			rgb->Timer->Instance->CCR3 = rgb->Data.red_value;
			break;
		case TIM_CHANNEL_4:
			rgb->Timer->Instance->CCR4 = rgb->Data.red_value;
			break;
	}

	switch (rgb->green_channel)
	{
		case TIM_CHANNEL_1:
			rgb->Timer->Instance->CCR1 = rgb->Data.green_value;
			break;
		case TIM_CHANNEL_2:
			rgb->Timer->Instance->CCR2 = rgb->Data.green_value;
			break;
		case TIM_CHANNEL_3:
			rgb->Timer->Instance->CCR3 = rgb->Data.green_value;
			break;
		case TIM_CHANNEL_4:
			rgb->Timer->Instance->CCR4 = rgb->Data.green_value;
			break;
	}

	switch (rgb->blue_channel)
	{
		case TIM_CHANNEL_1:
			rgb->Timer->Instance->CCR1 = rgb->Data.blue_value;
			break;
		case TIM_CHANNEL_2:
			rgb->Timer->Instance->CCR2 = rgb->Data.blue_value;
			break;
		case TIM_CHANNEL_3:
			rgb->Timer->Instance->CCR3 = rgb->Data.blue_value;
			break;
		case TIM_CHANNEL_4:
			rgb->Timer->Instance->CCR4 = rgb->Data.blue_value;
			break;
	}
}

void RGB_Init(RGB_t* rgb, TIM_HandleTypeDef* htim, uint32_t red_channel, uint32_t green_channel, uint32_t blue_channel)
{
    rgb->Timer = htim;
    rgb->red_channel  = red_channel;
    rgb->green_channel = green_channel;
    rgb->blue_channel = blue_channel;

    HAL_TIM_PWM_Start(rgb->Timer, red_channel);
	HAL_TIM_PWM_Start(rgb->Timer, green_channel);
	HAL_TIM_PWM_Start(rgb->Timer, blue_channel);

 	rgb->Data.red_value = 0;
 	rgb->Data.green_value = 0;
 	rgb->Data.blue_value = 0;

    assign_value(rgb);
}

void RGB_SetValue(RGB_t *rgb, uint8_t red, uint8_t green, uint8_t blue)
{
 	rgb->Data.red_value = red;
 	rgb->Data.green_value = green;
 	rgb->Data.blue_value = blue;

	assign_value(rgb);   
}
