/*
 * Ultrasonic.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Yasmine
 */

#include "main.h"
#include "Ultrasonic.h"


void HCSR04_Read (GPIO_TypeDef* Port, uint16_t Pin,TIM_HandleTypeDef* htim, uint8_t Channel)
{
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	osDelay(1);
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(htim, Channel);
//	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
//	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
//	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
}
