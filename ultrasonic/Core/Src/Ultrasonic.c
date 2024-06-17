/*
 * Ultrasonic.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Yasmine
 */

#include "main.h"
#include "Ultrasonic.h"


void HCSR04_Read (GPIO_TypeDef* Port, uint16_t Pin,TIM_HandleTypeDef* htim)
{
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us     //////CHANGE THE DELAY
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
}
