/*
 * Ultrasonic.h
 *
 *  Created on: Jun 17, 2024
 *      Author: Yasmine
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

void HCSR04_Read (GPIO_TypeDef* Port, uint16_t Pin,TIM_HandleTypeDef* htim);

#endif /* INC_ULTRASONIC_H_ */
