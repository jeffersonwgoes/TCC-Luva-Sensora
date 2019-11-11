/*
 * user.h
 *
 *  Created on: 25 de jul de 2018
 *      Author: jeffe
 */

#ifndef INDUTIVO_H_
#define INDUTIVO_H_
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "pt.h"
#include "bsp_mpu.h"

#define 	Sens_Port 						GPIOA
#define 	Mux_Port						GPIOB
#define 	PinSet(Port, Pin) 				HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET)
#define		PinReset(Port, Pin)				HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET)
#define 	MUXPin							GPIOB
#define		Mux0							MUX0_Pin
#define		Mux1							MUX1_Pin
#define		Mux2							MUX2_Pin

typedef enum
{
	S1=1, S2, S3, S4, S5, S6, S7, S8
}sensor_t;

typedef enum
{
	G1=1, G2, G3, G4, G5, G6
}generator_t;

void ind_set_ger(generator_t gerador);
void ind_set_sen(sensor_t sensor);
void ind_set_pwm(uint16_t count);
void ind_set_adc(ADC_HandleTypeDef * adc);
void ind_set_tim(TIM_HandleTypeDef * tim);
void ind_start_conversion(void);
uint8_t ind_get_conv_finish(void);
uint16_t ind_get_conv_value(void);

#endif /* INDUTIVO_H_ */
