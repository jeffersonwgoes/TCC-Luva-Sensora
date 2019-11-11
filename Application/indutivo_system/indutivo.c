/*
 * user.c
 *
 *  Created on: 25 de jul de 2018
 *      Author: jeffe
 */

#include "indutivo.h"

TIM_HandleTypeDef * tim1;
ADC_HandleTypeDef * adc1;
volatile uint16_t value_adc;
volatile uint8_t adc_complete = 0;

void ind_set_adc(ADC_HandleTypeDef * adc)
{
    adc1 = adc;
}

void ind_set_tim(TIM_HandleTypeDef * tim)
{
    tim1 = tim;
}

void ind_set_ger(generator_t gerador)
{
    switch(gerador)
    {
    case G6:
        HAL_GPIO_WritePin(MUXPin, Mux0|Mux1|Mux2, GPIO_PIN_RESET);
        break;
    case G5:
        HAL_GPIO_WritePin(MUXPin, Mux0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUXPin, Mux1|Mux2, GPIO_PIN_RESET);
        break;
    case G4:
        HAL_GPIO_WritePin(MUXPin, Mux0|Mux2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUXPin, Mux1, GPIO_PIN_SET);
        break;
    case G3:
        HAL_GPIO_WritePin(MUXPin, Mux0|Mux1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUXPin, Mux2, GPIO_PIN_RESET);
        break;
    case G2:
        HAL_GPIO_WritePin(MUXPin, Mux0|Mux1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUXPin, Mux2, GPIO_PIN_SET);
        break;
    case G1:
        HAL_GPIO_WritePin(MUXPin, Mux0|Mux2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUXPin, Mux1, GPIO_PIN_RESET);
        break;
    }
}

void ind_set_sen(sensor_t sensor)
{
    switch(sensor)
    {
    case S1:
        S1_GPIO_Port->BRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S2:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S3:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S4:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S5:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S6:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S7:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BRR = S7_Pin;
        S8_GPIO_Port->BSRR = S8_Pin;
        break;
    case S8:
        S1_GPIO_Port->BSRR = S1_Pin;
        S2_GPIO_Port->BSRR = S2_Pin;
        S3_GPIO_Port->BSRR = S3_Pin;
        S4_GPIO_Port->BSRR = S4_Pin;
        S5_GPIO_Port->BSRR = S5_Pin;
        S6_GPIO_Port->BSRR = S6_Pin;
        S7_GPIO_Port->BSRR = S7_Pin;
        S8_GPIO_Port->BRR = S8_Pin;
        break;
    }
}

void ind_set_pwm(uint16_t count)
{
    HAL_TIM_PWM_Stop(tim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_AUTORELOAD(tim1, count);
    __HAL_TIM_SET_COMPARE(tim1, TIM_CHANNEL_4, count>>1);
    HAL_Delay(1);
    HAL_TIM_PWM_Start(tim1, TIM_CHANNEL_4);
}

void ind_start_conversion(void) 
{
    HAL_ADC_Start_IT(adc1);
    adc_complete = 0;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    value_adc = HAL_ADC_GetValue(hadc);
    adc_complete = 1;
}

uint8_t ind_get_conv_finish(void) {
    return adc_complete;
}

uint16_t ind_get_conv_value(void) {
    return value_adc;
}

