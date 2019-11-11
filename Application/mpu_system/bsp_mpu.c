#include <bsp_mpu.h>
#include "math.h"

I2C_HandleTypeDef 	* i2c_h;
TIM_HandleTypeDef 	* tim1;
TIM_HandleTypeDef 	* tim4;
UART_HandleTypeDef 	* uart;
ADC_HandleTypeDef   * adc;

volatile uint8_t flag_indutivo = FALSE;
volatile uint8_t flag_uart = FALSE;

uint8_t i2c_init(I2C_HandleTypeDef * handle)
{
    i2c_h = handle;
    return TRUE;
}

uint8_t i2c_device_ready(uint16_t addr)
{
    uint8_t ret = FALSE;
    if(HAL_I2C_IsDeviceReady(i2c_h, addr<<1, 10, 1000) == HAL_OK) ret = TRUE;

    return ret;
}

uint8_t i2c_transmit(uint16_t addr, uint8_t * buff, uint8_t size)
{
	while(HAL_I2C_Master_Transmit(i2c_h, addr<<1, buff, size, 1000) != HAL_OK);
	return TRUE;
}

uint8_t i2c_receive(uint16_t addr, uint8_t * buff, uint8_t size)
{
	while(HAL_I2C_Master_Receive(i2c_h, addr<<1, buff, size, 1000) != HAL_OK);
	return TRUE;
}

uint8_t toggle_led(void) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    return TRUE;
}

uint8_t delay(uint32_t time) {
    HAL_Delay(time);
    return TRUE;
}

uint8_t tim4_init(TIM_HandleTypeDef * tim) {
    tim4 = tim;
    return TRUE;
}

uint8_t tim4_start_it(uint16_t counting) {
    __HAL_TIM_SET_AUTORELOAD(tim4, counting);
    HAL_TIM_Base_Start_IT(tim4);
    return TRUE;
}

uint8_t tim4_stop(void) {
	HAL_TIM_Base_Stop(tim4);
	return TRUE;
}

uint8_t uart_init(UART_HandleTypeDef * handle) {
	uart = handle;
	return TRUE;
}

uint8_t uart_transmit(uint8_t * data, uint8_t size) {
	uint8_t ret;
	if(HAL_UART_Transmit(uart, data, size, UART_TIMEOUT) == HAL_OK) {
		ret = TRUE;
	} else {
		ret = FALSE;
	}
	
	return ret;
}

uint8_t uart_receive(uint8_t * data, uint8_t size) {
	uint8_t ret;
	if(HAL_UART_Receive(uart, data, size, UART_TIMEOUT) == HAL_OK) {
		ret = TRUE;
	} else {
		ret = FALSE;
	}
	return ret;
}

uint8_t uart_receive_it(uint8_t * data, uint8_t size) {
	uint8_t ret;
	if(HAL_UART_Receive_IT(uart, data, size) == HAL_OK) {
		ret = TRUE;
	} else {
		ret = FALSE;
	}
	return ret;
}

//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	flag_indutivo = TRUE;
//}

void set_sem_indutivo(uint8_t value) {
	flag_indutivo = value;
}

uint8_t get_sem_indutivo(void) {
	return flag_indutivo;
}


