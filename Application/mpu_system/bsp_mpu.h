#ifndef BSP_H_
#define BSP_H_
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

#define TRUE 1
#define FALSE 0
#define UART_TIMEOUT 100

uint8_t i2c_init(I2C_HandleTypeDef * handle);
uint8_t i2c_device_ready(uint16_t addr);
uint8_t i2c_transmit(uint16_t addr, uint8_t * buff, uint8_t size);
uint8_t i2c_receive(uint16_t addr, uint8_t * buff, uint8_t size);

uint8_t toggle_led(void);
uint8_t delay(uint32_t time);

uint8_t tim4_init(TIM_HandleTypeDef * tim);
uint8_t tim4_start_it(uint16_t counting);
uint8_t tim4_stop(void);


uint8_t uart_init(UART_HandleTypeDef * handle);
uint8_t uart_transmit(uint8_t * data, uint8_t size);
uint8_t uart_receive(uint8_t * data, uint8_t size);
uint8_t uart_receive_it(uint8_t * data, uint8_t size);


void set_sem_indutivo(uint8_t value);
uint8_t get_sem_indutivo(void);
#endif /* BSP_H_ */
