/*
 * main_application.c
 *
 *  Created on: 30 de abr de 2019
 *      Author: Jefferson
 */
#include "main_application.h"

int16_t ind_values[13];
uint8_t xSemClassification = FALSE;
/*const generator_t ger[] = {G1 , G1 , G1 , G1 , G1 , G2,  G3 , G3 , G3 , G3 , G4 , G5 , G6 };
const sensor_t    sen[] = {S1 , S2 , S3 , S4 , S5 , S5 , S1 , S2 , S3 , S6 , S6 , S7 , S8 };
const uint16_t    pwm[] = {674, 676, 665, 678, 645, 667, 682, 682, 677, 683, 676, 667, 682};*/
uint8_t xSemIndData = FALSE;
volatile uint8_t xSemSerial = FALSE;

uint32_t get_tick(void);
uint32_t time_elapsed(uint32_t tick);
static int indutivoTask(struct pt * pt);
static int classificationTask(struct pt * pt);
static int serialTask(struct pt * pt);
static int mpuTask(struct pt * pt);

void app_run(void)
{
	static struct pt ptIndutivoHandler;
	static struct pt ptMpuHandler;
	static struct pt ptClassificationHandler;
	static struct pt ptSerialHandler;

	lcdInit(LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
	lcdPrint("TCC LIBRAS\nLETRA: ");
	PT_INIT(&ptIndutivoHandler);
	PT_INIT(&ptMpuHandler);
	PT_INIT(&ptClassificationHandler);
	PT_INIT(&ptSerialHandler);
//	PT_INIT();
	while(1) {
		/*indutivoTask(&ptIndutivoHandler);
		classificationTask(&ptClassificationHandler);*/
		serialTask(&ptSerialHandler);
		mpuTask(&ptMpuHandler);
	}
}

static int mpuTask(struct pt * pt)
{
	PT_BEGIN(pt);
	while(i2c_device_ready(MPU_ADDR<<1)) {
		asm("NOP");
	}
	while(1) {
		PT_YIELD(pt);
	}
	PT_END(pt);
}

static int indutivoTask(struct pt * pt)
{
	PT_BEGIN(pt);
	static uint32_t tick;
	static uint8_t par;
	while(1) {
		/*for(par = 0; par < 13; par++) {
			ind_set_ger(ger[par]);
			ind_set_sen(sen[par]);
			ind_set_pwm(pwm[par]);
			tick = get_tick();
			while(time_elapsed(tick) < 4) PT_YIELD(pt);

			ind_start_conversion();
			while(!ind_get_conv_finish()) PT_YIELD(pt);
			ind_values[par] = ind_get_conv_value();
		}
		xSemIndData = TRUE;*/
		PT_YIELD(pt);
	}
	PT_END(pt);
}

static int classificationTask(struct pt * pt)
{
	PT_BEGIN(pt);

	static float input[NUMBER_OF_INPUTS];
	static char letterClassified = '-';
	while(1) {
		if(xSemIndData == TRUE) {
			for(uint8_t i = 0; i < NUMBER_OF_INPUTS; i++) {
				input[i] = (float)ind_values[i];
			}
			float output[NUMBER_OF_INPUTS];
			xSemIndData = FALSE;
			classSystemScaleDataIput(input);
			classSystemPredict(input, output);
			letterClassified = classSystemPostProcess(output);
			PT_YIELD(pt);
			lcd16x2_gotoxy(8, 1);
			lcd16x2_putc(letterClassified);

		}
		PT_YIELD(pt);

	}
	PT_END(pt);
}

static int serialTask(struct pt * pt)
{
	PT_BEGIN(pt);
	while(1) {
		/*if(xSemIndData == TRUE && xSemSerial == TRUE) {
			xSemSerial = FALSE;
			char data[20];
			uint8_t len = 0;
			for(uint8_t i = 0; i < 12; i++) {
				len = sprintf(data, "%d,", ind_values[i]);
				uart_send((uint8_t*)data, len);
			}
			len = sprintf(data, "%d", ind_values[12]);
			uart_send((uint8_t*)data, len);*/
			PT_YIELD(pt);
//		}
	}
	PT_END(pt);
}


uint32_t get_tick(void) {
	return HAL_GetTick();
}

uint32_t time_elapsed(uint32_t tick) {
	return HAL_GetTick() - tick;
}
