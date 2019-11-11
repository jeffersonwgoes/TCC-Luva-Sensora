#include "main.h"
#include "stm32f1xx_hal.h"
//b#include "main_application.h"
#include "classification_system.h"
#include "lcd16x2.h"
#include "bsp_mpu.h"
#include "indutivo.h"
#include "mpu6050.h"
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

const generator_t ger[] = {G1 , G1 , G1 , G1 , G1 , G2,  G3 , G3 , G3 , G3 , G4 , G5 , G6 };
const sensor_t    sen[] = {S1 , S2 , S3 , S4 , S5 , S5 , S1 , S2 , S3 , S6 , S6 , S7 , S8 };
const uint16_t    pwm[] = {674, 676, 665, 678, 645, 667, 682, 682, 677, 683, 676, 667, 682};
uint16_t valores[13];
char result = '-';
uint8_t uart_d;
volatile uint8_t xSemMpu = 0;
volatile uint8_t xSemSerial = 0;
mpu_t mpu;
int16_t x[10];
int16_t y[10];
int16_t z[10];
int16_t roll[10];
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void classifyMoviment(uint8_t signal);
void getMpuValues(void);

void StartProjeto(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	ind_set_tim(&htim1);
	ind_set_adc(&hadc1);
	i2c_init(&hi2c1);
	tim4_init(&htim4);
	uart_init(&huart3);
	uart_receive_it(&uart_d, 1);
	lcd16x2_init(LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
	lcd16x2_puts("Iniciando Sensor MPU");
	for(uint8_t i = 0; i < 10; i++) {
		HAL_Delay(1000);
		lcd16x2_clrscr();
		lcd16x2_puts("Iniciando em ");
		lcd16x2_gotoxy(0, 1);
		lcd16x2_putc(i + 49);
	}
	lcd16x2_putc(result);
	if(!i2c_device_ready(MPU_ADDR)) {
		asm("NOP");
		while(1);
	}

	mpu_init(&mpu);
	mpu.acc_res = ACC_SENS_2;
	mpu_set_offset_acc_x(ACC_OFF_X);
	mpu_set_offset_acc_y(ACC_OFF_Y);
	mpu_set_offset_acc_z(ACC_OFF_Z);
	lcd16x2_clrscr();
	lcd16x2_gotoxy(0, 0);
	lcd16x2_puts("TCC LIBRAS\nLETRA: -");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	xSemMpu = 1;
	xSemSerial = 1;
	uart_receive_it(&uart_d, 1);
}

void feedTheInductiveSensor(void) {
	for(uint8_t i = 0; i < 12; i++) {
		ind_set_ger(ger[i]);
		ind_set_sen(sen[i]);
		ind_set_pwm(pwm[i]);
		HAL_Delay(5);
		ind_start_conversion();
		while(!ind_get_conv_finish());
		valores[i] = ind_get_conv_value();
		if(i == 4) {
			asm("NOP");
		}
	}
}

void classifyLetter(void) {
	lcd16x2_gotoxy(7, 1);
	float input[NUMBER_OF_INPUTS];
	float outpu[NUMBER_OF_OUTPUTS];
	for(uint8_t i = 0; i < NUMBER_OF_INPUTS; i++)
		input[i] = (float)valores[i];
	classSystemScaleDataInput(input);
	classSystemPredict(input, outpu);
	result = classSystemPostProcess(outpu);
	HAL_Delay(50);
	if(result == '1' || result == '2' || result == '3') {
		getMpuValues();
		classifyMoviment(result);
	} else {
		lcd16x2_gotoxy(7, 1);
		lcd16x2_putc(result);
	}

}

void classifyMoviment(uint8_t signal) {
	float input[NUMBER_OF_INPUTS_A];
	float output[NUMBER_OF_OUTPUTS_A];
	for(uint8_t i = 0; i < NUMBER_OF_INPUTS_A; i+=4) {
		input[i] = x[i/4];
		input[i+1] = y[i/4];
		input[i+2] = z[i/4];
		input[i+3] = roll[i/4];
	}
	classSystemScaleDataInputA(input);
	classSystemPredictA(input, output);
	result = classSystemPostProcessA(output);
	if(result == '-' && signal == '1') {
		result = 'P';
	} else if(result == '-' && signal == '2') {
		result = 'I';
	} else if(result == '-' && signal == '3') {
		result = 'Z';
	}
	lcd16x2_gotoxy(7, 1);
	lcd16x2_putc(result);
}

void getMpuValues(void) {
	xSemMpu = 0;
	for(uint8_t i = 0; i < 10; i++) {
		mpu_read_all(&mpu);
		x[i] = mpu.raw_acc_x;
		y[i] = mpu.raw_acc_y;
		z[i] = mpu.raw_acc_z;
		roll[i] = mpu.calc_roll;
		HAL_Delay(50);
	}
}

void sendToSerial(void) {
	char data[10];
	uint8_t len;
	set_sem_indutivo(0);
	xSemSerial = 0;

	for(uint8_t i = 0; i < 12; i++) {
		len = sprintf(data, "%d, ", valores[i]);
		uart_transmit((uint8_t*)data, len);
	}
	len = sprintf(data, "%d", valores[12]);
	uart_transmit((uint8_t*)data, len);
	uart_receive_it(&uart_d, 1);
}

int16_t xx[900], yy[900], zz[900], rolls[900];
int control;
int main(void)
{


	StartProjeto();

	asm("NOP");

	uint32_t moment = HAL_GetTick();
	for(control = 0; control < 900; control++) {
		if((HAL_GetTick() - moment) > 500)
			break;
		mpu_read_all(&mpu);
		xx[control] = mpu.raw_acc_x;
		yy[control] = mpu.raw_acc_y;
		zz[control] = mpu.raw_acc_z;
		rolls[control] = mpu.calc_roll;
	}
	asm("NOP");
	while (1)
	{
		feedTheInductiveSensor();
		if(xSemSerial) {
			sendToSerial();
		}
		classifyLetter();
	}
}

void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RCC_EnableCSS();
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}


static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}


static void MX_TIM1_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 656;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);

}

static void MX_TIM4_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 6400;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 49;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_USART3_UART_Init(void)
{

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, S7_Pin|S5_Pin|S6_Pin|S4_Pin
			|S3_Pin|S2_Pin|S1_Pin|RS_Pin
			|RW_Pin|EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, S8_Pin|D7_Pin|D6_Pin|D5_Pin
			|D4_Pin|MUX0_Pin|MUX1_Pin|MUX2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : S7_Pin S5_Pin S6_Pin S4_Pin
                           S3_Pin S2_Pin S1_Pin RS_Pin 
                           RW_Pin EN_Pin */
	GPIO_InitStruct.Pin = S7_Pin|S5_Pin|S6_Pin|S4_Pin
			|S3_Pin|S2_Pin|S1_Pin|RS_Pin
			|RW_Pin|EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : S8_Pin D7_Pin D6_Pin D5_Pin
                           D4_Pin MUX0_Pin MUX1_Pin MUX2_Pin */
	GPIO_InitStruct.Pin = S8_Pin|D7_Pin|D6_Pin|D5_Pin
			|D4_Pin|MUX0_Pin|MUX1_Pin|MUX2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
