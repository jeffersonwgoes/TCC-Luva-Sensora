#include "mpu6050.h"
#include "math.h"
#include "bsp_mpu.h"

#define buffersize 		1000
#define acel_deadzone	8

int mean_ax,mean_ay,mean_az;
int ax_offset,ay_offset,az_offset;

static uint8_t mpu_write_word(uint8_t reg, int16_t word);

uint8_t mpu_init(mpu_t * data)
{
	
	if(i2c_device_ready(MPU_ADDR) != TRUE) {
	    goto ERROR;
	}
	
	uint8_t buff[5];
	buff[0] = WHO_I_AM;
	if(i2c_transmit(MPU_ADDR, buff, 1) != TRUE) {
		goto ERROR;
	}
	
	/* wake up the device */
	buff[0] = PWR_MGMT_1;
	buff[1] = 0x00;
	
	if(i2c_transmit(MPU_ADDR, buff, 2) != TRUE) {
		goto ERROR;
	}

	return TRUE;

ERROR:
    return FALSE;

}

void meansensors(mpu_t * mpu){
	long i=0,buff_ax=0,buff_ay=0,buff_az=0;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	for(i = 0;i<(buffersize+101); i++){
		mpu_read_all(mpu);
		if (i > 100 && i <= (buffersize + 100)){ //First 100 measures are discarded
			buff_ax = buff_ax + mpu->raw_acc_x;
			buff_ay = buff_ay + mpu->raw_acc_y;
			buff_az = buff_az + mpu->raw_acc_z;
		}
		if(i == 500) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		HAL_Delay(2); //Needed so we don't get repeated measures
	}

	mean_ax = buff_ax / buffersize;
	mean_ay = buff_ay / buffersize;
	mean_az = buff_az / buffersize;

}

uint8_t mpu_calibration(mpu_t * mpu) 
{
	uint8_t ready = 0;
	meansensors(mpu);
	ax_offset =- mean_ax/8;
	ay_offset =- mean_ay/8;
	az_offset = (16384-mean_az)/8;
	while(ready != 3) {
		meansensors(mpu);
		if (fabs(mean_ax)<=acel_deadzone) ready++;
		else ax_offset=ax_offset-mean_ax/acel_deadzone;

		if (fabs(mean_ay)<=acel_deadzone) ready++;
		else ay_offset=ay_offset-mean_ay/acel_deadzone;

		if (fabs(16384-mean_az)<=acel_deadzone) ready++;
		else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
	}

	asm("NOP");
	mpu_set_offset_acc_x(ax_offset);
	mpu_set_offset_acc_y(ay_offset);
	mpu_set_offset_acc_z(az_offset);
	return TRUE;
}

uint8_t mpu_set_acc_res(int16_t res)
{
	return mpu_write_word(ACCEL_XOUT_H, res);
}

uint8_t mpu_set_gyr_res(int16_t res)
{
	return mpu_write_word(GYRO_CONFIG, res);
}
uint8_t mpu_set_offset_acc_x(int16_t offset)
{
	return mpu_write_word(XA_OFFS_H, offset);
}

uint8_t mpu_set_offset_acc_y(int16_t offset)
{
	return mpu_write_word(YA_OFFS_H, offset);
}

uint8_t mpu_set_offset_acc_z(int16_t offset)
{
	return mpu_write_word(ZA_OFFS_H, offset);
}

uint8_t mpu_set_offset_gyr_x(int16_t offset)
{
	return mpu_write_word(XG_OFFS_USRH, offset);
}

uint8_t mpu_set_offset_gyr_y(int16_t offset)
{
	return mpu_write_word(YG_OFFS_USRH, offset);
}

uint8_t mpu_set_offset_gyr_z(int16_t offset)
{
	return mpu_write_word(ZG_OFFS_USRH, offset);
}

uint8_t mpu_read_all(mpu_t * data) 
{
	uint8_t buff[14];
	buff[0] = ACCEL_XOUT_H;
	
	if(i2c_transmit(MPU_ADDR, buff, 1) != TRUE) {
		return FALSE;
	}
	
	if(i2c_receive(MPU_ADDR, buff, 14) != TRUE) {
		return FALSE;
	}
	
	data->raw_acc_x = (int16_t) (buff[0] << 8 | buff[1]);
	data->raw_acc_y = (int16_t) (buff[2] << 8 | buff[3]);
	data->raw_acc_z = (int16_t) (buff[4] << 8 | buff[5]);
	
	data->raw_gyr_x = (int16_t) (buff[8] << 8 | buff[9]);
	data->raw_gyr_y = (int16_t) (buff[10] << 8 | buff[11]);
	data->raw_gyr_z = (int16_t) (buff[12] << 8 | buff[13]);
	
	float ay, az;
	
	ay = (float) data->raw_acc_y / data->acc_res;
	az = (float) data->raw_acc_z / data->acc_res;
	
	data->calc_roll = (int16_t) ((float) atan2f(ay, az) * 180.0 / M_PI);
	
	return TRUE;
}

static uint8_t mpu_write_word(uint8_t reg, int16_t word)
{
	uint8_t buff[3];
	buff[0] = reg;
	buff[1] = word >> 8;
	buff[2] = word;
	if(i2c_transmit(MPU_ADDR, buff, 3) != TRUE) {
		return FALSE;
	}
	return TRUE;
}
