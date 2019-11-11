#ifndef _MPU6050_DEFINED_H_
#define _MPU6050_DEFINED_H_

#include <stdint.h>

#define MPU_ADDR                0x68

/* Offsets do aceler�metro e girosc�pio */
#define XA_OFFS_H               0x06
#define YA_OFFS_H               0x08
#define ZA_OFFS_H               0x0A
#define XG_OFFS_USRH            0x13
#define YG_OFFS_USRH            0x15
#define ZG_OFFS_USRH            0x17

#define GYRO_CONFIG             0x1B
#define ACCEL_CONFIG            0x1C
#define WHO_I_AM                0x75
#define ACCEL_XOUT_H            0x3B
#define PWR_MGMT_1              0x6B

/* Gyro sensitivities in degreee/s */
#define GYR_SENS_250            (float) 131
#define GYR_SENS_500            (float) 65.5
#define GYR_SENS_1000           (float) 32.8
#define GYR_SENS_2000           (float) 16.4

/* Accel sensitivities in g/s */
#define ACC_SENS_2              (float) 16384
#define ACC_SENS_4              (float) 8192
#define ACC_SENS_8              (float) 4096
#define ACC_SENS_16             (float) 2048


#define ACC_OFF_X                       -1655
#define ACC_OFF_Y                       -810
#define ACC_OFF_Z                       1547
#define GYR_OFF_X                       -18
#define GYR_OFF_Y                       -15
#define GYR_OFF_Z                       16

#define COEF_FILTER_PREV                0.95
#define COEF_FILTER_NEXT                0.05

typedef struct _mpu_t_{
	int16_t acc_res;
	int16_t gyr_res;
	
	int16_t raw_acc_x;
	int16_t raw_acc_y;
	int16_t raw_acc_z;
	
	int16_t raw_gyr_x;
	int16_t raw_gyr_y;
	int16_t raw_gyr_z;
	
	int16_t calc_roll;
} mpu_t;

uint8_t mpu_read_all(mpu_t * data);
uint8_t mpu_init(mpu_t * data);
uint8_t mpu_calibration(mpu_t * mpu);
uint8_t mpu_set_acc_res(int16_t res);
uint8_t mpu_set_gyr_res(int16_t res);
uint8_t mpu_set_offset_acc_x(int16_t offset);
uint8_t mpu_set_offset_acc_y(int16_t offset);
uint8_t mpu_set_offset_acc_z(int16_t offset);
uint8_t mpu_set_offset_gyr_x(int16_t offset);
uint8_t mpu_set_offset_gyr_y(int16_t offset);
uint8_t mpu_set_offset_gyr_z(int16_t offset);
#endif
