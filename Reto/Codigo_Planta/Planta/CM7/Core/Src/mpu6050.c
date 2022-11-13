#include "stm32h7xx_hal.h"
#include "mpu6050.h"

// Direcciones del MPU6050
#define MPU6050_ADDR 0xD0
#define WHO_AM_I 0x75
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

// Lectura del MPU6050
uint8_t check, data;
float gyrF[3];

void MPU6050_status(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef status;
	status = HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 1, 3000);
	if (status == HAL_OK) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
}

uint8_t MPU6050_init(I2C_HandleTypeDef *hi2c){
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 3000);
	HAL_Delay(1000);

	data = 0x00;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 3000);
	data = 0b10000011;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV, 1, &data, 1, 3000);
	data = 0x00;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, 3000);

	return check;
}

float *MPU6050_read_gyro(I2C_HandleTypeDef *hi2c){
	uint8_t read_gyro[6];

	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H, 1, read_gyro, 6, 3000);

	gyrF[0] = ((int16_t)(read_gyro[0] << 8 | read_gyro[1]))/131.0;
	gyrF[1] = ((int16_t)(read_gyro[2] << 8 | read_gyro[3]))/131.0;
	gyrF[2] = ((int16_t)(read_gyro[4] << 8 | read_gyro[5]))/131.0;

	return gyrF;
}
