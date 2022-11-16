#include "stm32h7xx_hal.h"

void MPU6050_status(I2C_HandleTypeDef *hi2c);

uint8_t MPU6050_init(I2C_HandleTypeDef *hi2c);

float *MPU6050_read_gyro(I2C_HandleTypeDef *hi2c);
