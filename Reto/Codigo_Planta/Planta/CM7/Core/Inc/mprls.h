#include "stm32h7xx_hal.h"

void MPRLS_status(I2C_HandleTypeDef *hi2c);

void MPRLS_init(I2C_HandleTypeDef *hi2c);

float MPRLS_read(I2C_HandleTypeDef *hi2c);
