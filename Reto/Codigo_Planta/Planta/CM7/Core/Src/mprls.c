#include "stm32h7xx_hal.h"
#include "mprls.h"

#define MPRLS_ADDR 0x18

float press_counts = 0; // digital pressure reading [counts]
float outputmax = 0; // output at maximum pressure [counts]
float outputmin = 0; // output at minimum pressure [counts]
float pmax = 25; // maximum value of pressure range [bar, psi, kPa, etc.]
float pmin = 0; // minimum value of pressure range [bar, psi, kPa, etc.
uint8_t cmd[3] = {0xAA, 0x00, 0x00}; // Comando a enviar

uint8_t press_read[4];
float press = 0, press_counts;

void MPRLS_status(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef status;
	status = HAL_I2C_IsDeviceReady(hi2c, 0x30, 3, 3000);
	if (status == HAL_OK) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, SET);
}

void MPRLS_init(I2C_HandleTypeDef *hi2c){
	HAL_I2C_Master_Transmit(hi2c, 0x30, cmd, 3, 3000);
}

float MPRLS_read(I2C_HandleTypeDef *hi2c){
	/*HAL_I2C_Master_Transmit(hi2c, 0x30, cmd, 3, 3000);
	HAL_I2C_Master_Receive(hi2c, 0x31, press_read, 4, 3000);
	press_counts = press_read[3] + press_read[2] * 256 + press_read[1] * 65536.0;
	press = ((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
*/
	outputmin = (uint32_t)((float)16777216L * (0 / 100.0) + 0.5);
	outputmax = (uint32_t)((float)16777216L * (90/ 100.0) + 0.5);
	HAL_I2C_Master_Transmit(hi2c, 0x30, cmd, 3, 3000);
	HAL_I2C_Master_Receive(hi2c, 0x31, press_read, 4, 3000);
	uint32_t psi1 = ((uint32_t)(press_read[1]) << 16) | ((uint32_t)(press_read[2]) << 8) | ((uint32_t)(press_read[3]));
	float psi = (psi1 - outputmin) * (pmax - pmin);
	psi /= (float)(outputmax - outputmin);
	psi += pmin;
	return psi;
}
