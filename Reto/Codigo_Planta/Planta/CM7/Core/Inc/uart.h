#include "stm32h7xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <complex.h>

typedef float complex cplx;

void print_int(UART_HandleTypeDef *huart, char *string, int var);

void print_float(UART_HandleTypeDef *huart, char *string, float var);

void print_clpx(UART_HandleTypeDef *huart, char *string, cplx var);

void print_arrayInt(UART_HandleTypeDef *huart, int *arr, uint16_t size);

void print_arrayFloat(UART_HandleTypeDef *huart, float *arr, uint16_t size);

void print_arrayCplx(UART_HandleTypeDef *huart, cplx *arr, uint16_t size);

void send_data(UART_HandleTypeDef *huart, uint8_t *arr, uint8_t size);
