#include "stm32h7xx_hal.h"
#include "uart.h"

#include <stdio.h>
#include <string.h>
#include <complex.h>

typedef float complex cplx;
char uart_buf[50];
uint16_t uart_buf_len;

void print_int(UART_HandleTypeDef *huart, char *string, int var){
	uart_buf_len = sprintf(uart_buf, string, var);
	HAL_UART_Transmit(huart, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void print_float(UART_HandleTypeDef *huart, char *string, float var){
	uart_buf_len = sprintf(uart_buf, string, var);
	HAL_UART_Transmit(huart, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void print_cplx(UART_HandleTypeDef *huart, char *string, cplx var){
	uart_buf_len = sprintf(uart_buf, "%s%.5f + %.5fj", string, creal(var), cimag(var));
	HAL_UART_Transmit(huart, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void print_arrayInt(UART_HandleTypeDef *huart, int *arr, uint16_t size){
	for(int i = 0; i < size; i++) print_int(huart, "%u, ", arr[i]);
	print_int(huart, "\r\n", 0);
}

void print_arrayFloat(UART_HandleTypeDef *huart, float *arr, uint16_t size){
	for(int i = 0; i < size; i++) print_float(huart, "%.5f, ", arr[i]);
	print_int(huart, "\r\n", 0);
}

void print_arrayCplx(UART_HandleTypeDef *huart, cplx *arr, uint16_t size){
	for(int i = 0; i < size; i++) print_cplx(huart, "", arr[i]);
	print_int(huart, "\r\n", 0);
}

void send_data(UART_HandleTypeDef *huart, uint8_t *arr, uint8_t size){
	HAL_UART_Transmit(huart, (uint8_t *)arr, size, 100);
}
