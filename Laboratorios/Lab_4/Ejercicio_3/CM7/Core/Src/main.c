/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050_ADDR 0xD0
#define WHO_AM_I 0x75
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Duty Cycle del motor
int16_t dutyCycle = 40;

// Lectura del MPU6050
uint8_t check, data;

int16_t acc_X_read = 0;
int16_t acc_Y_read = 0;
int16_t acc_Z_read = 0;
int16_t gyr_X_read = 0;
int16_t gyr_Y_read = 0;
int16_t gyr_Z_read = 0;

float acc_X, acc_Y, acc_Z, gyr_X, gyr_Y, gyr_Z;

// Variables para el muestreo
int16_t limite_muestras = 32;
int16_t n_muestras = 0;
float muestras40_x[32],  muestras40_y[32], muestras40_z[32];
float muestras60_x[32],  muestras60_y[32], muestras60_z[32];
float muestras80_x[32],  muestras80_y[32], muestras80_z[32];


HAL_StatusTypeDef status;

// Variables UART
char uart_buf[50];
uint16_t uart_buf_len;

// Variables de conprobaci??n de tiempo
unsigned timer_val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_init(void);
void MPU6050_read_gyro(void);
void changeCycle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Incializaci??n del timer 5
  HAL_TIM_Base_Start(&htim5);

  // Inicializaci??n del PWM
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, RESET);
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  TIM1->CCR1 = 100;

  // Revisi??n de conexi??n del MPU6050
  HAL_StatusTypeDef status;
  status = HAL_I2C_IsDeviceReady(&hi2c4, MPU6050_ADDR, 1, 3000);
  if (status == HAL_OK) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
  else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
  HAL_Delay(500);

  // Prueba de comunicaci??n UART
  uart_buf_len = sprintf(uart_buf, "MPU6050 test\r\n");
  HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);

  // Inicializaci??n del MPU6050
  MPU6050_init();

  // Inicializaci??n de la interrupci??n por timer
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 31249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 240;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MPU6050_init(){
	HAL_I2C_Mem_Read(&hi2c4, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 3000);
	uart_buf_len = sprintf(uart_buf, "Direcci??n: %u \r\n", check);
	HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
	HAL_Delay(1000);

	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 3000);
	data = 0b10000011;
	HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, SMPLRT_DIV, 1, &data, 1, 3000);
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, 3000);
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c4, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, 3000);

}

void MPU6050_read_gyro(void){
	uint8_t read_gyro[6];

	HAL_I2C_Mem_Read(&hi2c4, MPU6050_ADDR, GYRO_XOUT_H, 1, read_gyro, 6, 3000);

	gyr_X_read = (int16_t)(read_gyro[0] << 8 | read_gyro[1]);
	gyr_Y_read = (int16_t)(read_gyro[2] << 8 | read_gyro[3]);
	gyr_Z_read = (int16_t)(read_gyro[4] << 8 | read_gyro[5]);

	gyr_X = gyr_X_read/131.0;
	gyr_Y = gyr_Y_read/131.0;
	gyr_Z = gyr_Z_read/131.0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim2)
	{
		if(n_muestras == 0) timer_val = __HAL_TIM_GET_COUNTER(&htim5);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		MPU6050_read_gyro();

		uart_buf_len = sprintf(uart_buf, "\r\n Muestra: %u \r\n", n_muestras);
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		uart_buf_len = sprintf(uart_buf, "GyrX = %.5f ??/s \r\n", gyr_X);
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		uart_buf_len = sprintf(uart_buf, "GyrY = %.5f ??/s \r\n", gyr_Y);
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		uart_buf_len = sprintf(uart_buf, "GyrZ = %.5f ??/s \r\n\n", gyr_Z);
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);

		// Guardado de muestras en los arreglos
		if(dutyCycle == 40){
			muestras40_x[n_muestras] = gyr_X;
			muestras40_y[n_muestras] = gyr_Y;
			muestras40_z[n_muestras] = gyr_Z;
		} else if(dutyCycle == 60){
			muestras60_x[n_muestras] = gyr_X;
			muestras60_y[n_muestras] = gyr_Y;
			muestras60_z[n_muestras] = gyr_Z;
		} else if(dutyCycle == 80){
			muestras80_x[n_muestras] = gyr_X;
			muestras80_y[n_muestras] = gyr_Y;
			muestras80_z[n_muestras] = gyr_Z;
		}

		n_muestras++;
		if(n_muestras == limite_muestras){
			uart_buf_len = sprintf(uart_buf, "Muestras: %u \r\n", n_muestras);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
			HAL_TIM_Base_Stop_IT(&htim2);

			// Comprobaci??n del tiempo de muestreo
			timer_val = __HAL_TIM_GET_COUNTER(&htim5) - timer_val;
			uart_buf_len = sprintf(uart_buf, "Tiempo transcurrido: %u us\r\n", timer_val);
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);

			changeCycle();
		}
	}
}

void changeCycle(void){
	TIM1->CCR1 = 0;
	n_muestras = 0;

	if(dutyCycle == 40){
		dutyCycle = 60;
		TIM1->CCR1 = 80;

		uart_buf_len = sprintf(uart_buf, "DUTY CYCLE: 40%");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		uart_buf_len = sprintf(uart_buf, "\r\n Eje X: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras40_x[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		uart_buf_len = sprintf(uart_buf, "\r\n Eje Y: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras40_y[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		uart_buf_len = sprintf(uart_buf, "\r\n Eje Z: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras40_z[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}

		HAL_TIM_Base_Start_IT(&htim2);
	} else if(dutyCycle == 60){
		dutyCycle = 80;
		TIM1->CCR1 = 100;

		uart_buf_len = sprintf(uart_buf, "DUTY CYCLE: 60%");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		uart_buf_len = sprintf(uart_buf, "\r\n Eje X: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras60_x[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		uart_buf_len = sprintf(uart_buf, "\r\n Eje Y: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras60_y[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		uart_buf_len = sprintf(uart_buf, "\r\n Eje Z: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras60_z[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}

		HAL_TIM_Base_Start_IT(&htim2);
	} else if(dutyCycle == 80){
		uart_buf_len = sprintf(uart_buf, "DUTY CYCLE: 80%");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		uart_buf_len = sprintf(uart_buf, "\r\n Eje X: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras80_x[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		uart_buf_len = sprintf(uart_buf, "\r\n Eje Y: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras80_y[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		uart_buf_len = sprintf(uart_buf, "\r\n Eje Z: ");
		HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		for (int i = 0; i < limite_muestras; i++){
			uart_buf_len = sprintf(uart_buf, "%.2f, ", muestras80_z[i]);
			HAL_UART_Transmit(&huart3, &uart_buf, uart_buf_len, 100);
		}
		TIM1->CCR1 = 0;
		dutyCycle = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
