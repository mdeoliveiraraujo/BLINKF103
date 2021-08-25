/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "ads1256.h"
#include "file_handling.h"
#include "UartRingbuffer.h"
#include "stm32f1xx_it.h"
#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FILE_BASE_NAME "ESQ"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

FATFS fs;
FIL fil;
FRESULT fresult;
//char buffer[256];
char *exchange_buffer1[1];
int conversion_trigger = 0;
int line_counter = 0;
char uart_buffer[64];
uint32_t adc_return1;
uint32_t adc_return2;
uint32_t adc_return3;
uint32_t adc_return4;
uint32_t adc_return5;
uint32_t adc_return6;
uint32_t adc_return7;
uint32_t adc_return8;

extern float angleAccX, angleAccY;
extern float angleX, angleY, angleZ;
extern float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern float acc_total_vector, angle_pitch_acc, angle_roll_acc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "00.txt";

char cmd = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_FATFS_Init();
	MX_SPI2_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, 1);
	Ringbuf_init();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	HAL_Delay(1000);
	send_uart("Inicializando lado direito. Por favor, aguarde...\n");


	initADS();

	send_uart("Montando SD...\n");

	mount_sd();

	create_file_mod(BASE_NAME_SIZE, fileName);

	//send_uart("Inicializando IMU...\n");
	//MPU6050_Init();
	//MPU6050_Retrieve_Data();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// ------------------------------------
		// Testing the ADS1256 functions
		// ------------------------------------
		//send_uart("Faaaaaaaaala Nigeeeeeel\n");
		//MPU6050_Update();
		//sprintf(uart_buffer,"%0.2f\t%0.2f\t%0.2f\t%0.2f\n",(float)angleAccX,(float)angleAccY,(float)angleX,(float)angleY);
		//send_uart(uart_buffer);
		//HAL_Delay(50);

		if (conversion_trigger == 1) {
			const uint32_t timezinho = GetMicros();
			adc_return1 = read_Value(0x68);
			adc_return2 = read_Value(0x58);
			adc_return3 = read_Value(0x78);
			sprintf(buffer, "%lu\t%lu\t%lu\t%lu\n", (unsigned long) timezinho,
					(unsigned long) adc_return1, (unsigned long) adc_return2,
					(unsigned long) adc_return3);
			const uint8_t buflen = strlen(buffer);
			memccpy(&buffer_SD[line_counter * buflen], buffer, 0, buflen);
			line_counter++;
			if (line_counter * buflen > BUFFER_SIZE * ROW_SIZE - buflen) {
				write_file_mod(fileName, buffer_SD);
				line_counter = 0;
				clear_buffer_SD();
			}
		}

		if (conversion_trigger == 2) {
			const uint32_t timezinho = GetMicros();
			adc_return1 = read_Value(0x68);
			adc_return2 = read_Value(0x58);
			adc_return4 = read_Value(0x48);
			adc_return5 = read_Value(0x38);
			adc_return6 = read_Value(0x28);
			adc_return7 = read_Value(0x18);
			adc_return8 = read_Value(0x08);
			adc_return3 = read_Value(0x78);
			sprintf(buffer, "%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\n",
					(unsigned long) timezinho, (unsigned long) adc_return1,
					(unsigned long) adc_return2, (unsigned long) adc_return4,
					(unsigned long) adc_return5, (unsigned long) adc_return6,
					(unsigned long) adc_return7, (unsigned long) adc_return8,
					(unsigned long) adc_return3);
			const uint8_t buflen = strlen(buffer);
			memccpy(&buffer_SD[line_counter * buflen], buffer, 0, buflen);
			line_counter++;
			if (line_counter * buflen > BUFFER_SIZE * ROW_SIZE - buflen) {
				write_file_mod(fileName, buffer_SD);
				line_counter = 0;
				clear_buffer_SD();
			}
		}

		if (conversion_trigger == 3) {
			const uint32_t timezinho = GetMicros();
			adc_return1 = read_Value(0x68);
			adc_return2 = read_Value(0x58);
			adc_return3 = read_Value(0x78);
			sprintf(buffer, "%lu\t%lu\t%lu\t%lu\n", (unsigned long) timezinho,
					(unsigned long) adc_return1, (unsigned long) adc_return2,
					(unsigned long) adc_return3);
			const uint8_t buflen = strlen(buffer);
			send_uart(buffer);
			line_counter++;
			if (line_counter * buflen > BUFFER_SIZE * ROW_SIZE - buflen) {
				line_counter = 0;
			}
		}

		if (conversion_trigger == 4) {
			const uint32_t timezinho = GetMicros();
			adc_return1 = read_Value(0x68);
			adc_return2 = read_Value(0x58);
			adc_return4 = read_Value(0x48);
			adc_return5 = read_Value(0x38);
			adc_return6 = read_Value(0x28);
			adc_return7 = read_Value(0x18);
			adc_return8 = read_Value(0x08);
			adc_return3 = read_Value(0x78);
			sprintf(buffer, "%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\n",
					(unsigned long) timezinho, (unsigned long) adc_return1,
					(unsigned long) adc_return2,
					(unsigned long) adc_return4, (unsigned long) adc_return5,
					(unsigned long) adc_return6, (unsigned long) adc_return7,
					(unsigned long) adc_return8, (unsigned long) adc_return3);
			const uint8_t buflen = strlen(buffer);
			send_uart(buffer);
			line_counter++;
			if (line_counter * buflen > BUFFER_SIZE * ROW_SIZE - buflen) {
				line_counter = 0;
			}
		}
		// ------------------------------------
		// Strip of the code that treats the SD
		// ------------------------------------
		while (IsDataAvailable()) {
			//		  buffer = Uart_read();
			Get_string(buffer);
			uint8_t len = cmdlength(buffer);

			//		  buffer[0]='l';
			//		  buffer[1]='s';
			get_path();
			//
			if (!(strncmp("ls", buffer, len)))
				cmd = 'l';

			if (!(strncmp("mkdir", buffer, len)))
				cmd = 'm';

			if (!(strncmp("mkfil", buffer, len)))
				cmd = 'c';

			if (!(strncmp("read", buffer, len)))
				cmd = 'r';

			if (!(strncmp("write", buffer, len)))
				cmd = 'w';

			if (!(strncmp("rm", buffer, len)))
				cmd = 'd';

			if (!(strncmp("update", buffer, len)))
				cmd = 'u';

			if (!(strncmp("checkfile", buffer, len)))
				cmd = 'f';

			if (!(strncmp("checksd", buffer, len)))
				cmd = 's';

			if (!(strncmp("sd3", buffer, len)))
				cmd = 'x';

			if (!(strncmp("serial3", buffer, len)))
				cmd = 'b';

			if (!(strncmp("sd8", buffer, len)))
				cmd = 'a';

			if (!(strncmp("serial8", buffer, len)))
				cmd = 'e';

			if (!(strncmp("end", buffer, len)))
				cmd = 'y';
			//sprintf (buffer, "BUFFER SIZE: \t%d\n",len);
			//send_uart(buffer);

			switch (cmd) {
			case ('x'):
										//send_uart("ADS1256 started.");
										HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
			//				create_file_mod(BASE_NAME_SIZE, fileName);
			delayMicroseconds(500);
			cmd = 0;
			conversion_trigger = 1;
			clear_buffer();
			clear_path();
			UptimeMillis = 0;
			break;

			case ('a'):
										//send_uart("ADS1256 started.");
										HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
			//				create_file_mod(BASE_NAME_SIZE, fileName);
			delayMicroseconds(500);
			cmd = 0;
			conversion_trigger = 2;
			clear_buffer();
			clear_path();
			UptimeMillis = 0;
			break;

			case ('b'):
										//send_uart("ADS1256 started.");
										HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
			//				create_file_mod(BASE_NAME_SIZE, fileName);
			delayMicroseconds(500);
			cmd = 0;
			conversion_trigger = 3;
			clear_buffer();
			clear_path();
			UptimeMillis = 0;
			break;

			case ('e'):
										//send_uart("ADS1256 started.");
										HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
			//				create_file_mod(BASE_NAME_SIZE, fileName);
			delayMicroseconds(500);
			cmd = 0;
			conversion_trigger = 4;
			clear_buffer();
			clear_path();
			UptimeMillis = 0;
			break;

			case ('y'):
										//send_uart("ADS1256 halted.");
										HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
			cmd = 0;
			conversion_trigger = 0;
			write_file_mod(fileName, buffer_SD);
			close_file(fileName);
			clear_buffer_SD();
			clear_buffer();
			clear_path();
			create_file_mod(BASE_NAME_SIZE, fileName);
			break;

			case ('l'):
										scan_files(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('m'):
										create_dir(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('c'):
										create_file(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('r'):
										read_file(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('w'):
										write_file(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('d'):
										remove_file(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('u'):
										update_file(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('f'):
										check_file(path);
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			case ('s'):
										check_sd();
			cmd = 0;
			clear_buffer();
			clear_path();
			break;

			default:
				clear_buffer();
				clear_path();
				break;
			}
		}
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

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
	htim2.Init.Prescaler = 36-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535-1;
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SPI1_NCS_Pin|SPI2_NCS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI1_SYNC_Pin|SPI1_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_NCS_Pin SPI2_NCS_Pin */
	GPIO_InitStruct.Pin = SPI1_NCS_Pin|SPI2_NCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_RDY_Pin */
	GPIO_InitStruct.Pin = SPI1_RDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SPI1_RDY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_SYNC_Pin SPI1_RST_Pin */
	GPIO_InitStruct.Pin = SPI1_SYNC_Pin|SPI1_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void delayMicroseconds(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;  // wait for the counter to reach the us input in the parameter
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
