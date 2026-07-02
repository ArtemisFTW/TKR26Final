/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include<stdbool.h>
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_byte;
volatile uint8_t rx_count=0;
volatile bool ready= false;
volatile uint8_t idx=0;
uint8_t packet_buffer[11];

//data
int8_t LX;
int8_t LY;
int8_t RX;
int8_t RY;
uint8_t L2,R2;
uint16_t buttons;
uint16_t prev_buttons;

#define EVENT_QUEUE_SIZE 8
volatile uint16_t event_queue[8];
volatile uint8_t event_head =0;
volatile uint8_t event_tail =0;
volatile uint16_t prev_buttons_isr = 0;

#define NUM_BUTTONS 10
volatile uint8_t butt[NUM_BUTTONS] = {0};

static inline void enqueue_button_event(uint16_t pressed)
{
	uint8_t next = (event_head + 1) % EVENT_QUEUE_SIZE;

	if (next != event_tail) {   // queue not full
		event_queue[event_head] = pressed;
		event_head = next;
	}
}

bool check_packet(uint8_t *buf){
	if(buf[0]!=0xAA || buf[10]!=0x55) return false;
	uint8_t sum=0;
	for(int i=0;i<=8;i++) sum+=buf[i];
	return ((uint8_t)(0-sum)==buf[9]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		if (idx == 0)
		{
			if (rx_byte == 0xAA)
			{
				packet_buffer[idx++] = rx_byte;
			}
		}
		else
		{
			packet_buffer[idx++] = rx_byte;

			if (idx == 11)
			{
				if (check_packet(packet_buffer))
				{
					ready=true;
//					LX = (int8_t)packet_buffer[1];
//					LY = (int8_t)packet_buffer[2];
//					RX = (int8_t)packet_buffer[3];
//					RY = (int8_t)packet_buffer[4];
//					R2 = packet_buffer[5];
//					L2 = packet_buffer[6];

					uint16_t buttons =
							packet_buffer[7] |
							(packet_buffer[8] << 8);

					uint16_t pressed = buttons & ~prev_buttons;

					if (pressed)
						enqueue_button_event(pressed);

					prev_buttons = buttons;

					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				}

				idx = 0;
			}
		}

		HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
	}
}

void process_packet(void){
	if(ready){
		uint8_t local_buf[11];
		__disable_irq();
		memcpy(local_buf, packet_buffer, 11);
		ready = false;
		__enable_irq();

		if(check_packet(local_buf)){
//			LX = (int8_t)local_buf[1];
//			LY = (int8_t)local_buf[2];
//			RX = (int8_t)local_buf[3];
//			RY = (int8_t)local_buf[4];
			LX = ((int16_t)local_buf[1] - 50) * 2;
			LY = ((int16_t)local_buf[2] - 50) * 2;
			RX = ((int16_t)local_buf[3] - 50) * 2;
			RY = ((int16_t)local_buf[4] - 50) * 2;
			R2 = local_buf[5];
			L2 = local_buf[6];
		}
	}
}

void process_button_events(void)
{
	while (event_tail != event_head) {

		uint16_t evt = event_queue[event_tail];
		event_tail = (event_tail + 1) % EVENT_QUEUE_SIZE;

		if(evt & (1 << 0)) printf("Cross\r\n");
		if(evt & (1 << 1)) printf("Circle\r\n");
		if(evt & (1 << 2)) printf("Triangle\r\n");
		if(evt & (1 << 3)) printf("Square\r\n");
		if(evt & (1 << 4)) printf("L1\r\n");
		if(evt & (1 << 5)) printf("R1\r\n");
		if(evt & (1 << 6)) printf("Up\r\n");
		if(evt & (1 << 7)) printf("Down\r\n");
		if(evt & (1 << 8)) printf("Left\r\n");
		if(evt & (1 << 9)) printf("Right\r\n");
	}
}

void print_pots(void)
{
	printf("LX:%d  LY:%d  RX:%d  RY:%d  L2:%d  R2:%d\r\n", LX, LY, RX, RY, L2, R2);
}

// Send printf to uart2
int _write(int fd, char* ptr, int len) {
	HAL_StatusTypeDef hstatus;

	if (fd == 1 || fd == 2) {
		hstatus = HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
		if (hstatus == HAL_OK)
			return len;
		else
			return -1;
	}
	return -1;
}
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
	MX_UART4_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("STM32 ready\r\n");

	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		process_packet();
		process_button_events();
		print_pots();
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
