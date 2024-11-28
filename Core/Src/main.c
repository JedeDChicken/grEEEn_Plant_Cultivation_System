/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include <ESP8266_Code.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float temp, humid;
float buf_data[6];
//HAL_StatusTypeDef status;
//#define LED_ONOFF(); GPIOC->ODR ^= GPIO_PIN_13;
#define AM2320_ADDRESS	0xB8
unsigned int tick;
volatile unsigned int auto_recon=60000;

char *am_pm;
float tempF;
float direct_fan=0.0;					//0- off, 1- direct, 2- oscillating
float ceiling_fan=0.0;					//0- off, 1- CW (Circulate warm air), 2- CC (Pushes cool air down)
float humidity_tray=0.0;				//0- off, 1- drain water, 2- pump water
float moisture_absorber_tray=0.0;		//0- close, 1- open; silica gel?, rock salt?, calcium chloride?
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Notes
/*
STM32F4 AM2320 No HAL (Hardware Abstraction Layer)
1. Configure GPIO Pins- connect AM2320 pins
2. I2C Init (Config)- speed, addressing mode, ack...
3. I2C Start Condition-
	-pull SDA low, SCL high
4. Send I2C Device Address- of AM2320, write bit- 0
	-0xB8 for write, followed by write bit, wait for ack?
5. Send I2C Register Address- from where you want to read the data (e.g. temp and humid... datasheet)
	-0x03, wait for ack
6. I2C Stop-
7. Delay- datasheet
8. I2C Start-
9. I2C Device Address- 1- read bit
	-0xB9 for read, read bit
10. Read Data- datasheet- register structure and data format
	-Receive data bytes from sensor
	-Generate ACK- more bytes are expected
	-NACK- last byte
	-Repeat ACK/NACK for each byte
11. I2C Stop-
	-by releasing SDA line while SCL is high
12. Process Data- accd. sensor's data format and ur application
*/

/*
AM2320 Specs
*5V, 4.7k for SCL and SDA, 5V-SDA-GND-SCL
*I2C bus mode, 2s per read, max 3s total comms
*1byte, function codes 1-127, output- 16bits (bit15 is sign)
*/

/*
I2C_Config
1. Enable the I2C CLOCK and GPIO CLOCK
2. Configure the I2C PINs for ALternate Functions
	a) Select Alternate Function in MODER Register
	b) Select Open Drain Output
	c) Select High SPEED for the PINs
	d) Select Pull-up for both the Pins
	e) Configure the Alternate Function in AFR Register
3. Reset the I2C
4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
5. Configure the clock control registers
6. Configure the rise time register
7. Program the I2C_CR1 register to enable the peripheral
*/


//Inits
void GPIO_Config(void) {
	RCC->AHB1ENR |= (1<<1);			//GPIOB Clock Enable

	GPIOB->MODER &= ~(1<<16);		//Alternate Function Mode- 10
	GPIOB->MODER |= (1<<17);
	GPIOB->MODER &= ~(1<<18);
	GPIOB->MODER |= (1<<19);

	GPIOB->OTYPER |= (1<<8);		//Output Open-drain- 1
	GPIOB->OTYPER |= (1<<9);

	GPIOB->OSPEEDR |= (1<<16);		//High Speed- 11, 50Mhz?
	GPIOB->OSPEEDR |= (1<<17);
	GPIOB->OSPEEDR |= (1<<18);
	GPIOB->OSPEEDR |= (1<<19);

	GPIOB->PUPDR &= ~(1<<16);		//Pull-up- 01, No Pull- 00?
	GPIOB->PUPDR &= ~(1<<17);
	GPIOB->PUPDR &= ~(1<<18);
	GPIOB->PUPDR &= ~(1<<19);

	GPIOB->AFR[1] &= ~(1<<0);		//AF6- 0110
	GPIOB->AFR[1] &= ~(1<<1);		//Wrong ung una, AF4- 0100 (for I2C1)
	GPIOB->AFR[1] |= (1<<2);
	GPIOB->AFR[1] &= ~(1<<3);

	GPIOB->AFR[1] &= ~(1<<4);		//AF7- 0111
	GPIOB->AFR[1] &= ~(1<<5);
	GPIOB->AFR[1] |= (1<<6);
	GPIOB->AFR[1] &= ~(1<<7);
}

void I2C_Config(void) {
	RCC->APB1ENR |= (1<<21);		//I2C Clock Enable

	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
	I2C1->CR1 |= (1<<15);			//Reset- for errors or locked states
	I2C1->CR1 &= ~(1<<15);

	I2C1->CR2 |= (50<<0);			//50MHz, 100kHz; used 8MHz as APB1 PCLK; 8

	I2C1->CCR &= ~(1<<15);			//SM																																																																																																											+	`, PCLK- 8MHz? (>=2MHz for SM)
	I2C1->CCR &= ~(1<<14);			//Since not FM
	I2C1->CCR |= (250<<0);			//250, High, computation on CtrlsTech; 40

	I2C1->TRISE |= (51<<0);			//50+1, "; 9

	//Configure I2C1 addr mode, dual addr mode, gen call mode, own addr
	I2C1->CR1 &= ~(1<<0);			//Disable I2C1
	I2C1->OAR1 &= ~(1<<15);
	I2C1->OAR1 |= (1<<15);			//7-bit addr mode
	I2C1->OAR2 &= (1<<0);
	I2C1->CR1 &= ~(I2C_CR1_ENGC | I2C_CR1_NOSTRETCH);

//	I2C1->OAR1 &= ~(I2C_OAR1_ADDMODE | I2C_OAR1_OA1EN);
//	I2C1->OAR1 |= I2C_OAR1_ADDMODE_7BIT;
//	I2C1->OAR2 &= ~I2C_OAR2_OA2EN;
//	I2C1->CR1 &= ~(I2C_CR1_GCEN | I2C_CR1_NOSTETCH);

	I2C1->CR1 |= (1<<10);			//ACK Enable
	I2C1->CR1 |= (1<<0);			//Peripheral Enable
}

//Functions
void I2C_Start(void) {
	I2C1->CR1 |= (1<<8);				//Start- repeated
	while (!(I2C1->SR1 & (1<<0)));		//Wait for SB bit to set (1)
}

void I2C_Stop(void) {
	I2C1->CR1 |= (1<<9);				//Stop I2C
}

void I2C_Write(uint8_t data) {
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<<7)));		//Wait for TXE but to set, BTF?
}

void I2C_Wake(void) {
	I2C_Start();

	(void) I2C1->SR1;
	I2C1->DR = AM2320_ADDRESS;
	HAL_Delay(1);


	I2C_Stop();
}

void start_sequence(uint8_t dir) {
	I2C_Start();

	(void) I2C1->SR1;								//Addr
	I2C1->DR = dir == 0? AM2320_ADDRESS : (AM2320_ADDRESS + 1);

	while (!(I2C1->SR1 & (1<<1)));					//Wait for ADDR bit to set
	(void) I2C1->SR1;								//For SB?
	(void) I2C1->SR2;								//For ADDR?
}

unsigned int crc16(uint8_t *ptr, uint8_t len) {		//Based on datasheet
	unsigned int crc = 0xFFFF;
	uint8_t i;

	while(len--) {
		crc ^= *ptr++;
		for(i=0; i<8; i++) {
			if(crc & 0x01) {
				crc >>= 1;
				crc ^= 0xA001;
			}
			else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

void AM2320_ReadData(void) {				//*t, *h
	uint8_t i;
	uint8_t buf[8];
	//uint8_t data_t[3];

	I2C_Wake();

	//Send Read Command
	start_sequence(0);						//TX
	I2C_Write(0x03);
	I2C_Write(0x00);
	I2C_Write(0x04);
	I2C_Stop();
	HAL_Delay(1);

	//Read Data
	start_sequence(1);						//RX
	for(i=0;i<8;i++) {
		while (!(I2C1->SR1 & (1<<6)));		//Wait for RXNE bit to set
		buf[i] = I2C1->DR;
	}
	I2C_Stop();

	//CRC Check
//	data_t[0] = 0x03;
//	data_t[1] = 0x00;
//	data_t[2] = 0x04;
//
//	HAL_I2C_IsDeviceReady(&hi2c1, 0xB8, 2, 1);
//	HAL_I2C_Master_Transmit(&hi2c1, 0xB8, data_t, 3, 1);
//	HAL_I2C_Master_Receive(&hi2c1, 0xB9, buf, 8, 2);

	unsigned int Rcrc = buf[7] << 8;
	Rcrc += buf[6];
	if (Rcrc == crc16(buf, 6)) {
		unsigned int temperature = ((buf[4] & 0x7F) << 8) + buf[5];
		temp = temperature / 10.0;
		temp = (((buf[4] & 0x80) >> 7) == 1) ? ((temp) * (-1)) : temp;
		unsigned int humidity = (buf[2] << 8) + buf[3];
		humid = humidity / 10.0;
		//return 0;
	}
	//return 2;
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
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  GPIO_Config();
  I2C_Config();
  ESP_Init("EEE192-320", "EEE192_Room320");			//"JEDEDCHICKEN", "ChickenZ"; "HABIBI", "HABIBIBI";
  I2C1->CR1 |= I2C_CR1_ACK;		//ACK Enable
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	auto_recon = HAL_GetTick() + 60000;

	//LED_ONOFF();
	AM2320_ReadData();

	//Virtual Actuators
	//Fan system- 1 direct fan, 1 ceiling fan
	//Golden Pothos / "Epipremnum aureum" Requirements- T=2 (65degF night, 75degF day), H=2 (25%-49%), W=2 (Soil surface must be dry before re-watering)
	//1hr sensor data gathering...
	//Time
	time_t current_time = time(NULL);
	struct tm *local_time = localtime(&current_time);
	int hour = local_time->tm_hour;
	am_pm = (hour < 12) ? "AM" : "PM";

	//C-F conversion
	tempF = ((9.0/5) * temp) + 32.0;

	//
	if (strcmp(am_pm, "AM") == 0) {		//If equal
		if (tempF < 75.0) {
			direct_fan=0.0;				//Off
			ceiling_fan=1.0;			//CW
		}
		else if (tempF > 75.0) {
			direct_fan=1.0;				//Direct
			ceiling_fan=2.0;			//CC
		}
	}
	else {
		if (tempF < 65.0) {
			direct_fan=0.0;
			ceiling_fan=1.0;
		}
		else if (tempF > 65.0) {
			direct_fan=1.0;
			ceiling_fan=2.0;
		}
	}

	//Water system
	if (humid < 25.0) {
		humidity_tray=2.0;				//Pump
		moisture_absorber_tray=0.0;		//Close
	}
	else if (humid > 49.0) {
		humidity_tray=1.0;				//Drain
		moisture_absorber_tray=1.0;		//Open
	}

	//
	buf_data[0] = temp;
	buf_data[1] = humid;
	buf_data[2] = direct_fan;
	buf_data[3] = ceiling_fan;
	buf_data[4] = humidity_tray;
	buf_data[5] = moisture_absorber_tray;
	ESP_Send_Multi("F7VIFLRMML37ATTC", 6, buf_data);
	HAL_Delay(15001);										//Must be >=15s delay
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

void SysTick_Handler(void)
{
	tick = HAL_GetTick();
	if (tick > auto_recon) { NVIC_SystemReset(); }

	HAL_IncTick();
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
