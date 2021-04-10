/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stm32f103xb.h"
#include "tm1637.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_I2C1_Init(void);
static void printMsg(char *msg, ...);
void init_hardware_timer_version(void);
//void init_gpio_interrupt(void);
//void delayMS(uint16_t ms);

#define DEBUG_UART	 		USART1
#define TIM_CNT_DEFAULT		0							// Es distinto de 0 para que no oscile genere un UIE por oscilaciones alrededor del punto de inicio
#define ANTIHORARIO 		(TIM3->CR1>>4 & 0x01) 		// Sentido antihorario es decreciente
#define TIM_CNT_MAX			0xFFFF						// Maximo valor del Timer Counter
#define AVANCE				68							// 68 pulsos por cada cm de avance
#define PULSOS_POR_REV		1440
#define PERIMETRO			21.19						// perimetro de la rueda en cm

int16_t circle_count = 0;
uint32_t medicion = 0;
uint8_t underflow = 0;
uint16_t overflow = 0;
uint32_t muestra = 0;

uint8_t	Timer1Enable = ENABLE;
uint32_t Timer1  = 0;
uint8_t CurrentDisplay[4] = {0};
uint8_t tm1637_Segments[8] = {0};

int main(void)
{
	//	tm1637_Segments[0] = A_SEG;
	//	tm1637_Segments[1] = B_SEG;
	//	tm1637_Segments[2] = C_SEG;
	//	tm1637_Segments[3] = D_SEG;
	//	tm1637_Segments[4] = E_SEG;
	//	tm1637_Segments[5] = F_SEG;
	//	tm1637_Segments[6] = G_SEG;
	//	tm1637_Segments[7] = DP_SEG;

	HAL_Init();
	SystemClock_Config();

	HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);		// TM1637 Display pins
	HAL_GPIO_WritePin(SDO_GPIO_Port, SDO_Pin, GPIO_PIN_SET);
	MX_USART1_UART_Init();
	init_hardware_timer_version();
//	init_gpio_interrupt();
	MX_GPIO_Init();

	//	MX_I2C1_Init();

	NVIC_EnableIRQ(TIM3_IRQn);

	Timer1Enable = ENABLE;										//Turn on systick based timer



	while (1)
	{


		if (circle_count < 0 && (ANTIHORARIO == 0))		// si gira en sentido creciente y (contador de vuelta < 0) reseteo contador de vueltas
		{
			circle_count = 0;
		}
		if (circle_count < 0 && (ANTIHORARIO == 1))		// si gira en sentido decreciente y (contador de vuelta < 0) reseteo contador del timer
		{
			TIM3->CNT = TIM_CNT_DEFAULT;
		}
		if ( circle_count >= 0)							// si (contador de vuelta > 0) imprimo medicion
		{
			medicion = (TIM3->CNT + (TIM_CNT_MAX + 0x01) * circle_count) / AVANCE ;

		}
		else											// si (contador de vuelta < 0) imprimo 0
		{
			medicion = 0;
		}

		CurrentDisplay[0] = char2segments((char) (medicion / 1000) + '0');
		CurrentDisplay[1] = DP_SEG | char2segments((char) ((medicion % 1000) / 100) + '0');
		CurrentDisplay[2] = char2segments((char) ((medicion % 100) / 10) + '0');
		CurrentDisplay[3] = char2segments((char) (medicion % 10) + '0');

		tm1637_DisplayHandle(7, CurrentDisplay);

		printMsg("enc: %d  vueltas: %d Medicion %d\n",TIM3->CNT, circle_count, medicion);

		HAL_Delay(10);

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM3->SR & 0x01)
	{
		if( ANTIHORARIO == 0 )	    // rebalse por overflow
		{
			circle_count ++;
		}
		else if( ANTIHORARIO == 1 )	// rebalse por underflow
		{
			circle_count --;
		}
	}

	TIM3->SR &= ~(0x01);  			// Reset flag de interrupcion
}


//void EXTI0_IRQHandler(void)
//{
//	EXTI->PR |= EXTI_PR_PR0;						// borro flag de interrupcion en canal 0
//}
//
//void EXTI1_IRQHandler(void)
//{
//	EXTI->PR |= EXTI_PR_PR1;						// borro flag de interrupcion en canal 1
//	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
//}

//void init_gpio_interrupt(void)
//{
//	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
//
//	AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;   		// interrupcion en Pin A0
//	AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;		// interrupcion en Pin A1
//
//	EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1;	// interrucpiones configuradas como rising edge
//
//	EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1;		// habilito interrupciones en Line 0 y 1
//
//	NVIC_EnableIRQ(EXTI0_IRQn);
//	NVIC_EnableIRQ(EXTI1_IRQn);
//
//}
static void printMsg(char *msg, ...)
{
	char buff[80];
#ifdef DEBUG_UART

	// extract arguments from print function
	va_list args;
	va_start(args,msg);
	vsprintf(buff,msg,args);

	// send data to USART 8 bit data register
	for(int i=0; i < strlen(buff); i++)
	{
		USART1->DR = buff[i];
		while ( !(USART1->SR & USART_SR_TXE));
	}

#endif
}
void init_hardware_timer_version(void)
{

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //AFIO might not even be needed?

	// TIM3->PSC = 67; // prescaler = PSC + 1
	// TIM3->EGR |= TIM_EGR_UG; // force update of the registers (including prescaler)
	// TIM3->EGR &= ~(TIM_EGR_UG);

	TIM3->ARR = TIM_CNT_MAX;  		// rebalse del counter
	TIM3->DIER |= TIM_DIER_UIE;  	// habilito interrupcion por over/underflow
	TIM3->CNT = TIM_CNT_DEFAULT ;	// inicializo Timer counter

	//per datasheet instructions
	TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );  //step 1 and 2
	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);  // step 3 and 4
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;   //step 5
	TIM3->CR1 |= TIM_CR1_CEN;     //step 6

}

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

//static void MX_I2C1_Init(void)
//{
//	hi2c1.Instance = I2C1;
//	hi2c1.Init.ClockSpeed = 100000;
//	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//	hi2c1.Init.OwnAddress1 = 0;
//	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//	hi2c1.Init.OwnAddress2 = 0;
//	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}
static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
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
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|SCLK_Pin|SDO_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_11|SCLK_Pin|SDO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


}
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}

}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
