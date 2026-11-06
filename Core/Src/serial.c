/*
 * serial.c
 */
#include "main.h"
#include "serial.h"
#include <stdarg.h>
#include <stdio.h>

#define TX_TAM		512u	/* potencias de 2 */
#define RX_TAM		64u

static UART_HandleTypeDef s_huart;

static uint8_t s_tx[TX_TAM];
static volatile uint16_t s_tx_ini = 0;		/* lo mueve la ISR */
static volatile uint16_t s_tx_fin = 0;		/* lo mueve el programa */

static uint8_t s_rx[RX_TAM];
static volatile uint16_t s_rx_ini = 0;		/* lo mueve el programa */
static volatile uint16_t s_rx_fin = 0;		/* lo mueve la ISR */

void serial_init(uint32_t baudios)
{
	s_huart.Instance = USART1;
	s_huart.Init.BaudRate = baudios;
	s_huart.Init.WordLength = UART_WORDLENGTH_8B;
	s_huart.Init.StopBits = UART_STOPBITS_1;
	s_huart.Init.Parity = UART_PARITY_NONE;
	s_huart.Init.Mode = UART_MODE_TX_RX;
	s_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	s_huart.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&s_huart) != HAL_OK)		/* pines en HAL_UART_MspInit */
	{
		Error_Handler();
	}

	USART1->CR1 |= USART_CR1_RXNEIE;
	HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void tx_poner(uint8_t c)
{
	uint16_t fin = s_tx_fin;
	uint16_t sig = (uint16_t)((fin + 1u) & (TX_TAM - 1u));
	if (sig == s_tx_ini)
	{
		return;									/* lleno: se descarta */
	}
	s_tx[fin] = c;
	s_tx_fin = sig;
}

void serial_escribir(const char *texto)
{
	while (*texto)
	{
		tx_poner((uint8_t)*texto++);
	}
	USART1->CR1 |= USART_CR1_TXEIE;				/* la ISR vacia el buffer */
}

void serial_printf(const char *formato, ...)
{
	char buf[128];
	va_list args;
	va_start(args, formato);
	vsnprintf(buf, sizeof(buf), formato, args);
	va_end(args);
	serial_escribir(buf);
}

int serial_leer(void)
{
	uint16_t ini = s_rx_ini;
	if (ini == s_rx_fin)
	{
		return -1;
	}
	uint8_t c = s_rx[ini];
	s_rx_ini = (uint16_t)((ini + 1u) & (RX_TAM - 1u));
	return c;
}

void USART1_IRQHandler(void)
{
	uint32_t sr = USART1->SR;

	/* Leer DR despues de SR limpia RXNE y tambien los errores ORE/FE/NE */
	if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
	{
		uint8_t c = (uint8_t)USART1->DR;
		if ((sr & USART_SR_RXNE) && !(sr & (USART_SR_FE | USART_SR_NE)))
		{
			uint16_t fin = s_rx_fin;
			uint16_t sig = (uint16_t)((fin + 1u) & (RX_TAM - 1u));
			if (sig != s_rx_ini)
			{
				s_rx[fin] = c;
				s_rx_fin = sig;
			}
		}
	}

	if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
	{
		uint16_t ini = s_tx_ini;
		if (ini == s_tx_fin)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;	/* nada mas para enviar */
		}
		else
		{
			USART1->DR = s_tx[ini];
			s_tx_ini = (uint16_t)((ini + 1u) & (TX_TAM - 1u));
		}
	}
}
