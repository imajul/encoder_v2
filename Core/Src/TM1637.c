/*______________In the name of Allah, Most Gracious, Most Merciful______________

	This library is written to allow the STM32 micro-controller to handle TM1637 LED
	driver using a manual communication protocol (through GPIO pins)

	Created by 				: Ward Almasarani
	Start Date 				: 2020/09/29
	file name 				: tm1637.c
	Version					: 1.0

	Version 2.0 (medidor de tela):
	- CLK push-pull, DIO open-drain: el TM1637 puede bajar la linea en el ACK
	  sin chocar con la salida del micro, y se lee el estado real de la linea.
	- Tiempos fijos con el contador de ciclos DWT (antes dependian de la
	  velocidad de HAL_GPIO_WritePin y el reloj iba demasiado rapido).
	- El ACK se verifica sobre DIO con timeout (antes se leia CLK, el bucle
	  no tenia cuerpo y se reconfiguraba un pin de GPIOD).
______________________________________________________________________________*/
#include "TM1637.h"

#define ACK_TIMEOUT_US		50u

static void delay_us(uint32_t us)
{
	uint32_t inicio = DWT->CYCCNT;
	uint32_t ciclos = us * (SystemCoreClock / 1000000u);
	while ((DWT->CYCCNT - inicio) < ciclos)
	{
	}
}

static inline void clk(int nivel)
{
	HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, nivel ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void dio(int nivel)		/* 1 = liberar (pull-up), 0 = bajar */
{
	HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, nivel ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline int dio_leer(void)
{
	return HAL_GPIO_ReadPin(TM1637_DIO_PORT, TM1637_DIO_PIN) == GPIO_PIN_SET;
}

static void inicio_trama(void)			/* DIO baja con CLK en alto */
{
	dio(1);
	clk(1);
	delay_us(TM1637_SEMICICLO_US);
	dio(0);
	delay_us(TM1637_SEMICICLO_US);
	clk(0);
	delay_us(TM1637_SEMICICLO_US);
}

static void fin_trama(void)				/* DIO sube con CLK en alto */
{
	clk(0);
	dio(0);
	delay_us(TM1637_SEMICICLO_US);
	clk(1);
	delay_us(TM1637_SEMICICLO_US);
	dio(1);
	delay_us(TM1637_SEMICICLO_US);
}

/* Envia un byte (LSB primero). Devuelve 1 si el TM1637 respondio con ACK. */
static int escribir_byte(uint8_t b)
{
	for (uint8_t i = 0; i < 8u; i++)
	{
		clk(0);
		dio(b & 0x01u);
		b >>= 1;
		delay_us(TM1637_SEMICICLO_US);
		clk(1);
		delay_us(TM1637_SEMICICLO_US);
	}

	/* 9no pulso: el TM1637 baja DIO como ACK */
	clk(0);
	dio(1);
	uint32_t espera = 0;
	while (dio_leer() && espera < ACK_TIMEOUT_US)
	{
		delay_us(1);
		espera++;
	}
	int ack = !dio_leer();
	clk(1);
	delay_us(TM1637_SEMICICLO_US);
	clk(0);
	delay_us(TM1637_SEMICICLO_US);
	return ack;
}

void tm1637_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	/* Contador de ciclos para los retardos */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);

	gpio.Pin = TM1637_CLK_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TM1637_CLK_PORT, &gpio);

	gpio.Pin = TM1637_DIO_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;		/* el modulo trae pull-up */
	HAL_GPIO_Init(TM1637_DIO_PORT, &gpio);
}

static int comando(uint8_t cmd)
{
	inicio_trama();
	int ok = escribir_byte(cmd);
	fin_trama();
	return ok;
}

tm1637_StatusTypedef tm1637_DisplayHandle(uint8_t Brightness, const uint8_t *DisplayBuffer)
{
	if (Brightness > 7u)
	{
		return TM1637_ERROR;
	}

	int ok = comando(DATA_SET);

	inicio_trama();
	ok &= escribir_byte(C0H);
	for (uint8_t i = 0; i < 4u; i++)
	{
		ok &= escribir_byte(DisplayBuffer[i]);
	}
	fin_trama();

	ok &= comando((uint8_t)(DISPLAY_ON | Brightness));

	return ok ? TM1637_OK : TM1637_ERROR;
}

tm1637_StatusTypedef tm1637_DisplayClear(void)
{
	static const uint8_t vacio[4] = {0};
	tm1637_StatusTypedef st = tm1637_DisplayHandle(0, vacio);
	if (!comando(DISPLAY_OFF))
	{
		st = TM1637_ERROR;
	}
	return st;
}

uint8_t char2segments(char c) {
        switch (c) {
			case '0' : return 0x3f;
			case '1' : return 0x06;
			case '2' : return 0x5b;
			case '3' : return 0x4f;
			case '4' : return 0x66;
			case '5' : return 0x6d;
			case '6' : return 0x7d;
			case '7' : return 0x07;
			case '8' : return 0x7f;
			case '9' : return 0x6f;
            case '_' : return 0x08;
            case '^' : return 0x01; // ¯
            case '-' : return 0x40;
            case '*' : return 0x63; // °
            case ' ' : return 0x00; // space
            case 'A' : return 0x77; // upper case A
            case 'a' : return 0x5f; // lower case a
            case 'B' :              // lower case b
            case 'b' : return 0x7c; // lower case b
            case 'C' : return 0x39; // upper case C
            case 'c' : return 0x58; // lower case c
            case 'D' :              // lower case d
            case 'd' : return 0x5e; // lower case d
            case 'E' :              // upper case E
            case 'e' : return 0x79; // upper case E
            case 'F' :              // upper case F
            case 'f' : return 0x71; // upper case F
            case 'G' :              // upper case G
            case 'g' : return 0x35; // upper case G
            case 'H' : return 0x76; // upper case H
            case 'h' : return 0x74; // lower case h
            case 'I' : return 0x06; // 1
            case 'i' : return 0x04; // lower case i
            case 'J' : return 0x1e; // upper case J
            case 'j' : return 0x16; // lower case j
            case 'K' :              // upper case K
            case 'k' : return 0x75; // upper case K
            case 'L' :              // upper case L
            case 'l' : return 0x38; // upper case L
            case 'M' :              // twice tall n
            case 'm' : return 0x37; // twice tall ∩
            case 'N' :              // lower case n
            case 'n' : return 0x54; // lower case n
            case 'O' :              // lower case o
            case 'o' : return 0x5c; // lower case o
            case 'P' :              // upper case P
            case 'p' : return 0x73; // upper case P
            case 'Q' : return 0x7b; // upper case Q
            case 'q' : return 0x67; // lower case q
            case 'R' :              // lower case r
            case 'r' : return 0x50; // lower case r
            case 'S' :              // 5
            case 's' : return 0x6d; // 5
            case 'T' :              // lower case t
            case 't' : return 0x78; // lower case t
            case 'U' :              // lower case u
            case 'u' : return 0x1c; // lower case u
            case 'V' :              // twice tall u
            case 'v' : return 0x3e; // twice tall u
            case 'W' : return 0x7e; // upside down A
            case 'w' : return 0x2a; // separated w
            case 'X' :              // upper case H
            case 'x' : return 0x76; // upper case H
            case 'Y' :              // lower case y
            case 'y' : return 0x6e; // lower case y
            case 'Z' :              // separated Z
            case 'z' : return 0x1b; // separated Z
        }
        return 0;
    }
