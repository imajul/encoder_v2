/*______________In the name of Allah, Most Gracious, Most Merciful______________

	This library is written to allow the STM32 micro-controller to handle TM1637 LED
	driver using a manual communication protocol (through GPIO pins)

	Created by 				: Ward Almasarani
	Start Date 				: 2020/09/29
	file name 				: tm1637.h
	Version					: 1.0

	Version 2.0 (medidor de tela): protocolo reescrito con tiempos controlados,
	DIO en open-drain, verificacion de ACK con timeout y sin bloqueos.
______________________________________________________________________________*/
#ifndef TM1637_H
#define TM1637_H

#include "main.h"

/* Pines */
#define TM1637_CLK_PORT		GPIOB
#define TM1637_CLK_PIN		GPIO_PIN_6
#define TM1637_DIO_PORT		GPIOB
#define TM1637_DIO_PIN		GPIO_PIN_7

/* Semiciclo de reloj en microsegundos (~100 kHz). Los modulos tipicos tienen
 * 10k de pull-up y 100 pF en las lineas, asi que no conviene bajar de 5 us. */
#define TM1637_SEMICICLO_US	5u

//Segments
//  --0x01--
// |        |
//0x20     0x02
// |        |
//  --0x40- -
// |        |
//0x10     0x04
// |        |
//  --0x08--
#define A_SEG				0x01
#define B_SEG				0x02
#define C_SEG				0x04
#define D_SEG				0x08
#define E_SEG				0x10
#define F_SEG				0x20
#define G_SEG				0x40
#define DP_SEG				0x80

//Commands
#define DATA_SET			0x40	/* escritura con auto-incremento */
#define DATA_FIXED			0x44
#define C0H					0xC0	/* direccion del primer digito */
#define DISPLAY_ON			0x88
#define DISPLAY_OFF			0x80

typedef enum
{
	TM1637_OK = 0U,
	TM1637_ERROR		/* parametro invalido o el display no respondio (sin ACK) */
} tm1637_StatusTypedef;

/* Configura los pines. Llamar una vez antes de usar el display. */
void tm1637_Init(void);

/* Escribe los 4 digitos (segmentos crudos) y enciende con el brillo dado (0..7). */
tm1637_StatusTypedef tm1637_DisplayHandle(uint8_t Brightness, const uint8_t *DisplayBuffer);

/* Apaga el display. */
tm1637_StatusTypedef tm1637_DisplayClear(void);

/* Caracter ASCII -> segmentos. Devuelve 0 (apagado) si no lo conoce. */
uint8_t char2segments(char c);

#endif /* TM1637_H */
