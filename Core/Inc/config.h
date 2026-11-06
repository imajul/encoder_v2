/*
 * config.h
 *
 * Parametros configurables del medidor de tela.
 * Todo lo que haya que ajustar para una maquina concreta esta aca.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---------------------------------------------------------------------------
 * Encoder (TIM3 en modo encoder, CH1 = PA6, CH2 = PA7)
 * ------------------------------------------------------------------------- */

/* Flancos por vuelta de la rueda. El timer cuenta los 4 flancos de A y B,
 * asi que es 4 x PPR del encoder (360 PPR x 4 = 1440). */
#define ENC_PULSOS_POR_VUELTA		1440u

/* Perimetro de la rueda en micrometros (21.19 cm = 211900 um).
 * Es el valor por defecto: si se calibra por UART (comandos P o K) el valor
 * nuevo se guarda en flash y tiene prioridad sobre este. */
#define ENC_PERIMETRO_UM_DEFECTO	211900u

/* Limites aceptados para el perimetro (para descartar valores absurdos). */
#define ENC_PERIMETRO_UM_MIN		10000u		/*   1 cm */
#define ENC_PERIMETRO_UM_MAX		2000000u	/* 200 cm */

/* 1 = invierte el sentido de conteo (equivale a cruzar A y B). */
#define ENC_INVERTIR_SENTIDO		0

/* Filtro digital de las entradas (campo ICxF de TIM3, 0..15).
 * Con el timer a 72 MHz:
 *   0x0 = sin filtro
 *   0xA = muestrea a 4.5 MHz, 5 muestras -> ignora pulsos < ~1.1 us
 *   0xF = muestrea a 2.25 MHz, 8 muestras -> ignora pulsos < ~3.6 us
 * 0xF sigue permitiendo mas de 2000 m/min con 1440 pulsos/vuelta y 21 cm. */
#define ENC_FILTRO_ENTRADA			0xFu

/* 1 = pull-up interno en PA6/PA7. Necesario si el encoder es de colector
 * abierto (NPN) y no hay resistencias externas. Ver README (los pines PA6/PA7
 * NO toleran 5 V). */
#define ENC_PULLUP_INTERNO			1

/* 0 = la medicion no baja de cero: si la tela retrocede antes del origen, el
 *     origen se corre (comportamiento de la version anterior).
 * 1 = permite mediciones negativas. */
#define ENC_PERMITIR_NEGATIVO		0

/* ---------------------------------------------------------------------------
 * Boton de puesta a cero (activo en bajo, pull-up interno)
 * ------------------------------------------------------------------------- */
#define BTN_RESET_PORT				GPIOB
#define BTN_RESET_PIN				GPIO_PIN_12
#define BTN_ANTIRREBOTE_MS			50u

/* ---------------------------------------------------------------------------
 * Display TM1637 (CLK = PB6, DIO = PB7)
 * ------------------------------------------------------------------------- */
#define DISPLAY_BRILLO				7u		/* 0..7 */

/* 1 = rango automatico: 99.99 m -> 999.9 m -> 9999 m.
 *     Requiere un modulo con punto decimal en cada digito.
 * 0 = solo XX.XX (punto en el 2do digito) y despues XXXX metros sin punto.
 *     Usar con modulos de reloj que solo tienen ":" en el centro. */
#define DISPLAY_RANGO_AUTOMATICO	1

#define DISPLAY_REFRESCO_MIN_MS		50u		/* no actualizar mas seguido que esto */
#define DISPLAY_REFRESCO_MAX_MS		1000u	/* reescribir aunque no cambie (recupera glitches) */

/* ---------------------------------------------------------------------------
 * UART de depuracion / comandos (USART1, TX = PA9, RX = PA10)
 * ------------------------------------------------------------------------- */
#define SERIAL_BAUDIOS				9600u
#define TELEMETRIA_PERIODO_MS		500u

/* ---------------------------------------------------------------------------
 * Watchdog independiente
 * ------------------------------------------------------------------------- */
#define WATCHDOG_HABILITADO			1
#define WATCHDOG_TIMEOUT_MS			1000u

#endif /* CONFIG_H */
