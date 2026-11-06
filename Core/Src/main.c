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
/*
 * Medidor de tela con encoder de cuadratura.
 *
 *  - TIM3 en modo encoder cuenta por hardware (PA6/PA7) con filtro digital.
 *  - SysTick (1 ms) acumula los pulsos en 32 bits (ver encoder.c).
 *  - El lazo principal no bloquea: display, boton, UART y watchdog.
 *
 * Comandos por UART (terminar con Enter):
 *   R          pone la medicion en cero
 *   ?          muestra estado y calibracion
 *   T          activa/desactiva la telemetria periodica
 *   P<um>      fija el perimetro de la rueda en micrometros (ej: P211900)
 *   K<mm>      calibra: tras medir un largo conocido, indicar el largo real
 *              en mm (ej: K10000 si en realidad eran 10.000 m)
 */
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "encoder.h"
#include "medidor.h"
#include "serial.h"
#include "ajustes.h"
#include "TM1637.h"

#define VERSION_FW		"2.0"

static uint32_t g_perimetro_um = ENC_PERIMETRO_UM_DEFECTO;
static uint8_t  g_telemetria = 1;
static uint32_t g_errores_display = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void watchdog_init(void);
static void watchdog_refrescar(void);
static void procesar_serial(void);
static void procesar_boton(uint32_t ahora);
static void actualizar_display(uint32_t ahora);
static void telemetria(uint32_t ahora);
static void mostrar_estado(void);

static int32_t distancia_mm(void)
{
	return medidor_pulsos_a_mm(encoder_pulsos(), g_perimetro_um, ENC_PULSOS_POR_VUELTA);
}

int main(void)
{
	/* Causa del reinicio (hay que leerla antes de borrar los flags) */
	uint32_t causa = RCC->CSR;
	RCC->CSR |= RCC_CSR_RMVF;

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	tm1637_Init();
	serial_init(SERIAL_BAUDIOS);
	encoder_init();
	encoder_backup_init();

	g_perimetro_um = ajustes_leer_perimetro();

	/* Si el reinicio fue por watchdog, se recupera la medicion en curso */
	int restaurado = 0;
	if (causa & RCC_CSR_IWDGRSTF)
	{
		restaurado = encoder_backup_restaurar();
	}
	if (!restaurado)
	{
		encoder_reset();
	}

	serial_printf("\r\nMedidor de tela v%s - reinicio por:%s%s%s%s%s\r\n", VERSION_FW,
	              (causa & RCC_CSR_PORRSTF)  ? " encendido" : "",
	              (causa & RCC_CSR_PINRSTF)  ? " pin" : "",
	              (causa & RCC_CSR_IWDGRSTF) ? " WATCHDOG" : "",
	              (causa & RCC_CSR_SFTRSTF)  ? " software" : "",
	              restaurado ? " (medicion recuperada)" : "");
	mostrar_estado();

	watchdog_init();

	while (1)
	{
		uint32_t ahora = HAL_GetTick();

		watchdog_refrescar();
		procesar_serial();
		procesar_boton(ahora);
		actualizar_display(ahora);
		telemetria(ahora);

		static uint32_t led_ms = 0;
		if (ahora - led_ms >= 250u)						/* "latido" en PC13 */
		{
			led_ms = ahora;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
	}
}

/* ------------------------------------------------------------------------- */

static void actualizar_display(uint32_t ahora)
{
	static uint32_t ultimo_ms = 0;
	static uint8_t mostrado[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	static uint8_t pendiente = 1;

	if (ahora - ultimo_ms < DISPLAY_REFRESCO_MIN_MS)
	{
		return;
	}

	medidor_texto_t txt;
	uint8_t seg[4];
	medidor_formatear(distancia_mm(), DISPLAY_RANGO_AUTOMATICO, &txt);
	for (int i = 0; i < 4; i++)
	{
		seg[i] = char2segments(txt.c[i]);
		if (txt.punto == i)
		{
			seg[i] |= DP_SEG;
		}
	}

	if (pendiente || memcmp(seg, mostrado, sizeof(seg)) != 0
	    || ahora - ultimo_ms >= DISPLAY_REFRESCO_MAX_MS)
	{
		ultimo_ms = ahora;
		if (tm1637_DisplayHandle(DISPLAY_BRILLO, seg) == TM1637_OK)
		{
			memcpy(mostrado, seg, sizeof(seg));
			pendiente = 0;
		}
		else
		{
			g_errores_display++;
			pendiente = 1;							/* reintenta en el proximo ciclo */
		}
	}
}

static void procesar_boton(uint32_t ahora)
{
	static uint32_t ultimo_ms = 0;
	static uint32_t presionado_ms = 0;
	static uint8_t disparado = 0;

	if (ahora == ultimo_ms)
	{
		return;
	}
	uint32_t dt = ahora - ultimo_ms;
	ultimo_ms = ahora;

	if (HAL_GPIO_ReadPin(BTN_RESET_PORT, BTN_RESET_PIN) == GPIO_PIN_RESET)
	{
		presionado_ms += dt;
		if (!disparado && presionado_ms >= BTN_ANTIRREBOTE_MS)
		{
			disparado = 1;
			encoder_reset();
			serial_escribir("Medicion puesta a cero (boton)\r\n");
		}
	}
	else
	{
		presionado_ms = 0;
		disparado = 0;
	}
}

static void telemetria(uint32_t ahora)
{
	static uint32_t ultimo_ms = 0;
	static int32_t ultimo_mm = 0;

	uint32_t dt = ahora - ultimo_ms;
	if (dt < TELEMETRIA_PERIODO_MS)
	{
		return;
	}
	int32_t mm = distancia_mm();
	/* velocidad en cm/min = mm * 60000 / dt / 10 */
	int32_t cm_min = (int32_t)(((int64_t)(mm - ultimo_mm) * 6000) / (int64_t)dt);
	ultimo_ms = ahora;
	ultimo_mm = mm;

	if (!g_telemetria)
	{
		return;
	}
	int32_t v = abs(cm_min);
	serial_printf("pulsos=%ld mm=%ld vel=%s%ld.%02ld m/min err_disp=%lu\r\n",
	              (long)encoder_pulsos(), (long)mm,
	              (cm_min < 0) ? "-" : "", (long)(v / 100), (long)(v % 100),
	              (unsigned long)g_errores_display);
}

static void mostrar_estado(void)
{
	int32_t mm = distancia_mm();
	serial_printf("Medicion: %ld mm (%ld pulsos)\r\n", (long)mm, (long)encoder_pulsos());
	serial_printf("Perimetro: %lu um, %u pulsos/vuelta%s\r\n",
	              (unsigned long)g_perimetro_um, (unsigned)ENC_PULSOS_POR_VUELTA,
	              (g_perimetro_um == ENC_PERIMETRO_UM_DEFECTO) ? " (por defecto)" : "");
	serial_printf("Errores de display: %lu\r\n", (unsigned long)g_errores_display);
}

static void ejecutar_comando(char *linea)
{
	char cmd = linea[0];
	if (cmd >= 'a' && cmd <= 'z')
	{
		cmd = (char)(cmd - 'a' + 'A');
	}
	char *arg = linea + 1;
	char *fin = NULL;
	long valor = strtol(arg, &fin, 10);
	int hay_valor = (fin != arg);

	switch (cmd)
	{
	case 'R':
		encoder_reset();
		serial_escribir("Medicion puesta a cero\r\n");
		break;

	case '?':
		mostrar_estado();
		break;

	case 'T':
		g_telemetria = !g_telemetria;
		serial_printf("Telemetria %s\r\n", g_telemetria ? "activada" : "desactivada");
		break;

	case 'P':
		if (!hay_valor || valor <= 0)
		{
			serial_escribir("Uso: P<perimetro en um>, ej: P211900\r\n");
		}
		else if (ajustes_guardar_perimetro((uint32_t)valor))
		{
			g_perimetro_um = (uint32_t)valor;
			serial_printf("Perimetro guardado: %lu um\r\n", (unsigned long)g_perimetro_um);
		}
		else
		{
			serial_printf("Error: perimetro fuera de rango (%lu..%lu um) o fallo de flash\r\n",
			              (unsigned long)ENC_PERIMETRO_UM_MIN, (unsigned long)ENC_PERIMETRO_UM_MAX);
		}
		break;

	case 'K':
	{
		int32_t medido = distancia_mm();
		uint32_t nuevo = hay_valor ? medidor_calibrar(g_perimetro_um, medido, (int32_t)valor) : 0u;
		if (nuevo == 0u)
		{
			serial_escribir("Uso: medir un largo conocido (minimo 1 m) y enviar K<largo real en mm>\r\n");
		}
		else if (ajustes_guardar_perimetro(nuevo))
		{
			serial_printf("Calibrado: medido %ld mm, real %ld mm -> perimetro %lu um (antes %lu)\r\n",
			              (long)medido, valor, (unsigned long)nuevo, (unsigned long)g_perimetro_um);
			g_perimetro_um = nuevo;
		}
		else
		{
			serial_printf("Error: perimetro calculado %lu um fuera de rango o fallo de flash\r\n",
			              (unsigned long)nuevo);
		}
		break;
	}

	case '\0':
		break;

	default:
		serial_escribir("Comandos: R (cero), ? (estado), T (telemetria), P<um>, K<mm>\r\n");
		break;
	}
}

static void procesar_serial(void)
{
	static char linea[24];
	static uint8_t largo = 0;
	int c;

	while ((c = serial_leer()) >= 0)
	{
		if (c == '\r' || c == '\n')
		{
			linea[largo] = '\0';
			if (largo > 0u)
			{
				ejecutar_comando(linea);
			}
			largo = 0;
		}
		else if (largo < sizeof(linea) - 1u)
		{
			linea[largo++] = (char)c;
		}
	}
}

/* ------------------------------------------------------------------------- */

static void watchdog_init(void)
{
#if WATCHDOG_HABILITADO
	DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;			/* se detiene al pausar con el debugger */
	IWDG->KR = 0x5555;								/* habilita escritura de PR/RLR */
	IWDG->PR = 3;									/* LSI (~40 kHz) / 32 */
	IWDG->RLR = (WATCHDOG_TIMEOUT_MS * 40u) / 32u;
	while (IWDG->SR != 0u)
	{
	}
	IWDG->KR = 0xAAAA;
	IWDG->KR = 0xCCCC;								/* arranca */
#endif
}

static void watchdog_refrescar(void)
{
#if WATCHDOG_HABILITADO
	IWDG->KR = 0xAAAA;
#endif
}

/**
 * @brief Reloj del sistema: HSE 8 MHz x 9 = 72 MHz.
 *        Si el cristal no arranca, usa el oscilador interno (HSI/2 x 16 = 64 MHz)
 *        en lugar de quedarse colgado.
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef osc = {0};
	RCC_ClkInitTypeDef clk = {0};

	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState = RCC_HSE_ON;
	osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	osc.HSIState = RCC_HSI_ON;
	osc.PLL.PLLState = RCC_PLL_ON;
	osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	osc.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&osc) != HAL_OK)
	{
		RCC_OscInitTypeDef hsi = {0};
		hsi.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		hsi.HSIState = RCC_HSI_ON;
		hsi.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		hsi.PLL.PLLState = RCC_PLL_ON;
		hsi.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
		hsi.PLL.PLLMUL = RCC_PLL_MUL16;
		if (HAL_RCC_OscConfig(&hsi) != HAL_OK)
		{
			Error_Handler();
		}
	}

	clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clk.APB1CLKDivider = RCC_HCLK_DIV2;
	clk.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* PB11: salida en alto, igual que en la version anterior */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	gpio.Pin = GPIO_PIN_11;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio);

	/* PC13: LED de la placa */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	gpio.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &gpio);

	/* Boton de puesta a cero, activo en bajo */
	gpio.Pin = BTN_RESET_PIN;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BTN_RESET_PORT, &gpio);
}

/**
 * @brief Error irrecuperable: reinicia el micro en lugar de quedar colgado.
 */
void Error_Handler(void)
{
	__disable_irq();
	NVIC_SystemReset();
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	(void)file;
	(void)line;
}
#endif /* USE_FULL_ASSERT */
