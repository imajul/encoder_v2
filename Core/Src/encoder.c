/*
 * encoder.c
 *
 * TIM3 en modo encoder (SMS = 011: cuenta los flancos de TI1 y TI2, x4).
 * No se usan interrupciones de desborde: cada 1 ms se lee CNT y se acumula la
 * diferencia con signo. Asi no hay carrera entre "vueltas" y CNT, y no importa
 * si el contador desborda varias veces hacia un lado y otro.
 */
#include "main.h"
#include "config.h"
#include "encoder.h"
#include "medidor.h"

#define BKP_MAGIA		0xA5C3u

static volatile int32_t s_pulsos = 0;
static uint16_t s_cnt_anterior = 0;
static uint8_t s_backup_activo = 0;

void encoder_init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();

	/* PA6 = TIM3_CH1, PA7 = TIM3_CH2 como entradas */
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = ENC_PULLUP_INTERNO ? GPIO_PULLUP : GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	TIM3->CR1 = 0;									/* timer detenido, CKD = 0 */
	TIM3->DIER = 0;									/* sin interrupciones */
	TIM3->PSC = 0;
	TIM3->ARR = 0xFFFF;

	/* CC1S = 01 (IC1 <- TI1), CC2S = 01 (IC2 <- TI2), con filtro digital */
	TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0
	            | ((ENC_FILTRO_ENTRADA & 0xFu) << TIM_CCMR1_IC1F_Pos)
	            | ((ENC_FILTRO_ENTRADA & 0xFu) << TIM_CCMR1_IC2F_Pos);

	/* Polaridad: invertir TI1 cambia el sentido de conteo */
	TIM3->CCER = ENC_INVERTIR_SENTIDO ? TIM_CCER_CC1P : 0u;

	/* Modo encoder 3: cuenta en ambos flancos de TI1 y TI2 */
	TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

	TIM3->CNT = 0;
	TIM3->EGR = TIM_EGR_UG;							/* carga PSC/ARR */
	TIM3->SR = 0;
	TIM3->CR1 = TIM_CR1_CEN;

	s_cnt_anterior = (uint16_t)TIM3->CNT;
	s_pulsos = 0;
}

static void backup_guardar(int32_t valor)
{
	uint16_t lo = (uint16_t)((uint32_t)valor & 0xFFFFu);
	uint16_t hi = (uint16_t)((uint32_t)valor >> 16);
	BKP->DR1 = lo;
	BKP->DR2 = hi;
	BKP->DR3 = (uint16_t)(lo ^ hi ^ BKP_MAGIA);
}

void encoder_tick_1ms(void)
{
	uint16_t cnt = (uint16_t)TIM3->CNT;
	int32_t anterior = s_pulsos;

	s_pulsos = medidor_acumular(anterior, s_cnt_anterior, cnt, ENC_PERMITIR_NEGATIVO);
	s_cnt_anterior = cnt;

	if (s_backup_activo && s_pulsos != anterior)
	{
		backup_guardar(s_pulsos);
	}
}

int32_t encoder_pulsos(void)
{
	return s_pulsos;		/* lectura de 32 bits alineada: atomica en Cortex-M3 */
}

void encoder_reset(void)
{
	__disable_irq();
	s_cnt_anterior = (uint16_t)TIM3->CNT;
	s_pulsos = 0;
	if (s_backup_activo)
	{
		backup_guardar(0);
	}
	__enable_irq();
}

void encoder_backup_init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_BKP_CLK_ENABLE();
	PWR->CR |= PWR_CR_DBP;							/* habilita escritura en BKP */
	s_backup_activo = 1;
}

int encoder_backup_restaurar(void)
{
	uint16_t lo = (uint16_t)BKP->DR1;
	uint16_t hi = (uint16_t)BKP->DR2;
	uint16_t chk = (uint16_t)BKP->DR3;

	if (chk != (uint16_t)(lo ^ hi ^ BKP_MAGIA))
	{
		return 0;
	}

	__disable_irq();
	s_cnt_anterior = (uint16_t)TIM3->CNT;
	s_pulsos = (int32_t)(((uint32_t)hi << 16) | lo);
	__enable_irq();
	return 1;
}
