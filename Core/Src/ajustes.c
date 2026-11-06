/*
 * ajustes.c
 *
 * Usa la ultima pagina (1 KB) de los 64 KB de flash del STM32F103C8.
 */
#include "main.h"
#include "config.h"
#include "ajustes.h"

#define AJUSTES_DIRECCION	0x0800FC00u		/* pagina 63 */
#define AJUSTES_MAGIA		0x4D454431u		/* "MED1" */

typedef struct
{
	uint32_t magia;
	uint32_t perimetro_um;
	uint32_t control;						/* ~perimetro_um */
} ajustes_t;

/* Simbolos del linker: fin de la imagen del programa en flash */
extern uint32_t _sidata, _sdata, _edata;

static int perimetro_valido(uint32_t p)
{
	return (p >= ENC_PERIMETRO_UM_MIN) && (p <= ENC_PERIMETRO_UM_MAX);
}

uint32_t ajustes_leer_perimetro(void)
{
	const ajustes_t *a = (const ajustes_t *)AJUSTES_DIRECCION;
	if (a->magia == AJUSTES_MAGIA && a->control == ~a->perimetro_um
	    && perimetro_valido(a->perimetro_um))
	{
		return a->perimetro_um;
	}
	return ENC_PERIMETRO_UM_DEFECTO;
}

int ajustes_guardar_perimetro(uint32_t perimetro_um)
{
	if (!perimetro_valido(perimetro_um))
	{
		return 0;
	}

	/* Seguridad: no borrar la pagina si el programa llega hasta ella */
	uint32_t fin_programa = (uint32_t)&_sidata
	                      + ((uint32_t)&_edata - (uint32_t)&_sdata);
	if (fin_programa > AJUSTES_DIRECCION)
	{
		return 0;
	}

	const ajustes_t nuevo = { AJUSTES_MAGIA, perimetro_um, ~perimetro_um };
	const uint32_t *palabras = (const uint32_t *)&nuevo;
	FLASH_EraseInitTypeDef borrado = {0};
	uint32_t error_pagina = 0;
	int ok = 1;

	HAL_FLASH_Unlock();
	borrado.TypeErase = FLASH_TYPEERASE_PAGES;
	borrado.PageAddress = AJUSTES_DIRECCION;
	borrado.NbPages = 1;
	if (HAL_FLASHEx_Erase(&borrado, &error_pagina) != HAL_OK)
	{
		ok = 0;
	}
	for (uint32_t i = 0; ok && i < sizeof(nuevo) / 4u; i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, AJUSTES_DIRECCION + 4u * i,
		                      palabras[i]) != HAL_OK)
		{
			ok = 0;
		}
	}
	HAL_FLASH_Lock();

	return ok && (ajustes_leer_perimetro() == perimetro_um);
}
