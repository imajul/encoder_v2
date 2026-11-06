/*
 * medidor.c
 *
 * Logica pura del medidor. No usa HAL ni registros: se compila tambien en la
 * PC para las pruebas de tests/.
 */
#include "medidor.h"

int32_t medidor_acumular(int32_t pulsos, uint16_t cnt_anterior, uint16_t cnt_actual,
                         int permitir_negativo)
{
	int16_t delta = (int16_t)(uint16_t)(cnt_actual - cnt_anterior);
	int64_t nuevo = (int64_t)pulsos + delta;

	if (nuevo > INT32_MAX)
	{
		nuevo = INT32_MAX;
	}
	else if (nuevo < INT32_MIN)
	{
		nuevo = INT32_MIN;
	}

	if (!permitir_negativo && nuevo < 0)
	{
		nuevo = 0;
	}

	return (int32_t)nuevo;
}

int32_t medidor_pulsos_a_mm(int32_t pulsos, uint32_t perimetro_um, uint32_t pulsos_vuelta)
{
	if (pulsos_vuelta == 0u)
	{
		return 0;
	}
	int64_t num = (int64_t)pulsos * (int64_t)perimetro_um;
	return (int32_t)(num / ((int64_t)pulsos_vuelta * 1000));
}

uint32_t medidor_calibrar(uint32_t perimetro_um, int32_t medido_mm, int32_t real_mm)
{
	/* Por debajo de 1 m medido el error relativo del propio conteo es grande. */
	if (medido_mm < 1000 || real_mm <= 0)
	{
		return 0u;
	}
	uint64_t nuevo = ((uint64_t)perimetro_um * (uint64_t)real_mm + (uint64_t)medido_mm / 2u)
	                 / (uint64_t)medido_mm;
	if (nuevo > UINT32_MAX)
	{
		return 0u;
	}
	return (uint32_t)nuevo;
}

void medidor_formatear(int32_t mm, int rango_automatico, medidor_texto_t *out)
{
	static const uint32_t limite[5] = { 0u, 9u, 99u, 999u, 9999u };
	uint32_t cm = (uint32_t)((mm < 0 ? -(int64_t)mm : (int64_t)mm) / 10);
	int negativo = (mm < 0) && (cm > 0u);	/* -0.00 se muestra como 0.00 */
	int ndig = negativo ? 3 : 4;		/* el signo ocupa un digito */
	uint32_t valor;
	int decimales;

	if (cm <= limite[ndig])
	{
		valor = cm;			/* metros con 2 decimales */
		decimales = 2;
	}
	else if (rango_automatico && cm / 10u <= limite[ndig])
	{
		valor = cm / 10u;	/* metros con 1 decimal */
		decimales = 1;
	}
	else if (cm / 100u <= limite[ndig])
	{
		valor = cm / 100u;	/* metros enteros */
		decimales = 0;
	}
	else
	{
		/* Fuera de rango */
		out->c[0] = negativo ? '-' : 'O';
		out->c[1] = negativo ? 'O' : 'F';
		out->c[2] = negativo ? 'F' : 'L';
		out->c[3] = negativo ? 'L' : ' ';
		out->punto = -1;
		return;
	}

	/* Digitos alineados a la derecha. Se muestran al menos decimales + 1
	 * digitos (p.ej. "0.05"); los ceros a la izquierda se apagan. */
	int minimo = decimales + 1;
	int primero = 4;
	for (int i = 3; i >= 0; i--)
	{
		int posicion = 3 - i;		/* 0 = unidades del valor */
		if (valor != 0u || posicion < minimo)
		{
			out->c[i] = (char)('0' + (valor % 10u));
			valor /= 10u;
			primero = i;
		}
		else
		{
			out->c[i] = ' ';
		}
	}
	if (negativo && primero > 0)
	{
		out->c[primero - 1] = '-';
	}
	out->punto = (decimales > 0) ? (int8_t)(3 - decimales) : (int8_t)-1;
}
