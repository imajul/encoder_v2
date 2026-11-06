/*
 * Pruebas de la logica del medidor en la PC (no requiere la placa).
 *   cd tests && make
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "medidor.h"

static int fallas = 0;

#define VERIFICAR(cond, ...) do { if (!(cond)) { fallas++; \
	printf("FALLA %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void verificar_texto(int32_t mm, int automatico, const char *esperado, int punto)
{
	medidor_texto_t t;
	medidor_formatear(mm, automatico, &t);
	VERIFICAR(memcmp(t.c, esperado, 4) == 0 && t.punto == punto,
	          "formatear(%ld, %d) = \"%.4s\" punto %d, esperado \"%s\" punto %d",
	          (long)mm, automatico, t.c, t.punto, esperado, punto);
}

static void test_acumular(void)
{
	VERIFICAR(medidor_acumular(0, 100, 150, 1) == 50, "suma simple");
	VERIFICAR(medidor_acumular(0, 150, 100, 1) == -50, "resta simple");
	VERIFICAR(medidor_acumular(1000, 65535, 2, 1) == 1003, "desborde hacia arriba");
	VERIFICAR(medidor_acumular(1000, 2, 65530, 1) == 992, "desborde hacia abajo");
	VERIFICAR(medidor_acumular(5, 100, 90, 0) == 0, "no baja de cero");
	VERIFICAR(medidor_acumular(5, 100, 90, 1) == -5, "permite negativo");
	VERIFICAR(medidor_acumular(INT32_MAX - 1, 0, 100, 1) == INT32_MAX, "satura arriba");

	/* Simulacion: la tela avanza y retrocede al azar, con miles de
	 * desbordes del contador de 16 bits; el acumulado debe coincidir
	 * exactamente con la posicion real. */
	srand(1234);
	int64_t real = 0;
	int32_t acumulado = 0;
	uint16_t cnt_anterior = 0;
	for (int i = 0; i < 2000000; i++)
	{
		int paso = (rand() % 2001) - 900;		/* sesgo hacia adelante */
		if (i % 1000 < 50) paso = -paso;		/* tramos de retroceso */
		real += paso;
		uint16_t cnt = (uint16_t)(real & 0xFFFF);
		acumulado = medidor_acumular(acumulado, cnt_anterior, cnt, 1);
		cnt_anterior = cnt;
		if (acumulado != real)
		{
			VERIFICAR(0, "simulacion: paso %d acumulado %ld real %lld", i, (long)acumulado, (long long)real);
			break;
		}
	}
	VERIFICAR(real > 10 * 65536, "la simulacion debe cruzar muchas vueltas del contador (real=%lld)",
	          (long long)real);
}

static void test_conversion(void)
{
	VERIFICAR(medidor_pulsos_a_mm(144000, 211900, 1440) == 21190, "100 vueltas = 21190 mm");
	VERIFICAR(medidor_pulsos_a_mm(1440, 211900, 1440) == 211, "1 vuelta = 211 mm");
	VERIFICAR(medidor_pulsos_a_mm(-144000, 211900, 1440) == -21190, "negativo");
	/* 100 m: con el viejo factor entero 68 daba 99935 mm de "avance" por 100 m reales */
	int32_t p100m = (int32_t)((100000LL * 1440 * 1000) / 211900);
	int32_t mm = medidor_pulsos_a_mm(p100m, 211900, 1440);
	VERIFICAR(mm >= 99999 && mm <= 100000, "100 m -> %ld mm", (long)mm);
	/* Sin desborde con muchos pulsos (2^31 pulsos ~ 316 km) */
	VERIFICAR(medidor_pulsos_a_mm(INT32_MAX, 211900, 1440) > 300000000, "pulsos grandes");
	VERIFICAR(medidor_pulsos_a_mm(100, 211900, 0) == 0, "division por cero");
}

static void test_calibrar(void)
{
	VERIFICAR(medidor_calibrar(211900, 10100, 10000) == 209802, "calibracion basica");
	VERIFICAR(medidor_calibrar(211900, 10000, 10000) == 211900, "sin cambio");
	VERIFICAR(medidor_calibrar(211900, 999, 1000) == 0, "medido demasiado chico");
	VERIFICAR(medidor_calibrar(211900, 5000, 0) == 0, "real invalido");
}

static void test_formatear(void)
{
	verificar_texto(0,          1, " 000", 1);
	verificar_texto(50,         1, " 005", 1);
	verificar_texto(1234,       1, " 123", 1);
	verificar_texto(12345,      1, "1234", 1);
	verificar_texto(99999,      1, "9999", 1);
	verificar_texto(100000,     1, "1000", 2);	/* 100.0 */
	verificar_texto(999999,     1, "9999", 2);	/* 999.9 */
	verificar_texto(1000000,    1, "1000", -1);	/* 1000  */
	verificar_texto(9999999,    1, "9999", -1);
	verificar_texto(10000000,   1, "OFL ", -1);
	verificar_texto(100000,     0, " 100", -1);	/* modulo con ":" */
	verificar_texto(-50,        1, "-005", 1);	/* -0.05 */
	verificar_texto(-9,         1, " 000", 1);	/* sin "-0.00" */
	verificar_texto(-12345,     1, "-123", 2);	/* -12.3 */
	verificar_texto(-99999999,  1, "-OFL", -1);
}

int main(void)
{
	test_acumular();
	test_conversion();
	test_calibrar();
	test_formatear();
	if (fallas)
	{
		printf("%d pruebas fallaron\n", fallas);
		return 1;
	}
	printf("Todas las pruebas pasaron\n");
	return 0;
}
