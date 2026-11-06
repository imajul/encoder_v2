/*
 * medidor.h
 *
 * Logica pura del medidor (sin acceso a hardware), para poder probarla en la
 * PC: acumulacion de pulsos, conversion a distancia y formato para el display.
 */
#ifndef MEDIDOR_H
#define MEDIDOR_H

#include <stdint.h>

/* Suma al acumulado la diferencia entre dos lecturas del contador de 16 bits.
 * La resta en 16 bits con signo resuelve el desborde del contador sin
 * necesidad de interrupciones, siempre que entre dos lecturas no haya mas de
 * 32767 pulsos (con lecturas cada 1 ms sobra).
 * Si permitir_negativo es 0, el resultado nunca baja de cero. */
int32_t medidor_acumular(int32_t pulsos, uint16_t cnt_anterior, uint16_t cnt_actual,
                         int permitir_negativo);

/* Convierte pulsos a milimetros: mm = pulsos * perimetro_um / (pulsos_vuelta * 1000).
 * Se hace con aritmetica entera de 64 bits sobre el total de pulsos, asi que
 * no se acumula error de redondeo. */
int32_t medidor_pulsos_a_mm(int32_t pulsos, uint32_t perimetro_um, uint32_t pulsos_vuelta);

/* Calcula un perimetro nuevo a partir de una medicion de un largo conocido.
 * Devuelve 0 si los datos no sirven (medido demasiado chico, etc). */
uint32_t medidor_calibrar(uint32_t perimetro_um, int32_t medido_mm, int32_t real_mm);

/* Texto de 4 digitos para el display. */
typedef struct
{
	char    c[4];		/* caracteres a mostrar (' ' = apagado) */
	int8_t  punto;		/* indice del digito con punto decimal, -1 = ninguno */
} medidor_texto_t;

/* Arma el texto del display a partir de la distancia en mm.
 * rango_automatico: ver DISPLAY_RANGO_AUTOMATICO en config.h. */
void medidor_formatear(int32_t mm, int rango_automatico, medidor_texto_t *out);

#endif /* MEDIDOR_H */
