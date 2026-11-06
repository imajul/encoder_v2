/*
 * ajustes.h
 *
 * Ajustes persistentes (calibracion) guardados en la ultima pagina de flash.
 */
#ifndef AJUSTES_H
#define AJUSTES_H

#include <stdint.h>

/* Devuelve el perimetro guardado en flash, o el valor por defecto si no hay
 * uno valido. */
uint32_t ajustes_leer_perimetro(void);

/* Guarda el perimetro en flash. Devuelve 1 si se grabo y verifico bien. */
int ajustes_guardar_perimetro(uint32_t perimetro_um);

#endif /* AJUSTES_H */
