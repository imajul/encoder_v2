/*
 * encoder.h
 *
 * Lectura del encoder de cuadratura con TIM3 en modo encoder.
 * El contador de 16 bits del timer se muestrea cada 1 ms (desde SysTick) y la
 * diferencia se acumula en una posicion de 32 bits.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Configura PA6/PA7 y TIM3. Llamar una vez al inicio. */
void encoder_init(void);

/* Muestreo periodico: llamar desde SysTick_Handler (cada 1 ms). */
void encoder_tick_1ms(void);

/* Posicion acumulada en pulsos (lectura atomica). */
int32_t encoder_pulsos(void);

/* Pone la medicion en cero. */
void encoder_reset(void);

/* Copia de la posicion en los registros de backup, para recuperarla despues
 * de un reinicio por watchdog. */
void encoder_backup_init(void);
int  encoder_backup_restaurar(void);	/* 1 si habia un valor valido y se restauro */

#endif /* ENCODER_H */
