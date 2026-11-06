/*
 * serial.h
 *
 * USART1 con buffers circulares por interrupcion: escribir nunca bloquea el
 * programa principal y no se pierden caracteres recibidos.
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void serial_init(uint32_t baudios);

/* Encola texto para transmitir. Si el buffer esta lleno, descarta lo que no
 * entra (nunca bloquea). */
void serial_escribir(const char *texto);
void serial_printf(const char *formato, ...) __attribute__((format(printf, 1, 2)));

/* Devuelve el siguiente caracter recibido, o -1 si no hay. */
int serial_leer(void);

#endif /* SERIAL_H */
