encoder_v2
==========

Medidor de tela con encoder rotativo de cuadratura, STM32F103C8 ("Blue Pill") y display TM-1637 de 4 dígitos.

## Conexiones

| Señal | Pin | Notas |
|---|---|---|
| Encoder A | PA6 (TIM3_CH1) | Pull-up interno activado (`ENC_PULLUP_INTERNO`). **No tolera 5 V** |
| Encoder B | PA7 (TIM3_CH2) | Ídem |
| Display CLK | PB6 | |
| Display DIO | PB7 | Open-drain, usa el pull-up del módulo |
| Botón de puesta a cero | PB12 → GND | Activo en bajo, pull-up interno |
| UART TX / RX | PA9 / PA10 | 9600 8N1 |
| LED | PC13 | Parpadea cada 250 ms (indica que el programa está vivo) |

> **Importante:** PA6/PA7 no toleran 5 V. Si el encoder trabaja a 5–24 V (por ejemplo, NPN de colector abierto
> con pull-up a la alimentación), hay que adaptar el nivel: pull-up a 3,3 V en el lado del micro, divisor
> resistivo u optoacoplador. Con cables largos conviene usar un cable mallado con la malla a GND, y un
> capacitor de 1–10 nF a GND en cada entrada.

## Funcionamiento

* TIM3 en modo encoder cuenta los 4 flancos de A y B por hardware, con un filtro digital que ignora
  pulsos menores de ~3,6 µs (`ENC_FILTRO_ENTRADA`).
* Cada 1 ms (SysTick) se lee el contador de 16 bits y la diferencia con signo se acumula en 32 bits.
  No hay interrupción de desborde ni contador de vueltas, así que no se pueden producir los saltos de
  ±65536 pulsos (~9,6 m) que tenía la versión anterior.
* La distancia se calcula como `pulsos × perímetro / pulsos_por_vuelta`, con aritmética entera exacta.
  La versión anterior usaba un factor redondeado de 68 pulsos/cm, que daba un error de 6,5 cm cada 100 m.
* El display cambia de escala solo: `99.99` m → `999.9` m → `9999` m. Si el módulo solo tiene ":" en el
  centro (módulo de reloj), poner `DISPLAY_RANGO_AUTOMATICO` en 0.
* Hay un watchdog de 1 s. Si se reinicia por watchdog, la medición se recupera de los registros de backup.

Todos los parámetros están en `Core/Inc/config.h`.

## Comandos por UART

Se terminan con Enter.

| Comando | Acción |
|---|---|
| `R` | Pone la medición en cero |
| `?` | Muestra la medición, la calibración y el contador de errores del display |
| `T` | Activa o desactiva la telemetría periódica (cada 500 ms: pulsos, mm, velocidad) |
| `P<um>` | Fija el perímetro de la rueda en micrómetros, por ejemplo `P211900` (se guarda en flash) |
| `K<mm>` | Calibración automática (ver abajo) |

### Calibración

1. Poner a cero (`R` o el botón).
2. Pasar un largo de tela conocido y medido con cinta (cuanto más largo, mejor; mínimo 1 m).
3. Enviar `K` seguido del largo real en mm. Por ejemplo, `K10000` si eran 10,000 m.

El perímetro corregido se guarda en flash y se mantiene aunque se apague el equipo.

## Pruebas

La lógica de conteo, conversión y formato del display (`Core/Src/medidor.c`) se puede probar en la PC:

```
cd tests && make
```

Manual de referencia del STM32F103xx: https://www.st.com/resource/en/reference_manual/cd00171190.pdf
