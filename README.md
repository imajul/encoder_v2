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

### Encoder LPD3806-360BM

Es un encoder de 360 PPR con salidas **NPN de colector abierto**: solo baja la línea a GND y nunca entrega
tensión. Sin pull-up, las entradas quedan flotando y captan ruido. Se alimenta con 5–24 V.

| Cable | Conexión |
|---|---|
| Rojo | +5 V (o hasta 24 V) |
| Negro | GND (común con la Blue Pill) |
| Verde (A) | PA6 + resistencia de **2,2 kΩ a 3,3 V** |
| Blanco (B) | PA7 + resistencia de **2,2 kΩ a 3,3 V** |
| Malla | GND, solo del lado de la placa |

* Las resistencias de pull-up van a **3,3 V, nunca a 5 V ni a la alimentación del encoder**: PA6/PA7 no
  toleran 5 V. Como la salida es de colector abierto, el encoder puede alimentarse con 5–24 V sin problema.
* El pull-up interno del micro (~40 kΩ) alcanza para pruebas en la mesa, pero con cable largo y motores
  cerca es muy débil. Con 1–4,7 kΩ externos el flanco de subida es más rápido y la entrada es mucho más
  inmune al ruido.
* Opcional: un capacitor de 1 nF de cada entrada a GND, cerca del micro.
* 360 PPR × 4 flancos = 1440 pulsos/vuelta (`ENC_PULSOS_POR_VUELTA`). El encoder admite hasta 20 kHz por
  canal; con la rueda de 21,19 cm eso equivale a unos 700 m/min.

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
