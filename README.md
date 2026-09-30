# RoiCelda

Firmware para una celda de carga conectada a un ADC ADS1232 y controlada por un STM32F103C8T6 (Blue Pill). El equipo captura lecturas del ADC, elimina valores extremos, calcula el peso en gramos y lo envia por USART1.

## Estado actual

- Microcontrolador: STM32F103C8T6.
- Framework: STM32Cube HAL mediante PlatformIO.
- ADC: ADS1232, lectura serie de 24 bits.
- Lecturas por resultado: 160.
- Valores descartados: 10 menores y 10 mayores.
- Lecturas usadas para el promedio: 140.
- Salida serie: 115200 baudios, 8 bits, sin paridad, 1 bit de parada (8N1).
- Intervalo entre resultados: 100 ms, ademas del tiempo necesario para capturar las 160 conversiones.

## Hardware y conexiones

| Funcion | STM32F103C8T6 | ADS1232 |
| --- | --- | --- |
| Datos del ADC | PB14 | DOUT |
| Reloj del ADC | PB13 | SCLK |
| Reinicio / alimentacion del ADC | PB12 | PWDN |
| Alimentacion | 3.3 V | VCC |
| Referencia comun | GND | GND |

Conexion de la comunicacion serie:

| Funcion | STM32F103C8T6 |
| --- | --- |
| TX hacia el adaptador USB-UART | PA9 (USART1_TX) |
| RX desde el adaptador USB-UART | PA10 (USART1_RX) |
| GND | GND comun |

El LED de estado esta conectado a PC13. En la mayoria de placas Blue Pill es activo en nivel bajo, pero el programa lo usa como indicador de actividad y lo conmuta despues de cada resultado.

> Verificar niveles logicos, alimentacion y masa comun antes de conectar el ADS1232 o el adaptador USB-UART. No conectar una senal de 5 V directamente a una entrada de 3.3 V.

## Funcionamiento

1. `HAL_Init()` inicializa la HAL y la base de tiempo del sistema.
2. `GPIO_Init()` configura las senales del ADS1232, reinicia el ADC y prepara el LED.
3. `UART1_Init()` configura USART1 a 115200 8N1.
4. El programa espera que DOUT indique una conversion disponible.
5. Lee 24 bits por medio de SCLK y extiende el signo para obtener un `int32_t`.
6. Repite la lectura hasta reunir 160 muestras.
7. Ordena las muestras, descarta las 10 mas bajas y las 10 mas altas, y promedia las 140 restantes.
8. Convierte las cuentas filtradas a gramos y envia el resultado por UART.
9. Si DOUT permanece ocupado durante 1 segundo, envia un mensaje de timeout.

El codigo principal esta en [src/main.c](src/main.c).

## Calibracion

El peso se calcula con esta relacion:

```text
peso_gramos = (lectura_filtrada - g_offset_tara) / g_factor_escala
```

Los valores actuales se encuentran al principio de `src/main.c`:

```c
int32_t g_offset_tara = 11200;
float g_factor_escala = 262.144f;
```

### Tara

La tara es la lectura de la plataforma sin carga. Para recalcularla:

1. Retirar todo el peso de la plataforma.
2. Descomentar `ADS1232_HacerTara();` en `main()`.
3. Compilar y cargar el firmware.
4. Esperar a que termine la medicion inicial.
5. Copiar el valor obtenido a `g_offset_tara` si se desea dejarlo fijo.
6. Volver a comentar la llamada si no se quiere repetir la tara en cada arranque.

La tara usa el mismo bloque de 160 muestras y el mismo filtro que la medicion normal.

### Factor de escala

Para obtener un factor nuevo con un peso patron conocido:

1. Establecer una tara valida.
2. Colocar un peso conocido sobre la celda.
3. Ejecutar `ADS1232_CalibrarEscala(peso_en_gramos);` desde el codigo.
4. Observar el valor calculado de `g_factor_escala` en el depurador o imprimirlo por UART.
5. Guardar ese valor en el codigo y desactivar la llamada de calibracion.

La funcion de calibracion esta documentada dentro de `src/main.c`, pero permanece desactivada porque necesita intervencion fisica del operador.

## Filtro de muestras

`NUM_MUESTRAS_FILTRO` controla el tamano del bloque y actualmente vale 160. `MUESTRAS_DESCARTADAS` controla cuantos valores se eliminan en cada extremo y vale 10.

El filtro ordena el bloque completo usando `qsort()`. Con la configuracion actual:

```text
160 muestras - 10 minimas - 10 maximas = 140 muestras promediadas
```

El buffer ocupa 640 bytes en la pila (160 valores de 32 bits). El STM32F103C8T6 dispone de 20 KB de RAM, por lo que el consumo actual es adecuado. Si se aumenta el numero de muestras, revisar el uso de pila y memoria.

## Compilacion

Requisitos:

- Visual Studio Code.
- Extension PlatformIO.
- Toolchain STM32 instalado por PlatformIO.

Desde la carpeta del proyecto:

```powershell
C:\Users\rober\.platformio\penv\Scripts\platformio.exe run
```

Tambien se puede ejecutar la tarea `PlatformIO: Build` desde Visual Studio Code.

## Carga al microcontrolador

El proyecto esta configurado con `upload_protocol = stlink`, por lo que la carga normal se realiza mediante un programador ST-Link:

```powershell
C:\Users\rober\.platformio\penv\Scripts\platformio.exe run --target upload
```

El puerto `COM1` solo debe usarse si el metodo de carga conectado realmente aparece en ese puerto. Para ST-Link, PlatformIO normalmente detecta el dispositivo sin indicar un puerto serie.

## Monitor serie

Abrir un monitor a 115200 baudios, 8N1. La salida normal tiene este formato:

```text
RAW: 123456 | Peso: 42.50 g
```

Si el ADS1232 no entrega una conversion dentro del tiempo limite:

```text
Error ADS1232: Timeout DOUT
```

## Estructura del proyecto

```text
RoiCelda/
|-- platformio.ini       Configuracion de PlatformIO
|-- src/
|   `-- main.c           Firmware completo
|-- include/             Encabezados del proyecto, actualmente sin archivos propios
|-- lib/                 Librerias privadas, actualmente sin librerias propias
|-- test/                Espacio reservado para pruebas de PlatformIO
`-- diagrama_3.pdf       Diagrama de referencia del montaje
```

## Diagnostico rapido

### Solo aparecen timeouts

- Revisar alimentacion y GND del ADS1232.
- Confirmar que DOUT esta conectado a PB14.
- Confirmar que PWDN esta conectado a PB12 y queda en nivel alto despues del reinicio.
- Revisar que la celda y el ADS1232 esten correctamente conectados.
- Medir que las senales no superen 3.3 V.

### El peso tiene un desplazamiento constante

Repetir la tara sin carga y actualizar `g_offset_tara`.

### El peso tiene un error proporcional

Recalcular `g_factor_escala` usando un peso patron conocido.

### La lectura es inestable

- Confirmar que la celda esta mecanicamente fija.
- Revisar cables, masa y alimentacion.
- Mantener las conexiones de senal cortas.
- Aumentar `NUM_MUESTRAS_FILTRO` solo si la latencia adicional es aceptable.
- Ajustar `MUESTRAS_DESCARTADAS` si el ruido produce valores atipicos frecuentes.

## Limitaciones conocidas

- La calibracion se guarda en el codigo; no existe memoria no volatil para conservarla automaticamente.
- `delay_us()` es una espera aproximada dependiente de `SystemCoreClock`.
- La lectura es bloqueante mientras espera cada conversion del ADS1232.
- El filtrado necesita guardar todas las muestras del bloque antes de calcular el promedio.
