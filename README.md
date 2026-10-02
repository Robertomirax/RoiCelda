# RoiCelda

Firmware para una celda de carga conectada a un ADC ADS1232 y un sensor DS18B20, controlados por un STM32F103C8T6 (Blue Pill). El equipo filtra las lecturas de peso, mide la temperatura y envia ambos valores por USART1.

## Estado actual

- Microcontrolador: STM32F103C8T6.
- Framework: STM32Cube HAL mediante PlatformIO.
- ADC: ADS1232, lectura serie de 24 bits.
- Temperatura: DS18B20 mediante 1-Wire en PA8, con CRC del scratchpad.
- Lecturas crudas solicitadas por resultado: entre 1024 y 4096, agrupadas en bloques de 16.
- Filtrado de peso: valor central superior de cada bloque, rechazo de outliers por IQR, promedio de las medianas aceptadas y EMA dinamico.
- Filtrado de temperatura: dos bloques de tres conversiones, rechazo de outliers por IQR y EMA dinamico.
- Salida serie: 115200 baudios, 8 bits, sin paridad, 1 bit de parada (8N1).
- Pausa final entre resultados: 100 ms, ademas del tiempo de captura y filtrado.

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

Conexion del DS18B20 en modo de tres hilos:

| Funcion | STM32F103C8T6 | DS18B20 |
| --- | --- | --- |
| Datos 1-Wire | PA8 | DQ |
| Alimentacion | 3.3 V | VDD |
| Referencia comun | GND | GND |

Instalar una resistencia pull-up de 4.7 kOhm entre PA8/DQ y 3.3 V, salvo que el modulo del sensor ya la incluya. No alimentar DQ con 5 V. El firmware usa `Skip ROM`, por lo que esta configurado para un solo DS18B20 en el bus.

El LED de estado esta conectado a PC13. En la mayoria de placas Blue Pill es activo en nivel bajo, pero el programa lo usa como indicador de actividad y lo conmuta despues de cada resultado.

> Verificar niveles logicos, alimentacion y masa comun antes de conectar el ADS1232 o el adaptador USB-UART. No conectar una senal de 5 V directamente a una entrada de 3.3 V.

## Funcionamiento

1. `HAL_Init()` inicializa la HAL y la base de tiempo del sistema.
2. `GPIO_Init()` configura las senales del ADS1232, reinicia el ADC y prepara el LED.
3. `UART1_Init()` configura USART1 a 115200 8N1.
4. El programa espera que DOUT indique una conversion disponible.
5. Lee 24 bits por medio de SCLK y extiende el signo para obtener un `int32_t`.
6. Inicia una conversion del DS18B20 y captura hasta 4096 lecturas del ADS1232 en bloques de 16. La captura puede detenerse al agotarse el presupuesto de 9.5 s, una vez reunidos al menos ocho bloques.
7. Calcula el valor central superior de cada bloque, aplica rechazo de outliers por IQR a esos valores, promedia los aceptados y aplica el EMA dinamico.
8. Espera a que termine la conversion inicial del DS18B20 si aun esta en curso y captura las conversiones restantes del filtro de temperatura.
9. Verifica el CRC del DS18B20, convierte las cuentas filtradas a gramos y envia peso y temperatura por UART.
10. Si DOUT permanece ocupado durante 1 segundo en una lectura, envia un mensaje de timeout.

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
5. Consultar `g_offset_tara` en el depurador y copiar su valor inicializado si se desea conservarlo tras reiniciar.
6. Volver a comentar la llamada si no se quiere repetir la tara en cada arranque.

La llamada de tara esta antes del bucle principal: si se habilita, se ejecuta una vez al arrancar. Usa el filtro de peso hasta 4096 muestras, sin actualizar el estado del EMA. La nueva tara solo queda en RAM mientras el equipo esta encendido; para conservarla tras reiniciar, actualizar `g_offset_tara` en el codigo.

### Factor de escala

Para obtener un factor nuevo con un peso patron conocido:

1. Habilitar la definicion comentada de `ADS1232_CalibrarEscala()` en `src/main.c`.
2. Establecer una tara valida y colocar un peso conocido sobre la celda.
3. Agregar una llamada a `ADS1232_CalibrarEscala(peso_en_gramos);` antes del bucle principal.
4. Observar el valor calculado de `g_factor_escala` en el depurador o imprimirlo por UART.
5. Guardar ese valor en el codigo y desactivar la llamada de calibracion.

La funcion y la llamada permanecen desactivadas porque la calibracion requiere colocar fisicamente el peso patron. El nuevo factor solo queda en RAM; copiarlo a `g_factor_escala` para conservarlo despues de reiniciar.

## Filtro de muestras

`FILTRO_OVERSAMPLE_MUESTRAS` solicita 4096 lecturas crudas y puede tomar valores entre `FILTRO_OVERSAMPLE_MIN` (1024) y `FILTRO_OVERSAMPLE_MAX` (4096). Cada bloque contiene 16 lecturas; su valor central superior alimenta el rechazo de outliers por IQR y el promedio de medianas aceptadas. El resultado pasa por un EMA dinamico.

Con la configuracion maxima:

```text
4096 lecturas / 16 por bloque = 256 valores de bloque antes del rechazo IQR
```

El arreglo de medianas ocupa 1024 bytes en la pila con el oversampling maximo (256 valores de 32 bits), ademas de los arreglos temporales de cada bloque. El STM32F103C8T6 dispone de 20 KB de RAM; considerar tambien el resto del uso de pila al cambiar los limites del filtro.

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
RAW: 123456 | Peso: 42.500 g | Temperatura: 23.500 C
```

Si no se detecta el DS18B20 o falla el CRC, la temperatura se muestra como `N/D`. Si falla el ADS1232, se informa el timeout y se muestra la temperatura cuando su lectura fue valida.

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
- Aumentar `FILTRO_OVERSAMPLE_MUESTRAS` solo si la latencia adicional es aceptable.
- Revisar `FILTRO_MEDIANA_BLOQUE`, `FILTRO_IQR_FACTOR` y el presupuesto de tiempo si el ruido produce valores atipicos frecuentes.

## Limitaciones conocidas

- La calibracion se guarda en el codigo; no existe memoria no volatil para conservarla automaticamente.
- `delay_us()` es una espera aproximada dependiente de `SystemCoreClock`.
- La lectura es bloqueante mientras espera cada conversion del ADS1232.
- El filtrado necesita guardar las medianas de bloque antes de calcular el promedio; con los limites actuales ocupa 1024 bytes para ese arreglo.
