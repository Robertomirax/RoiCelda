// Firmware para STM32F103C8T6 + convertidor ADC para celda de carga ADS1232.
// El programa lee el ADC por GPIO, filtra las lecturas y transmite el peso
// calculado por USART1.
// Autor: Roberto Domingues
// Fecha: 02-10-2026
// Descripción: Firmware para la lectura de celda de carga y sensor de temperatura.
// Requiere la librería HAL de STM32 y está diseñado para la placa Blue Pill.
// Versión: 9

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Manejador global de la UART utilizada para enviar las mediciones al PC.
UART_HandleTypeDef huart1;

// ----------------------------- ADS1232 ------------------------------------
// PWDN reinicia o despierta el ADS1232. SCLK genera los pulsos de lectura y
// DOUT indica tanto que hay una conversión disponible como el bit de datos.
#define ADS1232_PWDN_PORT GPIOB
#define ADS1232_PWDN_PIN GPIO_PIN_12
#define ADS1232_SCLK_PORT GPIOB
#define ADS1232_SCLK_PIN GPIO_PIN_13
#define ADS1232_DOUT_PORT GPIOB
#define ADS1232_DOUT_PIN GPIO_PIN_14

#define ADS1232_TIMEOUT_MS 1000U       // Tiempo máximo esperando una conversión.
#define ADS1232_READ_TIMEOUT INT32_MIN // Valor reservado para indicar timeout.

// DS18B20 conectado a PA8. DQ necesita una resistencia externa de pull-up.
#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_8
#define DS18B20_CONVERSION_MS 750U // Tiempo máximo a resolución de 12 bits.
#define TEMP_READ_ERROR INT32_MIN  // Valor reservado para indicar fallo de 1-Wire/CRC.

// LED de estado de la placa Blue Pill (normalmente activo en nivel bajo).
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_13

// ----------------------- Configuración de medición -------------------------
// Esquema de filtrado multicapa:
//   1) Mediana por bloque: elimina picos aislados de ruido antes de promediar.
//   2) Rechazo de outliers por Rango Intercuartílico (IQR) sobre las medianas.
//   3) Promediado masivo (oversampling de 1024 a 4096 muestras) con las
//      medianas de bloque que sobrevivieron al rechazo de outliers.
//   4) EMA dinámico de alta inercia (alfa ultra bajo) con modo rápido ante
//      cambios bruscos de carga, para no perder respuesta ante un peso nuevo.

// Capa 1: cantidad de lecturas crudas que se combinan en cada mediana.
#define FILTRO_MEDIANA_BLOQUE 16U

// Capa 3: oversampling total solicitado, ajustable entre los límites.
#define FILTRO_OVERSAMPLE_MUESTRAS 4096U
#define FILTRO_OVERSAMPLE_MIN 1024U
#define FILTRO_OVERSAMPLE_MAX 4096U

// Medianas de bloque que entran a las capas 2 y 3 (con el oversampling máximo).
#define FILTRO_NUM_BLOQUES (FILTRO_OVERSAMPLE_MAX / FILTRO_MEDIANA_BLOQUE)

// Presupuesto de tiempo para las capas 1-3: se reserva el resto del ciclo de
// 14 s para las conversiones adicionales del filtro multicapa de temperatura
// (ver TEMP_FILTRO_* más abajo), el envío por UART y el delay final.
#define FILTRO_TIEMPO_MAX_MS 9500U

// Mínimo de bloques capturados antes de poder cortar por tiempo, para que el
// cálculo de cuartiles de la capa 2 siga siendo representativo.
#define FILTRO_NUM_BLOQUES_MIN 8U

// Capa 2: multiplicador de Tukey estándar para el rango intercuartílico.
#define FILTRO_IQR_FACTOR 1.5f

// Capa 4: alfa base ultra bajo para máxima estabilidad en reposo, y alfa
// rápido que se activa temporalmente cuando el cambio de peso es real.
#define FILTRO_EMA_ALPHA_BASE 0.005f
#define FILTRO_EMA_ALPHA_RAPIDO 0.25f
#define FILTRO_EMA_UMBRAL_CUENTAS 200.0f

// --------------- Filtrado multicapa para temperatura (DS18B20) ------------
// Mismo esquema de 4 capas que la celda de carga, con tamaños adaptados a que
// el DS18B20 solo entrega una conversión completa cada ~750 ms (12 bits):
//   1) Mediana de TEMP_FILTRO_MEDIANA_BLOQUE conversiones consecutivas.
//   2) Rechazo de outliers por IQR sobre las medianas de bloque.
//   3) Promedio masivo de las medianas que sobreviven al rechazo.
//   4) EMA dinámico de alta inercia, con modo rápido ante cambios reales de
//      temperatura (p. ej. la celda cambia de ambiente).
#define TEMP_FILTRO_MEDIANA_BLOQUE 3U
#define TEMP_FILTRO_NUM_BLOQUES 2U
// Conversiones DS18B20 totales por lectura (750 ms cada una, la primera se
// solapa con el filtrado del peso).
#define TEMP_FILTRO_MUESTRAS_TOTAL (TEMP_FILTRO_MEDIANA_BLOQUE * TEMP_FILTRO_NUM_BLOQUES)
#define TEMP_FILTRO_IQR_FACTOR 1.5f
#define TEMP_FILTRO_EMA_ALPHA_BASE 0.005f
#define TEMP_FILTRO_EMA_ALPHA_RAPIDO 0.25f
#define TEMP_FILTRO_EMA_UMBRAL_CUENTAS 16.0f // 1 °C expresado en cuentas de 1/16 °C.

// Valores de calibración. La tara representa la lectura sin carga y el factor
// indica cuántas cuentas del ADC equivalen a un gramo.
int32_t g_offset_tara = 11200;
float g_factor_escala = 262.144f;

// HAL utiliza este tick de 1 ms para HAL_Delay() y HAL_GetTick().
void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void delay_us(volatile uint32_t microseconds)
{
    // El ADS1232 necesita pulsos SCLK cortos. Esta espera es aproximada y
    // depende de SystemCoreClock; no sustituye a un temporizador hardware.
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000U) / 5U;
    while (cycles--)
    {
        __NOP();
    }
}

static void DS18B20_Init(void)
{
    // El contador DWT proporciona retardos precisos para los pulsos 1-Wire.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
}

static void DS18B20_DelayUs(uint32_t microseconds)
{
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

static uint32_t DS18B20_DisableInterrupts(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void DS18B20_RestoreInterrupts(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t DS18B20_Reset(void)
{
    uint32_t primask = DS18B20_DisableInterrupts();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    DS18B20_DelayUs(480U);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    DS18B20_DelayUs(70U);
    uint8_t presente = HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_RESET;
    DS18B20_DelayUs(410U);
    DS18B20_RestoreInterrupts(primask);
    return presente;
}

static void DS18B20_WriteBit(uint8_t bit)
{
    uint32_t primask = DS18B20_DisableInterrupts();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    if (bit)
    {
        DS18B20_DelayUs(6U);
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
        DS18B20_DelayUs(64U);
    }
    else
    {
        DS18B20_DelayUs(60U);
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
        DS18B20_DelayUs(10U);
    }
    DS18B20_RestoreInterrupts(primask);
}

static uint8_t DS18B20_ReadBit(void)
{
    uint32_t primask = DS18B20_DisableInterrupts();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    DS18B20_DelayUs(3U);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    DS18B20_DelayUs(10U);
    uint8_t bit = HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET;
    DS18B20_DelayUs(57U);
    DS18B20_RestoreInterrupts(primask);
    return bit;
}

static void DS18B20_WriteByte(uint8_t value)
{
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        DS18B20_WriteBit(value & 1U);
        value >>= 1;
    }
}

static uint8_t DS18B20_ReadByte(void)
{
    uint8_t value = 0U;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        value |= DS18B20_ReadBit() << bit;
    }
    return value;
}

static uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < length; i++)
    {
        uint8_t value = data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            uint8_t mix = (crc ^ value) & 1U;
            crc >>= 1;
            if (mix)
            {
                crc ^= 0x8CU;
            }
            value >>= 1;
        }
    }
    return crc;
}

static uint8_t DS18B20_StartConversion(void)
{
    if (!DS18B20_Reset())
    {
        return 0U;
    }

    DS18B20_WriteByte(0xCCU); // Skip ROM: se usa un único sensor.
    DS18B20_WriteByte(0x44U); // Convert T.
    return 1U;
}

// Lee el scratchpad y devuelve la temperatura cruda (cuentas de 1/16 °C) tras
// validar el CRC. Es la base de la capa 1 del filtro multicapa de temperatura.
static int32_t DS18B20_LeerCruda(void)
{
    uint8_t scratchpad[9];
    if (!DS18B20_Reset())
    {
        return TEMP_READ_ERROR;
    }

    DS18B20_WriteByte(0xCCU); // Skip ROM.
    DS18B20_WriteByte(0xBEU); // Read Scratchpad.
    for (uint8_t i = 0U; i < sizeof(scratchpad); i++)
    {
        scratchpad[i] = DS18B20_ReadByte();
    }

    if (DS18B20_CRC8(scratchpad, 8U) != scratchpad[8])
    {
        return TEMP_READ_ERROR;
    }

    return (int32_t)(int16_t)(((uint16_t)scratchpad[1] << 8) | scratchpad[0]);
}

void GPIO_Init(void)
{
    // Los pines A, B y C contienen las señales del ADC, la UART y el LED.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // DOUT es entrada: permanece alto mientras el ADC no tiene un dato nuevo.
    GPIO_InitStruct.Pin = ADS1232_DOUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADS1232_DOUT_PORT, &GPIO_InitStruct);

    // SCLK es salida y se mantiene inicialmente en bajo.
    GPIO_InitStruct.Pin = ADS1232_SCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS1232_SCLK_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);

    // PWDN es salida. Un pulso bajo reinicia el ADS1232.
    GPIO_InitStruct.Pin = ADS1232_PWDN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS1232_PWDN_PORT, &GPIO_InitStruct);

    // Reiniciar el ADC y esperar a que termine su arranque.
    HAL_GPIO_WritePin(ADS1232_PWDN_PORT, ADS1232_PWDN_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ADS1232_PWDN_PORT, ADS1232_PWDN_PIN, GPIO_PIN_SET);
    HAL_Delay(200);

    // El LED se utiliza como indicador de actividad: cambia de estado por
    // cada resultado transmitido.
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void UART1_Init(void)
{
    // USART1 usa PA9 como TX y PA10 como RX a 115200 baudios, 8N1.
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);
}

int32_t ADS1232_Read(void)
{
    // El ADS1232 entrega una conversión de 24 bits en complemento a dos.
    // Primero se espera a que DOUT baje, señal de que el dato está listo.
    uint32_t count = 0;
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= ADS1232_TIMEOUT_MS)
        {
            return ADS1232_READ_TIMEOUT;
        }
    }

    // Cada flanco alto de SCLK desplaza un bit hacia el microcontrolador.
    for (int i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
        delay_us(1);

        count = count << 1;
        if (HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) == GPIO_PIN_SET)
        {
            count++;
        }

        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
        delay_us(1);
    }

    // Pulso adicional requerido por el ADS1232 para iniciar la siguiente
    // conversión/ciclo de comunicación.
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
    delay_us(1);

    // Extender el signo de 24 a 32 bits conserva los valores negativos.
    if (count & 0x800000)
    {
        count |= 0xFF000000;
    }

    return (int32_t)count;
}

// ============================================================================
// FILTRADO DIGITAL Y FUNCIONES DE CALIBRACIÓN
// ============================================================================

static int comparar_muestras(const void *a, const void *b)
{
    // qsort() necesita una función que indique si a es menor, igual o mayor
    // que b. La resta directa se evita para no provocar overflow de enteros.
    int32_t muestra_a = *(const int32_t *)a;
    int32_t muestra_b = *(const int32_t *)b;

    return (muestra_a > muestra_b) - (muestra_a < muestra_b);
}

// Capa 1: ordena FILTRO_MEDIANA_BLOQUE lecturas crudas y devuelve el valor
// central superior (el bloque tiene tamaño par), reduciendo picos aislados.
static int32_t ADS1232_MedianaBloque(void)
{
    int32_t muestras[FILTRO_MEDIANA_BLOQUE];

    for (uint16_t i = 0; i < FILTRO_MEDIANA_BLOQUE; i++)
    {
        int32_t val = ADS1232_Read();
        if (val == ADS1232_READ_TIMEOUT)
            return ADS1232_READ_TIMEOUT;
        muestras[i] = val;
    }

    qsort(muestras, FILTRO_MEDIANA_BLOQUE, sizeof(muestras[0]), comparar_muestras);
    return muestras[FILTRO_MEDIANA_BLOQUE / 2U];
}

// Capas 2 y 3 (compartidas entre peso y temperatura): ordena los valores,
// calcula el rango intercuartílico (Q3 - Q1) y descarta todo valor fuera de
// [Q1 - k*IQR, Q3 + k*IQR]. Con los valores restantes calcula el promedio
// masivo (oversampling efectivo).
static int32_t FiltroIQR_Promedio(int32_t *valores, uint16_t n, float factor_iqr)
{
    qsort(valores, n, sizeof(valores[0]), comparar_muestras);

    int32_t q1 = valores[n / 4U];
    int32_t q3 = valores[(n * 3U) / 4U];
    float iqr = (float)(q3 - q1);
    float limite_inferior = (float)q1 - factor_iqr * iqr;
    float limite_superior = (float)q3 + factor_iqr * iqr;

    int64_t suma = 0;
    uint16_t validas = 0U;
    for (uint16_t i = 0; i < n; i++)
    {
        float v = (float)valores[i];
        if (v >= limite_inferior && v <= limite_superior)
        {
            suma += valores[i];
            validas++;
        }
    }

    // Si el IQR colapsa (todas las medianas son iguales) no hay nada que
    // descartar; se usa el bloque completo para no devolver una división por 0.
    if (validas == 0U)
    {
        for (uint16_t i = 0; i < n; i++)
            suma += valores[i];
        validas = n;
    }

    return (int32_t)(suma / validas);
}

// Capa 4 (compartida): EMA de alta inercia (alfa ultra bajo) para máxima
// estabilidad en reposo. Cuando el cambio respecto al valor filtrado supera
// el umbral -señal de un cambio real, no ruido- se usa un alfa alto temporal
// para seguir el nuevo valor sin esperar cientos de ciclos. El estado persiste
// en las variables apuntadas por ema/inicializado, propias de cada canal.
static float FiltroEMADinamico(float valor_entrada, float *ema, uint8_t *inicializado,
                               float alpha_base, float alpha_rapido, float umbral)
{
    if (!*inicializado)
    {
        *ema = valor_entrada;
        *inicializado = 1U;
        return *ema;
    }

    float delta = fabsf(valor_entrada - *ema);
    float alpha = (delta > umbral) ? alpha_rapido : alpha_base;
    *ema += alpha * (valor_entrada - *ema);
    return *ema;
}

static float ADS1232_FiltroEMADinamico(float valor_entrada)
{
    static float ema = 0.0f;
    static uint8_t inicializado = 0U;
    return FiltroEMADinamico(valor_entrada, &ema, &inicializado,
                             FILTRO_EMA_ALPHA_BASE, FILTRO_EMA_ALPHA_RAPIDO, FILTRO_EMA_UMBRAL_CUENTAS);
}

// Ejecuta las capas 1-3 (mediana + rechazo de outliers IQR + promedio masivo)
// y devuelve el resultado en cuentas de ADC, sin tocar el estado del EMA.
static int32_t ADS1232_ReadFiltradoMasivo(uint32_t n_muestras_totales)
{
    if (n_muestras_totales < FILTRO_OVERSAMPLE_MIN)
        n_muestras_totales = FILTRO_OVERSAMPLE_MIN;
    if (n_muestras_totales > FILTRO_OVERSAMPLE_MAX)
        n_muestras_totales = FILTRO_OVERSAMPLE_MAX;

    uint16_t n_bloques_objetivo = (uint16_t)(n_muestras_totales / FILTRO_MEDIANA_BLOQUE);
    int32_t medianas_bloque[FILTRO_NUM_BLOQUES];
    uint32_t inicio_ms = HAL_GetTick();
    uint16_t n_bloques_capturados = 0U;

    for (uint16_t i = 0; i < n_bloques_objetivo; i++)
    {
        // Corta el oversampling si se agota el presupuesto de tiempo, siempre
        // que ya existan bloques suficientes para un IQR confiable.
        if (n_bloques_capturados >= FILTRO_NUM_BLOQUES_MIN &&
            (HAL_GetTick() - inicio_ms) >= FILTRO_TIEMPO_MAX_MS)
            break;

        int32_t mediana = ADS1232_MedianaBloque();
        if (mediana == ADS1232_READ_TIMEOUT)
            return ADS1232_READ_TIMEOUT;
        medianas_bloque[n_bloques_capturados++] = mediana;
    }

    return FiltroIQR_Promedio(medianas_bloque, n_bloques_capturados, FILTRO_IQR_FACTOR);
}

// Captura y filtra un bloque de lecturas del ADS1232 con el esquema completo
// de 4 capas: mediana -> rechazo de outliers IQR -> promedio masivo -> EMA.
int32_t ADS1232_ReadFiltered(uint32_t n_muestras_totales)
{
    int32_t promedio_masivo = ADS1232_ReadFiltradoMasivo(n_muestras_totales);
    if (promedio_masivo == ADS1232_READ_TIMEOUT)
        return ADS1232_READ_TIMEOUT;

    float salida_ema = ADS1232_FiltroEMADinamico((float)promedio_masivo);
    return (int32_t)lroundf(salida_ema);
}

// Mide la plataforma sin carga y actualiza el cero de la báscula. Usa solo las
// capas 1-3 para no contaminar el estado del EMA con la medición de tara.
void ADS1232_HacerTara(void)
{
    int32_t lect_tara = ADS1232_ReadFiltradoMasivo(FILTRO_OVERSAMPLE_MUESTRAS);
    if (lect_tara != ADS1232_READ_TIMEOUT)
    {
        g_offset_tara = lect_tara;
    }
}

// ============================================================================
// FILTRADO MULTICAPA PARA TEMPERATURA (DS18B20)
// ============================================================================

// Capas 1-3: toma TEMP_FILTRO_MUESTRAS_TOTAL conversiones del DS18B20 en
// bloques, calcula la mediana de cada bloque, descarta outliers por IQR y
// promedia las medianas restantes. La primera conversión ya debe estar lista
// (el llamador la inició y esperó los 750 ms) para aprovechar el solapamiento
// con el filtrado del peso y no duplicar esa espera.
static int32_t DS18B20_ReadFiltradoMasivo(void)
{
    int32_t medianas_bloque[TEMP_FILTRO_NUM_BLOQUES];
    uint8_t primera_muestra = 1U;

    for (uint16_t b = 0; b < TEMP_FILTRO_NUM_BLOQUES; b++)
    {
        int32_t muestras[TEMP_FILTRO_MEDIANA_BLOQUE];

        for (uint16_t i = 0; i < TEMP_FILTRO_MEDIANA_BLOQUE; i++)
        {
            if (!primera_muestra)
            {
                if (!DS18B20_StartConversion())
                    return TEMP_READ_ERROR;
                HAL_Delay(DS18B20_CONVERSION_MS);
            }
            primera_muestra = 0U;

            int32_t raw = DS18B20_LeerCruda();
            if (raw == TEMP_READ_ERROR)
                return TEMP_READ_ERROR;
            muestras[i] = raw;
        }

        qsort(muestras, TEMP_FILTRO_MEDIANA_BLOQUE, sizeof(muestras[0]), comparar_muestras);
        medianas_bloque[b] = muestras[TEMP_FILTRO_MEDIANA_BLOQUE / 2U];
    }

    return FiltroIQR_Promedio(medianas_bloque, TEMP_FILTRO_NUM_BLOQUES, TEMP_FILTRO_IQR_FACTOR);
}

static float DS18B20_FiltroEMADinamico(float valor_entrada)
{
    static float ema = 0.0f;
    static uint8_t inicializado = 0U;
    return FiltroEMADinamico(valor_entrada, &ema, &inicializado,
                             TEMP_FILTRO_EMA_ALPHA_BASE, TEMP_FILTRO_EMA_ALPHA_RAPIDO, TEMP_FILTRO_EMA_UMBRAL_CUENTAS);
}

// Lee la temperatura con el mismo esquema de 4 capas que el peso. Requiere
// que el llamador ya haya iniciado y esperado la primera conversión.
static uint8_t DS18B20_ReadFiltered(float *temperatura_c)
{
    int32_t raw_masivo = DS18B20_ReadFiltradoMasivo();
    if (raw_masivo == TEMP_READ_ERROR)
        return 0U;

    float salida_ema = DS18B20_FiltroEMADinamico((float)raw_masivo);
    *temperatura_c = salida_ema / 16.0f;
    return 1U;
}
/* Calibración desactivada. Para usarla, habilitar esta función y llamarla con
    un peso patrón conocido, después de establecer una tara válida.
void ADS1232_CalibrarEscala(float peso_patron_g)
{
    int32_t lect_con_peso = ADS1232_ReadFiltered(FILTRO_OVERSAMPLE_MUESTRAS);
    if (lect_con_peso != ADS1232_READ_TIMEOUT && peso_patron_g > 0.0f)
    {
        int32_t delta_cuentas = lect_con_peso - g_offset_tara;
        g_factor_escala = (float)delta_cuentas / peso_patron_g;
    }
}
    */

int main(void)
{
    // Inicializar la HAL configura el tick del sistema y deja preparada la
    // base necesaria para los retardos usados por el resto del programa.
    HAL_Init();
    GPIO_Init();
    DS18B20_Init();
    UART1_Init();

    // El mensaje más largo cabe holgadamente en este búfer.
    char buffer[128];

    // Para hacer tara al arrancar, retirar toda carga y habilitar esta llamada.
    // La lectura actual de tara está precargada en g_offset_tara.
    // ADS1232_HacerTara();

    while (1)
    {
        // Iniciar la conversión ahora permite que ocurra mientras se lee el ADC.
        uint32_t inicio_temperatura_ms = HAL_GetTick();
        uint8_t conversion_iniciada = DS18B20_StartConversion();

        // Capturar hasta 4096 muestras puede tardar varios segundos según la
        // tasa del ADS1232; el resultado ya viene filtrado por las 4 capas.
        int32_t raw_filtrado = ADS1232_ReadFiltered(FILTRO_OVERSAMPLE_MUESTRAS);

        float temperatura_c = 0.0f;
        uint8_t temperatura_valida = 0U;
        if (conversion_iniciada)
        {
            uint32_t transcurrido_ms = HAL_GetTick() - inicio_temperatura_ms;
            if (transcurrido_ms < DS18B20_CONVERSION_MS)
            {
                HAL_Delay(DS18B20_CONVERSION_MS - transcurrido_ms);
            }
            // La primera conversión ya está lista; el resto de las capas del
            // filtro multicapa de temperatura toma las conversiones restantes.
            temperatura_valida = DS18B20_ReadFiltered(&temperatura_c);
        }

        if (raw_filtrado == ADS1232_READ_TIMEOUT)
        {
            if (temperatura_valida)
            {
                snprintf(buffer, sizeof(buffer), "Error ADS1232: Timeout DOUT | Temperatura: %.2f C\r\n", temperatura_c);
            }
            else
            {
                snprintf(buffer, sizeof(buffer), "Error ADS1232: Timeout DOUT | Temperatura: N/D\r\n");
            }
        }
        else
        {
            // Restar la tara elimina el peso propio de la plataforma y dividir
            // por el factor convierte cuentas del ADC en gramos.
            float peso_gramos = (float)(raw_filtrado - g_offset_tara) / g_factor_escala;

            if (temperatura_valida)
            {
                snprintf(buffer, sizeof(buffer), "RAW: %ld | Peso: %.3f g | Temperatura: %.3f C\r\n", raw_filtrado, peso_gramos, temperatura_c);
            }
            else
            {
                snprintf(buffer, sizeof(buffer), "RAW: %ld | Peso: %.3f g | Temperatura: N/D\r\n", raw_filtrado, peso_gramos);
            }
        }

        // Enviar la lectura al monitor serie y señalar actividad con el LED.
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        // Pausa entre bloques de medición para no saturar el monitor serie.
        HAL_Delay(100);
    }
}