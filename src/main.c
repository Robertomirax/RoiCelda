#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

#define ADS1232_PWDN_PORT GPIOB
#define ADS1232_PWDN_PIN GPIO_PIN_12
#define ADS1232_SCLK_PORT GPIOB
#define ADS1232_SCLK_PIN GPIO_PIN_13
#define ADS1232_DOUT_PORT GPIOB
#define ADS1232_DOUT_PIN GPIO_PIN_14

#define ADS1232_TIMEOUT_MS 1000U
#define ADS1232_READ_TIMEOUT INT32_MIN

#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_13

// ============================================================================
// VARIABLES Y CONFIGURACIÓN DE CALIBRACIÓN
// ============================================================================
#define NUM_MUESTRAS_FILTRO 160 // Muestras para el filtro de promediado

int32_t g_offset_tara = 11200;   // Inicializado con el promedio de tus mediciones
float g_factor_escala = 262.144f; // Cuentas por gramo (Valor estimado inicial)

void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void delay_us(volatile uint32_t microseconds)
{
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000U) / 5U;
    while (cycles--)
    {
        __NOP();
    }
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar DOUT (PB14)
    GPIO_InitStruct.Pin = ADS1232_DOUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADS1232_DOUT_PORT, &GPIO_InitStruct);

    // Configurar SCLK (PB13)
    GPIO_InitStruct.Pin = ADS1232_SCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS1232_SCLK_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);

    // Configurar PWDN (PB12)
    GPIO_InitStruct.Pin = ADS1232_PWDN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS1232_PWDN_PORT, &GPIO_InitStruct);

    // Reiniciar ADS1232
    HAL_GPIO_WritePin(ADS1232_PWDN_PORT, ADS1232_PWDN_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ADS1232_PWDN_PORT, ADS1232_PWDN_PIN, GPIO_PIN_SET);
    HAL_Delay(200);

    // Configurar LED (PC13)
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void UART1_Init(void)
{
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
    uint32_t count = 0;
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= ADS1232_TIMEOUT_MS)
        {
            return ADS1232_READ_TIMEOUT;
        }
    }

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

    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
    delay_us(1);

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
    int32_t muestra_a = *(const int32_t *)a;
    int32_t muestra_b = *(const int32_t *)b;

    return (muestra_a > muestra_b) - (muestra_a < muestra_b);
}

// Filtro: Toma N muestras, elimina las 10 menores y las 10 mayores
int32_t ADS1232_ReadFiltered(uint16_t n_muestras)
{
    const uint16_t muestras_descartadas = 10;
    int32_t muestras[NUM_MUESTRAS_FILTRO];

    if (n_muestras < (muestras_descartadas * 2 + 1))
        n_muestras = muestras_descartadas * 2 + 1;
    if (n_muestras > NUM_MUESTRAS_FILTRO)
        n_muestras = NUM_MUESTRAS_FILTRO;

    int64_t suma = 0;

    for (uint16_t i = 0; i < n_muestras; i++)
    {
        int32_t val = ADS1232_Read();
        if (val == ADS1232_READ_TIMEOUT)
            return ADS1232_READ_TIMEOUT;
        muestras[i] = val;
    }

    qsort(muestras, n_muestras, sizeof(muestras[0]), comparar_muestras);

    for (uint16_t i = muestras_descartadas; i < n_muestras - muestras_descartadas; i++)
        suma += muestras[i];

    return (int32_t)(suma / (n_muestras - muestras_descartadas * 2));
}

// Realiza la Tara (fija el cero)
void ADS1232_HacerTara(void)
{
    int32_t lect_tara = ADS1232_ReadFiltered(16);
    if (lect_tara != ADS1232_READ_TIMEOUT)
    {
        g_offset_tara = lect_tara;
    }
}
/*
// Calibra la escala usando un peso patrón conocido en gramos
void ADS1232_CalibrarEscala(float peso_patron_g)
{
    int32_t lect_con_peso = ADS1232_ReadFiltered(16);
    if (lect_con_peso != ADS1232_READ_TIMEOUT && peso_patron_g > 0.0f)
    {
        int32_t delta_cuentas = lect_con_peso - g_offset_tara;
        g_factor_escala = (float)delta_cuentas / peso_patron_g;
    }
}
    */

// ============================================================================
// MAIN
// ============================================================================

int main(void)
{
    HAL_Init();
    GPIO_Init();
    UART1_Init();

    char buffer[128];

    // Tara inicial al arrancar
   // ADS1232_HacerTara();

    while (1)
    {
        // Obtenemos la lectura filtrada
        int32_t raw_filtrado = ADS1232_ReadFiltered(NUM_MUESTRAS_FILTRO);

        if (raw_filtrado == ADS1232_READ_TIMEOUT)
        {
            snprintf(buffer, sizeof(buffer), "Error ADS1232: Timeout DOUT\r\n");
        }
        else
        {
            // Cálculo del peso en gramos
            float peso_gramos = (float)(raw_filtrado - g_offset_tara) / g_factor_escala;

            snprintf(buffer, sizeof(buffer), "RAW: %ld | Peso: %.2f g\r\n", raw_filtrado, peso_gramos);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        HAL_Delay(100);
    }
}