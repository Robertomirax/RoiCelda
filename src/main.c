#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

#define ADS1232_PWDN_PORT GPIOB
#define ADS1232_PWDN_PIN  GPIO_PIN_12
#define ADS1232_SCLK_PORT GPIOB
#define ADS1232_SCLK_PIN  GPIO_PIN_13
#define ADS1232_DOUT_PORT GPIOB
#define ADS1232_DOUT_PIN  GPIO_PIN_14
#define ADS1232_TIMEOUT_MS 1000U
#define ADS1232_READ_TIMEOUT INT32_MIN
#define LED_PORT       GPIOC
#define LED_PIN        GPIO_PIN_13

void SysTick_Handler(void) {
    HAL_IncTick();
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar DOUT (PB14) como Entrada sin pull
    GPIO_InitStruct.Pin = ADS1232_DOUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADS1232_DOUT_PORT, &GPIO_InitStruct);

    // Configurar SCLK (PB13) como Salida Push-Pull
    GPIO_InitStruct.Pin = ADS1232_SCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS1232_SCLK_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);

    // Reiniciar el ADS1232: PWDN bajo apaga y PWDN alto inicia conversiones
    GPIO_InitStruct.Pin = ADS1232_PWDN_PIN;
    HAL_GPIO_Init(ADS1232_PWDN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ADS1232_PWDN_PORT, ADS1232_PWDN_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ADS1232_PWDN_PORT, ADS1232_PWDN_PIN, GPIO_PIN_SET);
    HAL_Delay(200);

    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void UART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);
}

int32_t ADS1232_Read(void) {
    uint32_t count = 0;
    uint32_t start = HAL_GetTick();

    // Esperar a que DOUT pase a nivel BAJO (datos listos)
    while (HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= ADS1232_TIMEOUT_MS) {
            return ADS1232_READ_TIMEOUT;
        }
    }

    // Leer 24 bits mediante pulsos de reloj
    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
        count = count << 1;
        if (HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN)) {
            count++;
        }
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
    }

    // Extensión de signo para entero de 24 bits a 32 bits
    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    return (int32_t)count;
}

int main(void) {
    HAL_Init();
    GPIO_Init();
    UART1_Init();

    char buffer[64];

    while (1) {
        int32_t valor_raw = ADS1232_Read();
        if (valor_raw == ADS1232_READ_TIMEOUT) {
            snprintf(buffer, sizeof(buffer), "Error ADS1232: DOUT no baja\r\n");
        } else {
            snprintf(buffer, sizeof(buffer), "Lectura RAW ADS1232: %ld\r\n", valor_raw);
        }
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(500);
    }
}