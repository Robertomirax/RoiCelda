#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

#define HX711_DT_PORT  GPIOA
#define HX711_DT_PIN   GPIO_PIN_0
#define HX711_SCK_PORT GPIOA
#define HX711_SCK_PIN  GPIO_PIN_1

void SysTick_Handler(void) {
    HAL_IncTick();
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar DT (PA0) como Entrada sin pull
    GPIO_InitStruct.Pin = HX711_DT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HX711_DT_PORT, &GPIO_InitStruct);

    // Configurar SCK (PA1) como Salida Push-Pull
    GPIO_InitStruct.Pin = HX711_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HX711_SCK_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
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

int32_t HX711_Read(void) {
    uint32_t count = 0;

    // Esperar a que DT pase a nivel BAJO (datos listos)
    while (HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN) == GPIO_PIN_SET);

    // Leer 24 bits mediante pulsos de reloj
    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        count = count << 1;
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        if (HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN)) {
            count++;
        }
    }

    // Pulso 25 para fijar la ganancia a 128 (Canal A)
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);

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
        int32_t valor_raw = HX711_Read();
        snprintf(buffer, sizeof(buffer), "Lectura RAW HX711: %ld\r\n", valor_raw);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_Delay(500);
    }
}