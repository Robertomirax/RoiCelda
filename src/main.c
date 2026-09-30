// Firmware para STM32F103C8T6 + convertidor ADC para celda de carga ADS1232.
// El programa lee el ADC por GPIO, filtra las lecturas y transmite el peso
// calculado por USART1.
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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

#define ADS1232_TIMEOUT_MS 1000U // Tiempo máximo esperando una conversión.
#define ADS1232_READ_TIMEOUT INT32_MIN // Valor reservado para indicar timeout.

// LED de estado de la placa Blue Pill (normalmente activo en nivel bajo).
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_13

// ----------------------- Configuración de medición -------------------------
// En cada resultado se capturan 160 lecturas. Se eliminan las 10 menores y
// las 10 mayores para reducir el efecto de ruido y valores atípicos; por lo
// tanto, el promedio final se calcula con 140 lecturas.
#define NUM_MUESTRAS_FILTRO 160U
#define MUESTRAS_DESCARTADAS 10U

// El número mínimo debe dejar al menos una lectura después de descartar ambos
// extremos. En la práctica se recomienda usar NUM_MUESTRAS_FILTRO.
#define MUESTRAS_MINIMAS_FILTRO (MUESTRAS_DESCARTADAS * 2U + 1U)

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

// Captura y filtra un bloque de lecturas del ADS1232.
int32_t ADS1232_ReadFiltered(uint16_t n_muestras)
{
    // El búfer vive en la pila. Con 160 muestras de 32 bits ocupa 640 bytes,
    // una cantidad adecuada para la RAM disponible del STM32F103C8T6.
    int32_t muestras[NUM_MUESTRAS_FILTRO];

    // Protecciones para que siempre exista un promedio válido y para no
    // escribir fuera del búfer si otra función solicita más muestras.
    if (n_muestras < MUESTRAS_MINIMAS_FILTRO)
        n_muestras = MUESTRAS_MINIMAS_FILTRO;
    if (n_muestras > NUM_MUESTRAS_FILTRO)
        n_muestras = NUM_MUESTRAS_FILTRO;

    int64_t suma = 0;

    // Capturar primero todo el bloque es necesario para poder ordenar las
    // lecturas y retirar exactamente los extremos solicitados.
    for (uint16_t i = 0; i < n_muestras; i++)
    {
        int32_t val = ADS1232_Read();
        if (val == ADS1232_READ_TIMEOUT)
            return ADS1232_READ_TIMEOUT;
        muestras[i] = val;
    }

    // Tras ordenar, las primeras 10 y las últimas 10 son los valores que se
    // excluyen del promedio.
    qsort(muestras, n_muestras, sizeof(muestras[0]), comparar_muestras);

    for (uint16_t i = MUESTRAS_DESCARTADAS;
         i < n_muestras - MUESTRAS_DESCARTADAS;
         i++)
        suma += muestras[i];

    return (int32_t)(suma / (n_muestras - MUESTRAS_DESCARTADAS * 2U));
}

// Mide la plataforma sin carga y actualiza el cero de la báscula.
void ADS1232_HacerTara(void)
{
    // Debe usarse el bloque completo: pedir 16 provocaría que, después de
    // descartar 20 valores, solo quedara una lectura para el promedio.
    int32_t lect_tara = ADS1232_ReadFiltered(NUM_MUESTRAS_FILTRO);
    if (lect_tara != ADS1232_READ_TIMEOUT)
    {
        g_offset_tara = lect_tara;
    }
}
/* Calibra la escala usando un peso patrón conocido en gramos.
// Esta rutina queda disponible para una calibración desde el código, pero no
// se ejecuta automáticamente porque requiere colocar físicamente el patrón.
void ADS1232_CalibrarEscala(float peso_patron_g)
{
    int32_t lect_con_peso = ADS1232_ReadFiltered(NUM_MUESTRAS_FILTRO);
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
    UART1_Init();

    // El mensaje más largo cabe holgadamente en este búfer.
    char buffer[128];

    // Para tarar automáticamente, retirar toda carga y descomentar la línea.
    // La tara almacenada actualmente es g_offset_tara.
    // ADS1232_HacerTara();

    while (1)
    {
        // Capturar 160 muestras puede tardar varios segundos según la tasa de
        // conversión del ADS1232; el resultado ya viene filtrado.
        int32_t raw_filtrado = ADS1232_ReadFiltered(NUM_MUESTRAS_FILTRO);

        if (raw_filtrado == ADS1232_READ_TIMEOUT)
        {
            // Si el ADC no responde, informar del fallo y continuar para que
            // el siguiente ciclo pueda intentar recuperar la comunicación.
            snprintf(buffer, sizeof(buffer), "Error ADS1232: Timeout DOUT\r\n");
        }
        else
        {
            // Restar la tara elimina el peso propio de la plataforma y dividir
            // por el factor convierte cuentas del ADC en gramos.
            float peso_gramos = (float)(raw_filtrado - g_offset_tara) / g_factor_escala;

            snprintf(buffer, sizeof(buffer), "RAW: %ld | Peso: %.2f g\r\n", raw_filtrado, peso_gramos);
        }

        // Enviar la lectura al monitor serie y señalar actividad con el LED.
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        // Pausa entre bloques de medición para no saturar el monitor serie.
        HAL_Delay(100);
    }
}