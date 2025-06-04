#include "stdio.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1; // 假设使用的是USART1

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}