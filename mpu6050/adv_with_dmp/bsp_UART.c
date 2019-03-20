/**
 * @file bsp_UART.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-03-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "bsp_UART.h"
/* Exported types ------------------------------------------------------------*/
UART_HandleTypeDef *mpu_uart;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int Sensors_UART_Init(UART_HandleTypeDef *uart_t){
   mpu_uart = uart_t;
}
int Sensors_UART_SendChar(uint8_t ch){
   HAL_UART_Transmit(mpu_uart, &ch, 1, 0x3FFF);
}
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/