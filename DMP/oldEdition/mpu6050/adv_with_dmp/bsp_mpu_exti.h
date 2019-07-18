/**
 * @file bsp_mpu_exti.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-03-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifdef __cplusplus
   extern "C" {
#endif

#ifndef __MPU_EXTI_H
#define	__MPU_EXTI_H

#include "stm32f4xx_hal.h"


#define MPU_INT_GPIO_PORT                GPIOI
#define MPU_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOI
#define MPU_INT_GPIO_PIN                 GPIO_PIN_1
#define MPU_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOI
#define MPU_INT_EXTI_PINSOURCE           EXTI_PinSource1
#define MPU_INT_EXTI_LINE                EXTI_Line1
#define MPU_INT_EXTI_IRQ                 EXTI1_IRQn

#define MPU_IRQHandler                   EXTI1_IRQHandler

void EXTI_MPU_Config(void);

#endif /* __EXTI_H */

#ifdef __cplusplus
}
#endif
