/**
 * @file bsp_mpu_SysTick.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-03-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __BSP_MPU_SYSTICK_H__ 
#define __BSP_MPU_SYSTICK_H__ 

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void SysTick_Init(void);
void Delay_ms(__IO uint32_t nTime);
int get_tick_count(unsigned long *count);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void TimeStamp_Increment(void);
void TimingDelay_Decrement(void);
#ifdef __cplusplus
}
#endif


#endif	// __BSP_MPU_SYSTICK_H__
