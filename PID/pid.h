/**
 * @file PID.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-04
 * 
 * @copyright Copyright (c) 2019
 * vb
 */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
   extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#ifdef ARM_MATH_CM4
#include "arm_math.h"
#endif
#include "pid_conf.h"

/* Exported types ------------------------------------------------------------*/
/**
* @brief 8-bit fractional data type in 1.7 format.
*/
//typedef int8_t q7_t;

/**
* @brief 16-bit fractional data type in 1.15 format.
*/
//typedef int16_t q15_t;

/**
* @brief 32-bit fractional data type in 1.31 format.
*/
//typedef int32_t q31_t;

/**
* @brief 64-bit fractional data type in 1.63 format.
*/
//typedef int64_t q63_t;

/**
* @brief 32-bit floating-point type definition.
*/
//typedef float float32_t;

/**s
* @brief 64-bit floating-point type definition.
*/
//typedef double float64_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
float32_t fabs_pid(float32_t error);

#ifdef __cplusplus
}
#endif

#endif  /* __PID_H__ */
