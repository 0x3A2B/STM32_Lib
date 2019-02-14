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
/* Exported types ------------------------------------------------------------*/
/**
* @brief 8-bit fractional data type in 1.7 format.
*/
typedef int8_t q7_t;

/**
* @brief 16-bit fractional data type in 1.15 format.
*/
typedef int16_t q15_t;

/**
* @brief 32-bit fractional data type in 1.31 format.
*/
typedef int32_t q31_t;

/**
* @brief 64-bit fractional data type in 1.63 format.
*/
typedef int64_t q63_t;

/**
* @brief 32-bit floating-point type definition.
*/
typedef float float32_t;

/**s
* @brief 64-bit floating-point type definition.
*/
typedef double float64_t;
typedef struct
{

   float32_t kp;      /**< The proportional gain. */
   float32_t ki;      /**< The integral gain. */
   float32_t kd;      /**< The derivative gain. */

   float32_t e, ee;
   float32_t tar;     /**< The target value 目标值*/
   float32_t cur;     /**< The current 当前值*/
} PID_f32;  

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PID_setup_f32(PID_f32 *pid);
void PID_set_f32(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd);
void PID_setdiff_f32(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd);

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  /*  __PID_H__ */
