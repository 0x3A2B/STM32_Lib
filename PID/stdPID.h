/**
 * @file stdPID.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __STDPID_H__ 
#define __STDPID_H__ 
#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/

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

#endif	// __STDPID_H__