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
#ifndef __PID_STD_H__ 
#define __PID_STD_H__ 

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{

   float32_t kp;            /**< The proportional gain. */
   float32_t ki;            /**< The integral gain. */
   float32_t kd;            /**< The derivative gain. */
   
   float32_t e, ee;         /**上次误差, 上上次误差**/
   float32_t sume;          /**累计误差**/
   float32_t tar;           /**< The target value 目标值*/
   float32_t cur;           /**< The current 当前值*/
   float32_t deadband;      /**死区**/
   float32_t maximum;
   float32_t minimum;
   float32_t errorabsmax;
   float32_t errorabsmin;
	   
	float32_t result;        /**PID输出**/

	float32_t alpha;         /**不完全微分系数**/
   float32_t derivative;    /**微分项**/
   float32_t integralValue; /**积分累计量**/

} PID_f32; 

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void PID_setup_f32(PID_f32 *pid);
void PID_set_f32(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd);
void PID_setdiff_f32(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd);
void pid_reset_f32(PID_f32 *pid);
void StepPID(PID_f32 *pid, float32_t pv);
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
float32_t VariableIntegralCoefficient(float32_t error,float32_t absmax,float32_t absmin);
#ifdef __cplusplus
}
#endif

#endif	// __PID_STD_H__