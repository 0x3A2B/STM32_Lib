/**
 * @file stdPID.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief 设置PID值
 * 
 * @param pid 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 */
void PID_set_f32(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd){
   pid_reset_f32(pid);       // We need to reset pid befor change the p, i, d value.
   pid->kp = Kp;
   pid->ki = Ki;
   pid->kd = Kd;
}

/**
 * @brief 增减PID值
 * 
 * @param pid 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 */
void PID_setdiff_f32(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd){
   pid_reset_f32(pid);       // We need to reset pid befor change the p, i, d value.
   pid->kp += Kp;
   pid->ki += Ki;
   pid->kd += Kd;
}
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/