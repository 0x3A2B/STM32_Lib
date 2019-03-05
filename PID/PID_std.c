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
   pid_reset_f32(pid);       // We need to reset pid befor change the kp, ki, kd value.
   pid->e = pid->ee = pid->sume = 0;
   pid->kp = Kp;
   pid->ki = Ki;
   pid->kd = Kd;
   pid->result = 0;
}

/**
 * @brief 步进式PID
 * 
 * @param pid 输入PID结构体
 * @param pv  当前测量值
 */
void StepPID(PID_f32 *pid, float32_t pv){
   float32_t thisE;  /**当前误差**/
   float32_t result; /**PID输出**/
   thisE = pid->tar - pv;
   if(fabs_pid(thisE) < pid->deadband){   //死区判断
         thisE = 0;
   }
   
   result = pid->kp * (thisE - pid->e) + pid->ki * thisE + pid->kd * (thisE - 2 * pid->e + pid->ee);
   pid->result += pid->result;

   pid->ee = pid->e;
   pid->e = thisE;
}


void PosPID(PID_f32 *pid, float32_t pv){
   float32_t thisE;  /**当前误差**/
   float32_t result; /**PID输出**/
   thisE = pid->tar - pv;
   if(fabs_pid(thisE) < pid->deadband){   //死区判断
         thisE = 0;
   }

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
/**
 * @brief 重置PID
 * 
 * @param pid 
 */
void pid_reset_f32(PID_f32 *pid){
   pid->kp = 0;
   pid->ki = 0;
   pid->kd = 0;
}
/*位置式*/
void PIDRegulator(PID_f32 *vPID,float pv)
{
   float32_t thisError;
   float32_t result;
   float32_t factor;

   thisError = vPID->tar-pv;//得到偏差值
   result = vPID->result;
   if(fabs(thisError) > vPID->deadband){
      vPID->integralValue = vPID->integralValue + thisError;

      //变积分系数获取
      factor = VariableIntegralCoefficient(thisError, vPID->errorabsmax, vPID->errorabsmin);

      //计算微分项增量带不完全微分
      vPID->derivative = vPID->kd * (1 - vPID->alpha) * (thisError - vPID->e + vPID->alpha * vPID->derivative);
      result = vPID->kp * thisError + vPID->ki * vPID->integralValue + vPID->derivative;

   }
   else{
      if((fabs_pid(vPID->tar - vPID->minimum) < vPID->deadband)&&(fabs_pid(pv - vPID->minimum) < vPID->deadband)){
         result=vPID->minimum;
      }
   }

   /*对输出限值，避免超调和积分饱和问题*/
   if(result>=vPID->maximum){
      result=vPID->maximum;
   }
   if(result<=vPID->minimum){
      result=vPID->minimum;
   }

   vPID->ee=vPID->e;//存放偏差用于下次运算
   vPID->e=thisError;
   vPID->result=result;
}


/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* 变积分系数处理函数，实现一个输出0和1之间的分段线性函数           */
/* 当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0   */
/* 当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间现行变化    */
/* float error，当前输入的偏差值                                         */
/* float absmax，偏差绝对值的最大值                                      */
/* float absmin，偏差绝对值的最小值                                      */
float32_t VariableIntegralCoefficient(float32_t error,float32_t absmax,float32_t absmin)
{
   float32_t factor=0.0;

   if(fabs_pid(error)<=absmin){
      factor=1.0;
   }
   else if(fabs_pid(error)>absmax){
      factor=0.0;
   }
   else{
      factor=(absmax-fabs_pid(error))/(absmax-absmin);
   }

   return factor;
}
