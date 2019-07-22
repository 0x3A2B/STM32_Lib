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
 * @brief 设置PID参数
 * 
 * @param pid 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 */
void PID_set(PID_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd)
{
   pid_reset_f32(pid);
   pid->kp = Kp;
   pid->ki = Ki;
   pid->kd = Kd;
   pid->result = 0;
}

/**
 * @brief 重置PID
 * 
 * @param pid  需要重置的PID结构体
 */
void pid_reset_f32(PID_f32 *pid)
{
   //  pid->kp = 0;
   //  pid->ki = 0;
   //  pid->kd = 0;
   pid->e = 0;
   pid->ee = 0;
   pid->sumE = 0;
   //   pid->cur= 0;
}

/**
 * @brief 步进式PID
 * TODO
 * @param pid 输入PID结构体
 * @param pv  当前测量值
 */
void StepPID(PID_f32 *vPID, float32_t pv)
{
   float32_t thisError;
   float32_t result;


   thisError = pv - vPID->tar;            //得到偏差值
   if (fabs(thisError) <= vPID->deadband) //判断死区
   {
      thisError = 0;
   }
#ifdef ADVCTRL
	 	 float32_t factor;
	 
   //变积分系数获取
   factor = VariableIntegralCoefficient(thisError, vPID->errorAbsMax, vPID->errorAbsMin);
   //变积分
   vPID->integral = factor * thisError;
   //不完全微分
   vPID->derivative = (1 - vPID->alpha) * (thisError - 2 * vPID->e + vPID->ee) + vPID->alpha * vPID->derivative;
   //计算PID输出
   result = vPID->kp * (thisError - vPID->e) + vPID->ki * vPID->integral + vPID->kd * vPID->derivative;
#else
   /* 经典步进PID*/
   result = vPID->kp * (thisError - vPID->e) + vPID->ki * thisError + vPID->kd * (thisError - 2 * vPID->e + vPID->ee);
   vPID->result += result;
#endif

   if (vPID->ampCtrl)
   {
      /*对输出限幅，避免超调和积分饱和问题*/
      if (vPID->result >= vPID->maximum)
      {
         vPID->result = vPID->maximum;
      }
      if (vPID->result <= vPID->minimum)
      {
         vPID->result = vPID->minimum;
      }
   }
   vPID->ee = vPID->e; //存放偏差用于下次运算
   vPID->e = thisError;
}

/**
 * @brief 位置式PID
 * 变积分 + 不完全微分 + 死区控制 + 梯形积分
 * @param vPID 
 * @param pv 
 */
void PosPID(PID_f32 *vPID, float pv)
{
   float32_t thisError;
   float32_t result;


   thisError = pv - vPID->tar;            //得到偏差值
   if (fabs(thisError) <= vPID->deadband) //判断死区
   {
      thisError = 0;
   }

#ifdef ADVCTRL
	 float32_t factor;
   //变积分系数获取
   factor = VariableIntegralCoefficient(thisError, vPID->errorAbsMax, vPID->errorAbsMin);
   //变积分
   vPID->integral = factor * thisError + vPID->sumE;
   //不完全微分
   vPID->derivative = (1 - vPID->alpha) * (thisError - vPID->e) + vPID->alpha * vPID->derivative;
   //计算PID输出
   result = vPID->kp * thisError + vPID->ki * vPID->integral + vPID->kd * vPID->derivative;
#else
   /* 经典位置PID*/
   vPID->sumE += vPID->ki * thisError;
   result = vPID->kp * thisError + vPID->sumE + vPID->kd * (thisError - vPID->e);
#endif

   if (vPID->ampCtrl)
   {
      /*对输出限幅，避免超调和积分饱和问题*/
      if (vPID->result >= vPID->maximum)
      {
         vPID->result = vPID->maximum;
      }
      if (vPID->result <= vPID->minimum)
      {
         vPID->result = vPID->minimum;
      }
   }

   vPID->ee = vPID->e; //存放偏差用于下次运算
   vPID->e = thisError;
   vPID->result = result;
}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief 变积分系数处理函数
 * 变积分系数处理函数，实现一个输出0和1之间的分段线性函数
 * 当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
 * 当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间变化
 * 
 * @param error  当前输入的偏差值
 * @param absmax 偏差绝对值的最大值
 * @param absmin 偏差绝对值的最小值
 * @return float32_t 变积分系数
 */
float32_t VariableIntegralCoefficient(float32_t error, float32_t absmax, float32_t absmin)
{
   float32_t factor = 0.0;

   if (fabs_pid(error) <= absmin)
   {
      factor = 1.0;
   }
   else if (fabs_pid(error) > absmax)
   {
      factor = 0.0;
   }
   else
   {
      factor = (absmax - fabs_pid(error)) / (absmax - absmin);
   }

   return factor;
}
