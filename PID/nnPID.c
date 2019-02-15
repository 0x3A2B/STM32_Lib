/**
 * @file nnPID.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-15
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "nnPID.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief          PID初始化
 * 
 * @param vPID     PID结构体
 * @param tar      目标值
 * @param vMax     量程最大值
 * @param vMin     量程最小值
 * @param deadband 死区
 */
void NeuralPIDInitialization(NEURALPID *vPID, float32_t tar, float vMax,float vMin, float deadband){
   vPID->tar = tar;                        /*设定值*/

   vPID->kcoef = 0.12;                     /*神经元输出比例*/
   vPID->kp = 0.4;                         /*比例学习速度*/
   vPID->ki = 0.35;                        /*积分学习速度*/
   vPID->kd = 0.4;                         /*微分学习速度*/

   vPID->e = 0.0;                          /*前一拍偏差*/
   vPID->ee = 0.0;                         /*前两拍偏差*/
	 vPID->result = -0.3;                       /*PID控制器结果 似乎不能为负*/
   vPID->output = 0.0;                     /*输出值，百分比*/

   vPID->maximum = vMax;                   /*输出值上限*/
   vPID->minimum = vMin;                   /*输出值下限*/  
   //vPID->deadband = (vMax - vMin)*0.0005;  /*死区*/
	 vPID->deadband = deadband;

   vPID->wp = 0.10;                        /*比例加权系数*/
   vPID->wi = 0.10;                        /*积分加权系数*/
   vPID->wd = 0.10;                        /*微分加权系数*/
}
            
/**
 * @brief        神经网络参数自整定PID控制器，以增量型方式实现
 *               有监督Hebb学习
 * 
 * @param vPID   PID结构体
 * @param pv     测量值
 */
void HebbPID(NEURALPID *vPID,float pv){
   float x[3];
   float w[3];
   float sabs;
   float error;
   float result;
   float deltaResult;

   error = vPID->tar - pv;
   result = vPID->result;
   if(fabs_(error) > vPID->deadband){
      x[1] = error - vPID->e; 
		  x[0] = error;
      x[2] = error - vPID->e*2 + vPID->ee;

      sabs = fabs_(vPID->wi) + fabs_(vPID->wp) + fabs_(vPID->wd);
		  w[1] = vPID->wp/sabs;
      w[0] = vPID->wi/sabs;
      w[2] = vPID->wd/sabs;

      deltaResult = (w[0]*x[0] + w[1]*x[1] + w[2]*x[2])*vPID->kcoef;
   }
   else{
      deltaResult = 0;
   }

   result = result + deltaResult;
   if(result > vPID->maximum){
      result = vPID->maximum;
   }
   if(result < vPID->minimum){
      result = vPID->minimum;
   }
   vPID->result = result;
   vPID->output = (vPID->result - vPID->minimum)*100/(vPID->maximum - vPID->minimum);

   //单神经元学习
   NeureLearningRules(vPID,error,result,x);
   
   vPID->ee = vPID->e;
   vPID->e = error;
}

/**
 * @brief         单神经元学习规则函数
 *                有监督Hebb学习规则
 * @param vPID    PID结构体
 * @param zk      偏差
 * @param uk      当前输出
 * @param xi      
 */
static void NeureLearningRules(NEURALPID *vPID,float zk,float uk,float *xi){
	 vPID->wp = vPID->wp + vPID->kp*zk*uk*xi[1];
   vPID->wi = vPID->wi + vPID->ki*zk*uk*xi[0];
   vPID->wd = vPID->wd + vPID->kd*zk*uk*xi[2];
}


/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static float32_t fabs_(float32_t error){
   return error > 0 ? error : -error;
}