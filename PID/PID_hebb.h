/**
 * @file PID_hebb.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-15
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __PID_HEBB_H__ 
#define __PID_HEBB_H__ 

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float tar;                    /*设定值*/
	
  float kcoef;                  /*神经元输出比例*/
  float kp;                     /*比例学习速度*/
  float ki;                     /*积分学习速度*/
  float kd;                     /*微分学习速度*/
	
  float e;                      /*前一拍偏差*/
  float ee;                     /*前两拍偏差*/
  float deadband;               /*死区*/
  float result;                 /*输出值*/
  float output;                 /*百分比输出值*/
  float maximum;                /*输出值的上限*/
  float minimum;                /*输出值的下限*/
	
  float wp;                     /*比例加权系数*/
  float wi;                     /*积分加权系数*/
  float wd;                     /*微分加权系数*/
}HEBBPID;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void HebbPIDInit(HEBBPID *vPID, float32_t tar, float vMax,float vMin, float deadband);
void HebbPID(HEBBPID *vPID,float32_t pv);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void NeureLearningRules(HEBBPID *vPID,float32_t zk,float32_t uk,float32_t *xi);
#ifdef __cplusplus
}
#endif

#endif	// __PID_HEBB_H__