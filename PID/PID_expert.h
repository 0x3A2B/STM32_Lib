/**
 * @file PID_expert.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __PID_EXPERT_H__ 
#define __PID_EXPERT_H__ 

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
   float tar;                    /*设定值*/
   float kp;                     /*比例系数*/
   float ki;                     /*积分系数*/
   float kd;                     /*微分系数*/

   float lasterror;              /*前一拍偏差*/
   float preerror;               /*前两拍偏差*/
   float result;                 /*PID控制器结果*/
   float output;                 /*输出值，0-100，为百分比值*/
   float maximum;                /*输出值上限*/
   float minimum;                /*输出值下限*/
   
   float errorabsmax;            /*偏差绝对值最大值*/
   float errorabsmid;            /*偏差绝对值中位值*/
   float errorabsmin;            /*偏差绝对值最小值*/
}EXPERTPID;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void ExpertPIDInit(EXPERTPID *vPID, float tar, float max, float min, float errabsmax, float errabsmid, float errabsmin, float kp, float ki,float kd);
void ExpertPID(EXPERTPID *vPID, float pv);
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif	// __PID_EXPERT_H__