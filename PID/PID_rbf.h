/**
 * @file PID_rbf.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __PID_RBF_H__ 
#define __PID_RBF_H__ 

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/

#define INPUT_NODE 3
#define HIDE_NODE 6
#define OUTPUT_NODE 1

typedef struct 
{
   float32_t tar;                         /*设定值*/
   
   float32_t krbf;                        /*网络学习速度*/
   float32_t kp;                          /*比例学习速度*/
   float32_t ki;                          /*积分学习速度*/
   float32_t kd;                          /*微分学习速度*/
   float32_t alfa;                        /*动量因子*/
   
   float32_t c[INPUT_NODE][HIDE_NODE];    /*节点基函数中心向量*/
   float32_t cc[INPUT_NODE][HIDE_NODE];   /*上次节点基函数中心向量*/
   
   float32_t b[HIDE_NODE];                /*节点基函数方差*/
   float32_t bb[HIDE_NODE];               /*上次节点基函数方差*/
   
   float32_t w[HIDE_NODE];                /*输出层权重*/
   float32_t ww[HIDE_NODE];               /*上次输出层权重*/
   
   float32_t x[INPUT_NODE];               /*神经网络输入*/
   
   float32_t ppv;                         /*上次测量结果*/
   float32_t e;                           /*前一拍偏差*/
   float32_t ee;                          /*前两拍偏差*/
   float32_t deadband;                    /*死区*/
   float32_t result;                      /*输出值*/
   float32_t presult;                     /*上次调节结果*/
   float32_t output;                      /*百分比输出值*/
   float32_t maximum;                     /*输出值的上限*/
   float32_t minimum;                     /*输出值的下限*/
   
   float32_t wp;                          /*比例加权系数*/
   float32_t wi;                          /*积分加权系数*/
   float32_t wd;                          /*微分加权系数*/

}RBFPID;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void RbfPIDInit(RBFPID *vpid, float32_t tar, float32_t kp, float32_t ki, float32_t kd);
void RBF(RBFPID *vpid, float32_t pv);
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void net_cal(RBFPID *vpid, float32_t *jacobian, float32_t pv);

#ifdef __cplusplus
}
#endif

#endif	// __PID_RBF_H__