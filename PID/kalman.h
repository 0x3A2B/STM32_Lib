/**
 * @file kalman.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __KALMAN_H__ 
#define __KALMAN_H__ 
#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//标量卡尔曼滤波
typedef struct {
    float x;     /*系统的状态量*/
    float A;     /*x(n)=A*x(n-1)+u(n),u(n)~N(0,q)*/
    float H;     /*z(n)=H*x(n)+w(n),w(n)~N(0,r)*/
    float q;     /*预测过程噪声协方差*/
    float r;     /*测量过程噪声协方差*/
    float p;     /*估计误差协方差*/
    float gain;  /*卡尔曼增益*/
}KalmanFilter;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void kalmanFilter_init(KalmanFilter *kalmanFilter, float init_x, float init_p,float predict_q,float newMeasured_q);
float kalmanFilter_filter(KalmanFilter *kalmanFilter, float newMeasured);
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif	// __KALMAN_H__