/**
 * @file PID_rbf.c
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
#include "math.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
//#define DEBUG
/**
 * @brief RBF_PID神经PID自整定初始化
 * 
 * @param vpid  pid结构体
 * @param tar 目标值
 * @param kp 
 * @param ki 
 * @param kd 
 */
void RbfPIDInit(RBFPID *vpid, float32_t tar, float32_t kp, float32_t ki, float32_t kd){
   vpid->tar = tar;            /*设定值*/
   vpid->krbf = 0.5;           /*网络学习速度*/
   vpid->wp = 0.15;            /*比例初值*/
   vpid->wi = 0.15;            /*积分初值*/
   vpid->wd = 0.15;            /*微分初值*/
   vpid->kp = 1;               /*比例权重*/
   vpid->ki = 1;               /*积分权重*/
   vpid->kd = 1;               /*微分权重*/
   vpid->alfa = 0.05;          /*动量因子*/


   int i, j;
   for(i = 0; i < INPUT_NODE; i++){
      for(j = 0; j < HIDE_NODE; j++){
         vpid->c[i][j] = 0;     /*节点基函数中心向量*/
         //vpid->cc[i][j] = 0;    /*节点基函数中心向量*/
      }
      vpid->x[i] = 0;           /*神经网络输入*/
   }
   for(i = 0; i < HIDE_NODE; i++){
      vpid->w[i] = 0.1;           /*输出层权重*/
      vpid->b[i] = 10;            /*节点基函数方差*/
      //vpid->ww[i] = 0.1;          /*输出层权重*/
      //vpid->bb[i] = 10;           /*节点基函数方差*/
   }

}
/**
 * @brief RBF_PID算法
 * 
 * @param vpid PID结构体
 * @param pv 测量值
 */
void RBF(RBFPID *vpid, float32_t pv){
   float32_t err;            /*当前误差*/
   float32_t error[3];       /*误差输入*/
   float32_t result;         /*本次结果*/
   float32_t jacobian;       /*雅各比辨识结果*/
   int i, j;

   err = vpid->tar - pv;

   vpid->x[0] = vpid->presult;
   vpid->x[1] = pv;
   vpid->x[2] = vpid->ppv;
   vpid->ppv = pv;

   net_cal(vpid, &jacobian, pv);

   error[1] = err - vpid->e; 
   error[0] = err;
   error[2] = err - vpid->e*2 + vpid->ee;

   vpid->wp += vpid->kp * err * jacobian * error[1];
   vpid->wi += vpid->ki * err * jacobian * error[0];
   vpid->wd += vpid->kd * err * jacobian * error[2];

   result = vpid->wp * error[1] + vpid->wi * error[0] + vpid->wd * error[2];
   vpid->presult = result;
   vpid->result += result;

   vpid->ee = vpid->e;
   vpid->e = err;
}

/* Private types -------------------------------------------------------------*/ 
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief RBF网络计算, 雅各比行列式计算
 * 
 * @param vpid PID结构体
 * @param jacobian 输出雅各比值 
 * @param pv 当前测量值
 */
void net_cal(RBFPID *vpid, float32_t *jacobian, float32_t pv){
   float32_t net;                               /*辨识网络输出*/
   float32_t h[HIDE_NODE] = {0};                /*隐藏层输出向量*/
   float32_t dw[HIDE_NODE] = {0};               /*权值更新矩阵*/
   float32_t db[HIDE_NODE] = {0};               /*基向量更新矩阵*/
   float32_t dc[INPUT_NODE][HIDE_NODE] = {0};   /*节点中心更新矩阵*/
   float32_t wk[HIDE_NODE] = {0};               /*当前权值矩阵*/
   float32_t bk[HIDE_NODE] = {0};               /*当前基向量矩阵*/
   float32_t ck[INPUT_NODE][HIDE_NODE] = {0};   /*当前节点中心矩阵*/
   float32_t temp;

   int i, j, k, l;

#ifdef ARM_MATH_CM4 
/****************************RBF Net compulet**********************/
   for(i = 0; i < HIDE_NODE;i++){
      temp = 0;
      for(j = 0; j <INPUT_NODE; j++){
         temp += (vpid->x[j] - vpid->c[j][i]) * (vpid->x[j] - vpid->c[j][i]);
      }
      h[i] = exp( -temp / (2 * vpid->b[i] * vpid->b[i]) );       /*RBF节点输出*/
   }

   net = 0;
   for(i = 0; i < HIDE_NODE;i++){
      net += h[i] * vpid->w[i];
   }

/**************************GradientDescent************************/
//输出权重更新
	for(i = 0; i < HIDE_NODE; i++){
       dw[i] = vpid->krbf * (pv - net) * h[i];
   }

   for(i = 0; i < HIDE_NODE; i++){
	   wk[i] = vpid->w[i] + dw[i] + vpid->alfa * (vpid->w[i] - vpid->ww[i]);
   }

//节点基函数方差更新
   for(i = 0; i < HIDE_NODE; i++){
      temp = 0;
      for(j = 0; j <INPUT_NODE; j++){
         temp += (vpid->x[j] - vpid->c[j][i]) * (vpid->x[j] - vpid->c[j][i]);
      }
      db[i] = vpid->krbf * (pv - net) * vpid->w[i] * h[i] * temp / (vpid->b[i] * vpid->b[i] * vpid->b[i]);
   }

   for(i = 0; i < HIDE_NODE; i++){
	   bk[i] = vpid->b[i] + db[i] + vpid->alfa * (vpid->b[i] - vpid->bb[i]);
   }

//节点中心更新
   for(i = 0; i < HIDE_NODE; i++ ){
      for(j = 0; j < INPUT_NODE; j++){
            dc[i][j] = vpid->krbf * (pv - net) * vpid->w[i] * h[i] * (vpid->x[i] - vpid->c[i][j]) / (vpid->b[j] * vpid->b[j]);
            //dc[j][i] = vpid->krbf * (pv - net) * vpid->w[i] * (vpid->x[i] - vpid->c[j][i]) / (vpid->b[j] * vpid->b[j]);
      }
   }

   for(i = 0; i < HIDE_NODE; i++ ){
      for(j = 0; j < INPUT_NODE; j++){
            ck[j][i] = vpid->c[j][i] + dc[j][i] + vpid->alfa * (vpid->c[j][i] - vpid->cc[j][i]);
      }
   }

/******************************Jacobian***************************/
	(*jacobian) = 0;
	for(i = 0; i < HIDE_NODE; i++){
      (*jacobian) += wk[i] * h[i] * (ck[0][i] - vpid->x[0]) / (bk[i] * bk[i]);
   }
	 
#else

/****************************RBF Net compulet**********************/
   for(i = 0; i < HIDE_NODE;i++){
      temp = 0;
      for(j = 0; j <INPUT_NODE; j++){
         temp += (vpid->x[j] - vpid->c[j][i]) * (vpid->x[j] - vpid->c[j][i]);
      }
      h[i] = exp( -temp / (2 * vpid->b[i] * vpid->b[i]) );       /*RBF节点输出*/
   }

   net = 0;
   for(i = 0; i < HIDE_NODE;i++){
      net += h[i] * vpid->w[i];
   }

/**************************GradientDescent************************/
//输出权重更新
	for(i = 0; i < HIDE_NODE; i++){
       dw[i] = vpid->krbf * (pv - net) * h[i];
   }

   for(i = 0; i < HIDE_NODE; i++){
	   wk[i] = vpid->w[i] + dw[i] + vpid->alfa * (vpid->w[i] - vpid->ww[i]);
   }

//节点基函数方差更新
   for(i = 0; i < HIDE_NODE; i++){
      temp = 0;
      for(j = 0; j <INPUT_NODE; j++){
         temp += (vpid->x[j] - vpid->c[j][i]) * (vpid->x[j] - vpid->c[j][i]);
      }
      db[i] = vpid->krbf * (pv - net) * vpid->w[i] * h[i] * temp / (vpid->b[i] * vpid->b[i] * vpid->b[i]);
   }

   for(i = 0; i < HIDE_NODE; i++){
	   bk[i] = vpid->b[i] + db[i] + vpid->alfa * (vpid->b[i] - vpid->bb[i]);
   }

//节点中心更新
   for(i = 0; i < HIDE_NODE; i++ ){
      for(j = 0; j < INPUT_NODE; j++){
            dc[i][j] = vpid->krbf * (pv - net) * vpid->w[i] * h[i] * (vpid->x[i] - vpid->c[i][j]) / (vpid->b[j] * vpid->b[j]);
            //dc[j][i] = vpid->krbf * (pv - net) * vpid->w[i] * (vpid->x[i] - vpid->c[j][i]) / (vpid->b[j] * vpid->b[j]);
      }
   }

   for(i = 0; i < HIDE_NODE; i++ ){
      for(j = 0; j < INPUT_NODE; j++){
            ck[j][i] = vpid->c[j][i] + dc[j][i] + vpid->alfa * (vpid->c[j][i] - vpid->cc[j][i]);
      }
   }

/******************************Jacobian***************************/
	(*jacobian) = 0;
	for(i = 0; i < HIDE_NODE; i++){
      (*jacobian) += wk[i] * h[i] * (ck[0][i] - vpid->x[0]) / (bk[i] * bk[i]);
   }

#endif 

#ifdef DEBUG
   //int k = 0;
	printf("net:%f", net);
	printf("\nh: \n");
	for(k = 0 ; k < HIDE_NODE;k++){
		printf("%f ", h[k]);
	}
	printf("\ndw: \n");
	for(k = 0 ; k < HIDE_NODE;k++){
		printf("%f ", dw[k]);
	}
	printf("\ndb: \n");
	for(k = 0 ; k < HIDE_NODE;k++){
	printf("%f ", db[k]);
	}
	printf("\nwk: \n");
	for(k = 0 ; k < HIDE_NODE;k++){
		printf("%f ", wk[k]);
	}
	printf("\nbk: \n");
	for(k = 0 ; k < HIDE_NODE;k++){
		printf("%f ", bk[k]);
	}
	printf("\ndc: \n");
	for(k = 0; k < HIDE_NODE; k++ ){
      for(l = 0; l < INPUT_NODE; l++){
            printf("%f ", dc[l][k]);
      }
      printf("\n");
   }
   printf("ck: \n");
	for(k = 0; k < HIDE_NODE; k++ ){
      for(l = 0; l < INPUT_NODE; l++){
            printf("%f ", ck[l][k]);
      }
      printf("\n");
   }
   printf("jacobian: %f\n", (*jacobian));
#endif
//传递下一次的值
   for(i = 0; i<HIDE_NODE; i++ ){
      for(j = 0; j < INPUT_NODE; j++){
         vpid->cc[j][i] = vpid->c[j][i];      /*上上次节点基函数中心向量*/
      }
      vpid->bb[i] = vpid->b[i];               /*上上次节点基函数方差*/
      vpid->ww[i] = vpid->w[i];               /*上上次输出层权重*/
   }
   for(i = 0; i<HIDE_NODE; i++ ){
      for(j = 0; j < INPUT_NODE; j++){
         vpid->c[j][i] = ck[j][i];      /*上次节点基函数中心向量*/
      }
      vpid->b[i] = bk[i];               /*上次节点基函数方差*/
      vpid->w[i] = wk[i];               /*上次输出层权重*/
   }
}