/**
 * @file PID_fuzzy.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-14
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
 * @brief 
 * 
 * @param pid 
 * @param tar       目标值
 * @param maximum   输出值的上限
 * @param minimum   输出值的下限
 * @param maxdKp    Kp增量的最大限值  
 * @param mindKp    Kp增量的最小限值 
 * @param qKp       Kp增量的影响系数  
 * @param maxdKi    Ki增量的最大限值 
 * @param mindKi    Ki增量的最小限值 
 * @param qKi       Ki增量的影响系数  
 * @param maxdKd    Kd增量的最大限值 
 * @param mindKd    Kd增量的最小限值 
 * @param qKd       Kd增量的影响系数
 */
void FuzzyPIDInit(FUZZYPID *pid, float32_t tar,float32_t maximum, float32_t minimum,\
                  float32_t maxdKp, float32_t mindKp, float32_t qKp, \
                  float32_t maxdKi, float32_t mindKi, float32_t qKi, \
                  float32_t maxdKd, float32_t mindKd, float32_t qKd){
   pid->tar      = tar;
   pid->maxdKp   = maxdKp;   
   pid->mindKp   = mindKp;   
   pid->qKp      = qKp;      
   pid->maxdKi   = maxdKi;   
   pid->mindKi   = mindKi;   
   pid->qKi      = qKi;      
   pid->maxdKd   = maxdKd;   
   pid->mindKd   = mindKd;   
   pid->qKd      = qKd;   
	pid->maximum  = maximum;
   pid->minimum  = minimum;							
}
/**
 * @brief 模糊PID
 * 
 * @param pid PID结构体
 * @param cur 当前值
 */
void FuzzyPID(FUZZYPID *pid, float32_t cur){
   float32_t deltaK[3];               //模糊PID结果

   FuzzyComputation(pid, deltaK, cur);
   //FuzzyPIDset(pid, deltaK);

   //pid->output = pid->kp*pid->e + pid->ki*pid->sume+ pid->kd*(pid->e - pid->ee);
	pid->output = (pid->kp + deltaK[0]) * pid->e + (pid->ki + deltaK[1])*pid->sume+ (pid->kd + deltaK[2])*(pid->e - pid->ee);
	
   pid->sume += pid->e;
   pid->ee = pid->e;
	pid->e = pid->tar - cur;
}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief         线性量化操作函数
 *                论域{-3，-2，-1，0，1，2，3}
 * @param vPID    输入PID结构体
 * @param pv      当前值
 * @param qValue  偏差 及其 增量的 量化值
 */
static void LinearQuantization(FUZZYPID *vPID, float32_t *qValue, float32_t pv){
   float32_t thisError;
   float32_t deltaError;
 
   thisError=vPID->tar - pv;                  //计算偏差值
   deltaError=thisError - vPID->e;            //计算偏差增量

   qValue[0]=3.0*thisError/(vPID->maximum-vPID->minimum);
   qValue[1]=1.5*deltaError/(vPID->maximum-vPID->minimum);

}

/**
 * @brief         隶属度计算函数
 * 
 * @param ms      隶属度
 * @param index   隶属度索引
 * @param qv      量化值
 */
static void CalcMembership(float32_t *ms, int32_t * index, float32_t qv){

	float32_t temp;
   if((qv>=NL)&&(qv<NM)){
#ifdef zmf
	 #define a1 -3.0F
	 #define b1 -2.0F
#endif
      index[0]=0;
      index[1]=1;
#ifdef zmf
		  temp = (qv-a1)/(b1-a1);
		  temp *= temp;
		  temp *=2;
		  ms[0] = qv > -2.5? temp:1-temp;
#else
      ms[0]=-qv-2.0;  //y=-x-2.0
#endif
      ms[1]=qv+3.0;   //y=x+3.0
   }
   else if((qv>=NM)&&(qv<NS)){
      index[0]=1;
      index[1]=2;
      ms[0]=-qv-1.0;  //y=-x-1.0
      ms[1]=qv+2.0;   //y=x+2.0
   }
   else if((qv>=NS)&&(qv<ZE)){
      index[0]=2;
      index[1]=3;
      ms[0]=-qv;      //y=-x
      ms[1]=qv+1.0;   //y=x+1.0
   }
   else if((qv>=ZE)&&(qv<PS)){
      index[0]=3;
      index[1]=4;
      ms[0]=-qv+1.0;  //y=-x+1.0
      ms[1]=qv;       //y=x
   }
   else if((qv>=PS)&&(qv<PM)){
      index[0]=4;
      index[1]=5;
      ms[0]=-qv+2.0;  //y=-x+2.0
      ms[1]=qv-1.0;   //y=x-1.0
   }
   else if((qv>=PM)&&(qv<=PL)){
#ifdef smf
	 #define a2 2.0F
	 #define b2 3.0F
#endif
      index[0]=5;
      index[1]=6;
#ifdef smf
		  temp = (qv-a2)/(b2-a2);
		  temp *= temp;
		  temp *=2;
		  ms[0] = qv < 2.5? temp:1-temp;
#else
      ms[0]=-qv+3.0;  //y=-x+3.0
#endif
      ms[1]=qv-2.0;   //y=x-2.0
   }
}

/**
 * @brief         模糊PID计算
 * 
 * @param vPID    输入PID结构体
 * @param deltaK  输出模糊结果
 * @param pv      当前值
 */
static void FuzzyComputation (FUZZYPID *vPID, float *deltaK, float pv){
   float32_t qValue[2]={0,0};        //偏差 及其 增量的 量化值
   int32_t indexE[2]={0,0};          //偏差隶属度索引
   float32_t msE[2]={0,0};           //偏差隶属度
   int32_t indexEC[2]={0,0};         //偏差增量隶属度索引
   float32_t msEC[2]={0,0};          //偏差增量隶属度
   float32_t qValueK[3];             //解模糊结果
   LinearQuantization(vPID, qValue, pv);
   CalcMembership(msE, indexE, qValue[0]);
   CalcMembership(msEC, indexEC, qValue[1]);

   qValueK[0]=msE[0] * (msEC[0] * ruleKp[ indexE[0] ][ indexEC[0] ] + msEC[1] * ruleKp[ indexE[0] ] [ indexEC[1] ])\
             +msE[1] * (msEC[0] * ruleKp[ indexE[1] ][ indexEC[0] ] + msEC[1] * ruleKp[ indexE[1] ] [ indexEC[1] ]);
   qValueK[1]=msE[0] * (msEC[0] * ruleKi[ indexE[0] ][ indexEC[0] ] + msEC[1] * ruleKi[ indexE[0] ] [ indexEC[1] ])\
             +msE[1] * (msEC[0] * ruleKi[ indexE[1] ][ indexEC[0] ] + msEC[1] * ruleKi[ indexE[1] ] [ indexEC[1] ]);
   qValueK[2]=msE[0] * (msEC[0] * ruleKd[ indexE[0] ][ indexEC[0] ] + msEC[1] * ruleKd[ indexE[0] ] [ indexEC[1] ])\
             +msE[1] * (msEC[0] * ruleKd[ indexE[1] ][ indexEC[0] ] + msEC[1] * ruleKd[ indexE[1] ] [ indexEC[1] ]);
   deltaK[0] = LinearRealization(vPID->maxdKp, vPID->mindKp, qValueK[0]);
   deltaK[1] = LinearRealization(vPID->maxdKi, vPID->mindKi, qValueK[1]);
   deltaK[2] = LinearRealization(vPID->maxdKd, vPID->mindKd, qValueK[2]);
}

/**
 * @brief        调节PID参数
 * 
 * @param pid    输入pid结构体
 * @param deltaK 输入调节值
 */
inline void FuzzyPIDset(FUZZYPID *pid, float32_t *deltaK){
   pid->kp += deltaK[0] * pid->qKp;
   pid->ki += deltaK[1] * pid->qKi;
   pid->kd += deltaK[2] * pid->qKd;
}

/**
 * @brief       防过调
 * 
 * @param max   调节最大值
 * @param min   调节最小值
 * @param val   当前值
 * @return float32_t 
 */
float32_t LinearRealization(float32_t max, float32_t min, float32_t val){
   if(val > max) {
      return max;
   }
   else if(val < min) {
      return min;
   }
   else {
      return val;
   }
}
