/**
 * @file fuzzy.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __PID_FUZZY_H__ 
#define __PID_FUZZY_H__ 

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported types ------------------------------------------------------------*/

typedef struct{
   float32_t tar;       /**< The target value 目标值*/
   float32_t kp;        /**< The proportional gain. */
   float32_t ki;        /**< The integral gain. */
   float32_t kd;        /**< The derivative gain. */

   float32_t e;         /*前一拍偏差*/
   float32_t ee;        /*前两拍偏差*/
   float32_t sume;      /*前两拍偏差*/
   float32_t deadband;  /*死区*/
   float32_t output;    /*输出值*/
   float32_t result;    /*物理量输出值*/
   float32_t maximum;   /*输出值的上限*/
   float32_t minimum;   /*输出值的下限*/

   float32_t maxdKp;    /*Kp增量的最大限值*/
   float32_t mindKp;    /*Kp增量的最小限值*/
   float32_t qKp;       /*Kp增量的影响系数*/
   float32_t maxdKi;    /*Ki增量的最大限值*/
   float32_t mindKi;    /*Ki增量的最小限值*/
   float32_t qKi;       /*Ki增量的影响系数*/
   float32_t maxdKd;    /*Kd增量的最大限值*/
   float32_t mindKd;    /*Kd增量的最小限值*/
   float32_t qKd;       /*Kd增量的影响系数*/
} FUZZYPID;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//#define zmf 1
//#define smf 1
/* Exported functions --------------------------------------------------------*/
//fuzzy pid
void FuzzyPIDInit(FUZZYPID *pid, float32_t tar,float32_t maximum, float32_t minimum,\
                  float32_t maxdKp, float32_t mindKp, float32_t qKp, \
                  float32_t maxdKi, float32_t mindKi, float32_t qKi, \
                  float32_t maxdKd, float32_t mindKd, float32_t qKd);
void FuzzyPID(FUZZYPID *pid, float32_t cur);

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3
 
/*******************************************
 * table format
 * 
 *  --------------------E-------------------
 *  | Kp    NL   NM   NS   ZE   PS  PM    PL
 *  |    -----------------------------------
 *  | NL || PL | PL | PM | PM | PS | PS | ZE
 *  | NM || PL | PL | PM | PM | PS | ZE | ZE
 *  | NS || PM | PM | PM | PS | ZE | NS | NM
 * EC ZE || PM | PS | PS | ZE | NS | NM | NM
 *  | PS || PS | PS | ZE | NS | NS | NM | NM
 *  | PM || ZE | ZE | NS | NM | NM | NM | NL
 *  | PL || ZE | NS | NS | NM | NM | NL | NL
 * 
 *  --------------------E-------------------
 *  | Ki    NL   NM   NS   ZE   PS  PM    PL
 *  |    -----------------------------------
 *  | NL || NL | NL | NL | NM | NM | ZE | ZE
 *  | NM || NL | NL | NM | NM | NS | ZE | ZE
 *  | NS || NM | NM | NS | NS | ZE | PS | PS
 * EC ZE || NM | NS | NS | ZE | PS | PS | PM
 *  | PS || NS | NS | ZE | PS | PS | PM | PM
 *  | PM || ZE | ZE | PS | PM | PM | PL | PL
 *  | PL || ZE | ZE | PS | PM | PL | PL | PL
 * 
 *  --------------------E-------------------
 *  | Kd    NL   NM   NS   ZE   PS  PM    PL
 *  |    -----------------------------------
 *  | NL || PS | PS | ZE | ZE | ZE | PL | PL
 *  | NM || NS | NS | NS | NS | ZE | NS | PM
 *  | NS || NL | NL | NM | NS | ZE | PS | PM
 * EC ZE || NL | NM | NM | NS | ZE | PS | PM
 *  | PS || NL | NM | NS | NS | ZE | PS | PS
 *  | PM || NM | NS | NS | NS | ZE | PS | PS
 *  | PL || PS | ZE | ZE | ZE | ZE | PL | PL
 * 
 */
static const float ruleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	PS,	ZE,
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PM,	PM,	PM,	PS,	ZE,	NS,	NM,
	PM,	PS,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	ZE,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	NS,	NS,	NM,	NM,	NL,	NL 
};
 
static const float ruleKi[7][7]={
	NL,	NL,	NL,	NM,	NM,	ZE,	ZE,
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NM,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	ZE,	PS,	PS,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PM,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PL,	PL,	PL
};
 
static const float ruleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};

/* Private functions ---------------------------------------------------------*/
static void LinearQuantization(FUZZYPID *vPID, float32_t *qValue, float32_t pv);
static void CalcMembership(float32_t *ms, int32_t * index, float32_t qv);
static void FuzzyComputation (FUZZYPID *vPID, float32_t *deltaK, float32_t pv);
inline void FuzzyPIDset(FUZZYPID *pid, float32_t *deltaK);
float32_t LinearRealization(float32_t max, float32_t min, float32_t val);


#ifdef __cplusplus
}
#endif

#endif	// __PID_FUZZY_H__
