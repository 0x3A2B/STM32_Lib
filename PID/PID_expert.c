/**
 * @file PID_expert.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-17
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
 * @param vPID 
 * @param tar 
 * @param max 
 * @param min 
 * @param errabsmax 
 * @param errabsmid 
 * @param errabsmin 
 * @param kp 
 * @param ki 
 * @param kd 
 */
void ExpertPIDInit(EXPERTPID *vPID, float tar, float max, float min, float errabsmax, float errabsmid, float errabsmin, float kp, float ki,float kd){
   vPID->tar = tar;                          /*设定值*/
   vPID->maximum = max;                      /*输出值上限*/
   vPID->minimum = min;                      /*输出值下限*/
   vPID->errorabsmax = errabsmax;            /*偏差绝对值最大值*/ 
   vPID->errorabsmid = errabsmid;            /*偏差绝对值中位值*/
   vPID->errorabsmin = errabsmin;            /*偏差绝对值最小值*/
   vPID->lasterror = 0;
   vPID->preerror = 0;
   vPID->result = 0;
   vPID->kp = kp;
   vPID->ki = ki;
   vPID->kd = kd;
}
/**
 * @brief 
 * 
 * @param vPID 
 * @param pv 
 */
void ExpertPID(EXPERTPID *vPID, float pv){
   float thiserror;
   float deltaerror;
   float lastdeltaerror;
   float result;//本次调节输出值

   thiserror = vPID->tar - pv;
   deltaerror = thiserror - vPID->lasterror;
   lastdeltaerror = vPID->lasterror - vPID->preerror;

   if(fabs_pid(thiserror) >= vPID->errorabsmax){/*执行规则1*/
      if(thiserror>0){
         result = vPID->maximum;
      }
      if(thiserror<0){
         result = vPID->minimum;
      }
   }

   if((thiserror * deltaerror > 0)||(deltaerror == 0)){/*执行规则2*/
      if(fabs_pid(thiserror) >= vPID->errorabsmid){
         result = vPID->result + 2.0 * (vPID->kp * deltaerror + vPID->ki * thiserror + vPID->kd * (deltaerror - lastdeltaerror));
      }
      else{
         result = vPID->result + 0.4 * (vPID->kp * deltaerror + vPID->ki * thiserror + vPID->kd * (deltaerror-lastdeltaerror));
      }
   }

   if(((thiserror * deltaerror < 0)&&(deltaerror * lastdeltaerror > 0))||(fabs_pid(thiserror) < 0.0001)){/*执行规则3*/
      result = vPID->kp * thiserror + vPID->ki * 0+ vPID->kd * deltaerror ;
   }

   if((thiserror * deltaerror < 0)&&(deltaerror * lastdeltaerror < 0)){/*执行规则4*/
      if(fabs_pid(thiserror)>=vPID->errorabsmid){
         result = vPID->result + 2.0 * vPID->kp * thiserror;
      }
      else{
         result = vPID->result + 0.6 * vPID->kp * thiserror;
      }
   }

   if((fabs_pid(thiserror) <= vPID->errorabsmin)&&(fabs_pid(thiserror) > 0)){/*执行规则5*/
      //result = vPID->result + 0.5 * vPID->kp * deltaerror + 0.3 * vPID->ki * thiserror;
      result = vPID->kp * deltaerror + vPID->ki * thiserror + vPID->kd * (deltaerror - lastdeltaerror);
   }

   /*对输出限值，避免超调*/
   if(result >= vPID->maximum){
      result = vPID->maximum;
   }
   if(result <= vPID->minimum){
      result = vPID->minimum;
   }

   vPID->result = result;
   vPID->preerror = vPID->lasterror;
   vPID->lasterror = thiserror;
   vPID->output = (result/(vPID->maximum - vPID->minimum))*100;
}
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/