/**
 * @file pid_conf_template.h
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef __PID_CONF_H__ 
#define __PID_CONF_H__  

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* ########################## Module Selection ############################## */
#define PID_ENABLED 

#define PID_STD_ENABLED
// #define PID_FUZZY_ENABLED
// #define PID_HEBB_ENABLED
// #define PID_RBF_ENABLED
// #define PID_EXPERT_ENABLED
// #define PID_TESTBENCH_ENABLED

#ifdef PID_STD_ENABLED
   #include "PID_std.h"
#endif	// PID_STD_ENABLED

#ifdef PID_FUZZY_ENABLED
   #include "PID_fuzzy.h"
#endif	// PID_FUZZY_ENABLED

#ifdef PID_HEBB_ENABLED
   #include "PID_hebb.h"
#endif	// PID_HEBB_ENABLED

#ifdef PID_RBF_ENABLED
   #include "PID_rbf.h"
#endif	// PID_RBF_ENABLED

#ifdef PID_EXPERT_ENABLED
   #include "PID_expert.h"
#endif

#ifdef PID_TESTBENCH_ENABLED
   #include "testbench.h"
#endif
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	// __PID_CONF_TEMPLATE_H__

