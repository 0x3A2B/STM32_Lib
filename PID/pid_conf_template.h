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
#ifndef __PID_CONF_TEMPLATE_H__ 
#define __PID_CONF_TEMPLATE_H__  

#ifdef __cplusplus
   extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* ########################## Module Selection ############################## */
#define PID_MODULE_ENABLED 

//#define PID_STDPID_ENABLED
//#define PID_FUZZY_ENABLED
//#define PID_HEBB_ENABLED

#ifdef PID_STDPID_ENABLED
#define PID_STDPID_ENABLED 
#include "stdPID.h"
#endif	// PID_STDPID_ENABLED

#ifdef PID_FUZZY_ENABLED
#define PID_FUZZY_ENABLED 
#include "fuzzyPID.h"
#endif	// PID_FUZZY_ENABLED

#ifdef PID_HEBB_ENABLED
#define PID_HEBB_ENABLED 
#include "HebbPID.h"
#endif	// PID_HABB_ENABLED

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	// __PID_CONF_TEMPLATE_H__