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
#define PID_ENABLED 

//#define PID_STD_ENABLED
//#define PID_FUZZY_ENABLED
//#define PID_HEBB_ENABLED
#define PID_RBF_ENABLED

#ifdef PID_STDPID_ENABLED
#define PID_STDPID_ENABLED 
#include "PID_std.h"
#endif	// PID_STDPID_ENABLED

#ifdef PID_FUZZYPID_ENABLED
#define PID_FUZZYPID_ENABLED 
#include "PID_fuzzy.h"
#endif	// PID_FUZZYPID_ENABLED

#ifdef PID_HEBBPID_ENABLED
#define PID_HEBBPID_ENABLED 
#include "PID_hebb.h"
#endif	// PID_HABBPID_ENABLED

#ifndef PID_RBF_ENABLED
#define PID_RBF_ENABLED
#include "PID_rbf.h"
#endif	// PID_RBF_ENABLED

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	// __PID_CONF_TEMPLATE_H__