/**
 * @file bsp_mpu_exti.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-03-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
  
#include "bsp_mpu_exti.h"

 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  /**标准库配置
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Configure one bit for preemption priority 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  // 配置中断源 
  NVIC_InitStructure.NVIC_IRQChannel = MPU_INT_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  **/

  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MPU_INT_EXTI_IRQ, 1, 1);
  HAL_NVIC_EnableIRQ(MPU_INT_EXTI_IRQ);

}

 /**
  * @brief  配置中断口，并设置中断优先级
  * @param  无
  * @retval 无
  */
void EXTI_MPU_Config(void)
{ 
  GPIO_InitTypeDef   GPIO_InitStructure;
	/*开启按键GPIO口的时钟*/
  __HAL_RCC_GPIOI_CLK_ENABLE();      //不同接线需要修改
  
  /* 使能 SYSCFG 时钟 ，使用GPIO外部中断时必须使能SYSCFG时钟*/
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = MPU_INT_GPIO_PIN;
  HAL_GPIO_Init(MPU_INT_GPIO_PORT, &GPIO_InitStructure);

  /* 配置 NVIC */
  NVIC_Configuration();


}
/*********************************************END OF FILE**********************/
