/**
 * @file i2c.c
 * @author AnQi Wang (waq5858@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-03-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
/* Exported types ------------------------------------------------------------*/
I2C_HandleTypeDef *mpu_i2c;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int Sensors_I2C_Init(I2C_HandleTypeDef *i2cdevice){
   mpu_i2c = i2cdevice;
	return 0;
}
/**
 * @brief  写寄存器，这是提供给上层的接口
 * @param  slave_addr: 从机地址
 * @param 	reg_addr:寄存器地址
 * @param len：写入的长度
 *	@param data_ptr:指向要写入的数据
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len, 
                              const unsigned char *data_ptr){
   int ret = 0;
   HAL_I2C_Mem_Write(mpu_i2c, slave_addr<<1, reg_addr, sizeof(reg_addr), (uint8_t *)data_ptr, len, 0x3f3f3f3f);
   return ret;
}
/**
 * @brief  读寄存器，这是提供给上层的接口
 * @param  slave_addr: 从机地址
 * @param 	reg_addr:寄存器地址
 * @param len：要读取的长度
 *	@param data_ptr:指向要存储数据的指针
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                             unsigned char reg_addr,
                             unsigned short len, 
                             unsigned char *data_ptr)
{
   int ret = 0;
   ret = HAL_I2C_Mem_Read(mpu_i2c, slave_addr<<1, reg_addr, sizeof(reg_addr), (uint8_t *)data_ptr, len, 0x3f3f3f);
   return ret;
}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/