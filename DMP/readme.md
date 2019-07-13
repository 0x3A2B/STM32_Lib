# 适用于MPU9255\MPU9250的DMP库
使用时需要定义宏`MPL_LOG_NDEBUG=1`,`EMPL`,`MPU9250`,`EMPL_TARGET_STM32F4`.
# Usage
## 初始化
```C++
delay_init(216);
i2cinit(&hi2c2);//初始化与MPU9255通信的I2C端口
if (ret = MPU9250_Init())
{
   sprintf(uart_tx, "MPU9255 INIT ERROR:%d\r\n", ret);
   HAL_UART_Transmit(&huart3, uart_tx, strlen(uart_tx), 0x3FFF);
}
else
{
   sprintf(uart_tx, "MPU9255 INIT OK\r\n");
   HAL_UART_Transmit(&huart3, uart_tx, strlen(uart_tx), 0x3FFF);
}
while (ret = mpu_dmp_init())
{
   sprintf(uart_tx, "DMP ERROR : %d\r\n", ret);
   HAL_UART_Transmit(&huart3, uart_tx, strlen(uart_tx), 0x3FFF);
   delay_ms(500);
}
sprintf(uart_tx, "DMP INIT ok\r\n");
HAL_UART_Transmit(&huart3, uart_tx, strlen(uart_tx), 0x3FFF);
```
##从FIFO获取值
```
if (ret = mpu_mpl_get_data(&pitch, &roll, &yaw))
{
   sprintf(uart_tx, "MPU9255 MPL ERROR : %d\r\n", ret);
   HAL_UART_Transmit(&huart3, uart_tx, strlen(uart_tx), 0x3FFF);
}
else
{
   temp = MPU_Get_Temperature();               //得到温度值
   MPU_Get_Accelerometer(&aacx, &aacy, &aacz); //得到加速度传感器数据
   MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);  //得到陀螺仪数据
   // sprintf(uart_tx, "MPU9255 MPL OK p=%d r=%d y=%d t=%d aacx=%d aacy=%d aacz=%d\r\n", pitch * 100, roll * 100, yaw * 100, temp, aacx, aacy, aacz);
   sprintf(uart_tx, "MPU9255 MPL OK:%f,%f,%f\n", pitch, roll, yaw);
   HAL_UART_Transmit(&huart3, uart_tx, strlen(uart_tx), 0x3FFF);
}
```
注意FIFO中的数据会堆积,导致数据不实时.