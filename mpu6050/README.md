# mpu6050
# easy
## Description
基本的MPU6050库, 不含有DMP.
## Usage
初始化: `MPU6050_Init(&hi2c1);`

读取角加速度: `void MPU6050ReadGyro(short *gyroData);`

读取加速度原始值: `MPU6050ReadAcc(short *accData);`

读取原始温度值: `MPU6050ReadTemp(short *tempData);`, 摄氏度: ` MPU6050_ReturnTemp(float *Temperature);`

## Reference
...
# adv_with_dmp
## Description
含有DMP的MPU6050库.
## Usage
初始化: `u8 MPU_Init(I2C_HandleTypeDef *i2c); `

读取角加速度: `MPU_Get_Gyroscope(short *gx,short *gy,short *gz);`

读取加速度原始值: `MPU_Get_Accelerometer(short *ax,short *ay,short *az);`

读取原始温度值: `MPU6050ReadTemp(short *tempData)`, 摄氏度: `MPU_Get_Temperature(void);`

## Reference

