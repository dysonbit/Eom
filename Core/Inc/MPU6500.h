#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "main.h" // 通常包含了 stm32f4xx_hal.h 和其他必要头文件
#include <stdio.h> // 如果在 .c 文件中使用了 printf
#include <math.h>  // 添加math.h 以便使用 atan2f 等函数 (虽然主要在c文件中用，但在这里也include一下比较完整)


// MPU6500 I2C 地址 (AD0 引脚接地时为 0x68, AD0 引脚接 VCC 时为 0x69)
// HAL I2C 函数需要7位地址左移一位
#define MPU6500_I2C_ADDR_AD0_LOW  (0x68 << 1)
#define MPU6500_I2C_ADDR_AD0_HIGH (0x69 << 1)

// 选择你的 MPU6500 地址 (根据你硬件上的 AD0 引脚接法)
#define MPU6500_I2C_ADDR MPU6500_I2C_ADDR_AD0_LOW
// #define MPU6500_I2C_ADDR MPU6500_I2C_ADDR_AD0_HIGH


// MPU6500 寄存器地址定义
#define WHO_AM_I_REG    0x75  // WHO_AM_I 寄存器，用于设备识别
#define PWR_MGMT_1_REG  0x6B  // 电源管理寄存器 1
#define CONFIG_REG      0x1A  // 配置寄存器 (DLPF)
#define GYRO_CONFIG_REG 0x1B  // 陀螺仪配置寄存器
#define ACCEL_CONFIG_REG 0x1C // 加速度计配置寄存器

// 添加加速度计和陀螺仪的输出寄存器地址
#define ACCEL_XOUT_H_REG 0x3B
#define ACCEL_XOUT_L_REG 0x3C
#define ACCEL_YOUT_H_REG 0x3D
#define ACCEL_YOUT_L_REG 0x3E
#define ACCEL_ZOUT_H_REG 0x3F
#define ACCEL_ZOUT_L_REG 0x40

#define GYRO_XOUT_H_REG 0x43
#define GYRO_XOUT_L_REG 0x44
#define GYRO_YOUT_H_REG 0x45
#define GYRO_YOUT_L_REG 0x46
// GYRO_ZOUT_H_REG 和 GYRO_ZOUT_L_REG 已有 (0x47, 0x48)


// 陀螺仪和加速度计灵敏度因子 (根据你在 mpu6500_init() 中的设置调整)
// 假设 GYRO_CONFIG 设置为 ±2000 dps (0x18), 对应灵敏度 16.4 LSB/(deg/s)
#define GYRO_SENSITIVITY_2000DPS 16.4f

// 假设 ACCEL_CONFIG 设置为 ±2g (0x00), 对应灵敏度 16384 LSB/g
#define ACCEL_SENSITIVITY_2G  16384.0f
// 如果你在 init 中设置了其他量程, 务必修改这里的定义!
// 例如: ±4g (0x08), 灵敏度 8192 LSB/g -> #define ACCEL_SENSITIVITY_4G  8192.0f


// 函数原型
int mpu6500_init(void); // 返回0表示成功，-1表示失败

// 添加读取原始数据的函数原型
int8_t mpu6500_read_accel_raw(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z);
int8_t mpu6500_read_gyro_raw(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z);

// 原有的函数原型
float mpu6500_read_gyro_z(void); // 可以保留或者后续废弃
uint8_t mpu6500_read_byte(uint8_t reg_addr);
void mpu6500_write_byte(uint8_t reg_addr, uint8_t data);

#endif /* INC_MPU6500_H_ */
