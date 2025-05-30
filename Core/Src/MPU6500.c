#include "mpu6500.h"

// 声明外部 I2C2 句柄 (应在 main.c 中定义并通过 CubeMX 初始化)
extern I2C_HandleTypeDef hi2c2;


// --- 添加一个帮助函数，用于读取两个字节的数据并合并为 int16_t ---
/**
  * @brief  从 MPU6500 读取两个字节的数据并合并为 int16_t
  * @param  reg_addr_h: 数据高位的寄存器地址
  * @param  data: 指向 int16_t 变量的指针，用于存储读取到的数据
  * @retval 0: 成功, -1: 失败
  */
static int8_t mpu6500_read_word(uint8_t reg_addr_h, int16_t* data) {
    uint8_t buffer[2];
    // MPU6500 支持连续读取，从高位地址开始读2个字节
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6500_I2C_ADDR, reg_addr_h, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) == HAL_OK) {
        // 数据高位在前，低位在后
        *data = (int16_t)(buffer[0] << 8 | buffer[1]);
        return 0; // Success
    }
    printf("MPU6500 Read Word Error: reg 0x%X\r\n", reg_addr_h);
    return -1; // Error
}

// --- 实现读取三轴加速度计原始数据的函数 ---
/**
  * @brief  读取 MPU6500 三轴加速度计的原始数据
  * @param  accel_x: 指向 int16_t 变量的指针，用于存储 X 轴数据
  * @param  accel_y: 指向 int16_t 变量的指针，用于存储 Y 轴数据
  * @param  accel_z: 指向 int16_t 变量的指针，用于存储 Z 轴数据
  * @retval 0: 成功, -1: 失败
  */
int8_t mpu6500_read_accel_raw(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z) {
    // 加速度计数据寄存器地址是连续的: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ..., ACCEL_ZOUT_L
    // 可以通过一次I2C读取6个字节
    uint8_t buffer[6];
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6500_I2C_ADDR, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY) == HAL_OK) {
        *accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
        *accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
        *accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
        return 0;
    }
    printf("MPU6500 Read Accel Raw Error\r\n");
    return -1;
}

// --- 实现读取三轴陀螺仪原始数据的函数 ---
/**
  * @brief  读取 MPU6500 三轴陀螺仪的原始数据
  * @param  gyro_x: 指向 int16_t 变量的指针，用于存储 X 轴数据
  * @param  gyro_y: 指向 int16_t 变量的指针，用于存储 Y 轴数据
  * @param  gyro_z: 指向 int16_t 变量的指针，用于存储 Z 轴数据
  * @retval 0: 成功, -1: 失败
  */
int8_t mpu6500_read_gyro_raw(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z) {
     // 陀螺仪数据寄存器地址是连续的: GYRO_XOUT_H, ..., GYRO_ZOUT_L
    uint8_t buffer[6];
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6500_I2C_ADDR, GYRO_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY) == HAL_OK) {
        *gyro_x = (int16_t)(buffer[0] << 8 | buffer[1]);
        *gyro_y = (int16_t)(buffer[2] << 8 | buffer[3]);
        *gyro_z = (int16_t)(buffer[4] << 8 | buffer[5]);
        return 0;
    }
    printf("MPU6500 Read Gyro Raw Error\r\n");
    return -1;
}

/**
  * @brief  向 MPU6500 写入一个字节数据到指定寄存器
  * @param  reg_addr: 目标寄存器地址
  * @param  data: 要写入的数据
  * @retval None
  */
void mpu6500_write_byte(uint8_t reg_addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c2, MPU6500_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        // 错误处理，例如打印错误信息或进入 Error_Handler()
        printf("MPU6500 Write Error: reg 0x%X, status %d\r\n", reg_addr, status);
        // Error_Handler(); // 如果需要，可以取消注释
    }
}

/**
  * @brief  从 MPU6500 指定寄存器读取一个字节数据
  * @param  reg_addr: 目标寄存器地址
  * @retval 读取到的数据
  */
uint8_t mpu6500_read_byte(uint8_t reg_addr)
{
    uint8_t read_data = 0;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c2, MPU6500_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &read_data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        // 错误处理
        printf("MPU6500 Read Error: reg 0x%X, status %d\r\n", reg_addr, status);
        // Error_Handler(); // 如果需要，可以取消注释
    }
    return read_data;
}

// --- 修改 mpu6500_init 函数，确保配置了加速度计的量程 ---
/**
  * @brief  初始化 MPU6500
  * @param  None
  * @retval 0: 成功, -1: 失败
  */
int  mpu6500_init(void)
{
    uint8_t who_am_i_val = 0;
    HAL_StatusTypeDef status;

    HAL_Delay(100); // 等待 MPU6500 上电稳定

    // 1. 检查设备是否在线
    // 尝试3次，超时100ms，使用 HAL_I2C_IsDeviceReady 更好
    status = HAL_I2C_IsDeviceReady(&hi2c2, MPU6500_I2C_ADDR, 3, 100);
    if (status != HAL_OK)
    {
        printf("MPU6500 not found on I2C2. Status: %d\r\n", status);
        // Error_Handler(); // 初始化失败通常应该阻止程序继续
        return -1;
    }
    else
    {
        printf("MPU6500 found on I2C2.\r\n");
    }

    // 2. 读取 WHO_AM_I 寄存器进行验证
    who_am_i_val = mpu6500_read_byte(WHO_AM_I_REG);
    printf("WHO_AM_I register value: 0x%X\r\n", who_am_i_val);

    // MPU6500 的 WHO_AM_I 默认值是 0x70。根据模块型号也可能是 0x71 (MPU9250/6500), 0x68 (MPU6050), 0x73, 0x7D, 0x98。
    // 请根据你的 MPU6500 模块数据手册确认。
    if (who_am_i_val != 0x70 && who_am_i_val != 0x71 && who_am_i_val != 0x68 && who_am_i_val != 0x73 && who_am_i_val != 0x7D && who_am_i_val != 0x98) {
        printf("MPU6500 WHO_AM_I check failed. Expected 0x70 or similar, got 0x%X\r\n", who_am_i_val);
        // Error_Handler(); // WHO_AM_I 错误通常是接线或设备问题
        return -1;
    } else {
        printf("MPU6500 WHO_AM_I check passed.\r\n");
    }

    HAL_Delay(50); // 等待稳定

    // 3. 唤醒 MPU6500 (清除 PWR_MGMT_1 寄存器的 SLEEP 位)
    mpu6500_write_byte(PWR_MGMT_1_REG, 0x00); // 设置 CLKSEL=0 (PLL)
    HAL_Delay(50);

    // 4. 配置数字低通滤波器 (DLPF) - 可选
    //    查阅数据手册 CONFIG 寄存器 (0x1A)
    //    例如设置为 Accel BW 44.8Hz, Gyro BW 41Hz (DLPF_CFG = 3)
//    mpu6500_write_byte(CONFIG_REG, 0x03);
//    HAL_Delay(50);


    // 5. 配置陀螺仪量程
    //    0x18: ±2000 dps (对应灵敏度 16.4 LSB/dps) - 与 mpu6500.h 中的定义一致
    mpu6500_write_byte(GYRO_CONFIG_REG, 0x18);
    HAL_Delay(50);

    // 6. 配置加速度计量程
    //    0x00: ±2g (对应灵敏度 16384 LSB/g) - 与 mpu6500.h 中的定义一致
    mpu6500_write_byte(ACCEL_CONFIG_REG, 0x00);
    HAL_Delay(50);

    // 更多配置 (例如中断设置 INT_PIN_CFG, INT_ENABLE 等，如果需要中断)
    // mpu6500_write_byte(0x37, 0x02); // INT_PIN_CFG: INT_ANYRD_2CLEAR, LATCH_INT_EN=0
    // mpu6500_write_byte(0x38, 0x01); // INT_ENABLE: DATA_RDY_EN=1

    printf("MPU6500 configured successfully.\r\n");
    return 0; // 初始化成功
}

/**
  * @brief  读取 MPU6500 Z轴陀螺仪的角速度值
  * @param  None
  * @retval float Z轴角速度 (单位: 度/秒 deg/s)
  */
//float mpu6500_read_gyro_z(void)
//{
//    uint8_t gyro_z_h, gyro_z_l;
//    int16_t gyro_z_raw;
//    float omega_z;
//
//    // 读取 Z 轴陀螺仪数据高位和低位
//    gyro_z_h = mpu6500_read_byte(GYRO_ZOUT_H_REG);
//    gyro_z_l = mpu6500_read_byte(GYRO_ZOUT_L_REG);
//
//    // 合并高低位数据 (注意是有符号16位数)
//    gyro_z_raw = (int16_t)((gyro_z_h << 8) | gyro_z_l);
//
//    // 根据灵敏度转换为实际角速度 (度/秒)
//    // 如果你在 GYRO_CONFIG_REG 中设置了不同的量程, 需要修改 GYRO_SENSITIVITY
//    omega_z = (float)gyro_z_raw / GYRO_SENSITIVITY_2000DPS;
//
//    return omega_z;
//}
