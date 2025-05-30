/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h" // OLED 显示库
#include "mpu6500.h" // MPU6500 驱动库
#include "bluetooth_hc05.h"
#include "bluetooth_hc05_at.h"
#include "motor.h"
#include "mpu6500.h"
#include "math.h" // 包含 M_PI 等常量 (如果需要在 main.c 中使用)
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HC05_DATA_MODE_BAUDRATE 9600 // 定义HC-05在数据模式下的波特率 (必须与AT指令配置的一致)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char oled_display_buffer[BLUETOOTH_RX_BUFFER_SIZE + 20]; // OLED显示缓冲区, 留出 "BT Rcvd: " 前缀空间
uint8_t bluetooth_received_data_buffer[BLUETOOTH_RX_BUFFER_SIZE];
char at_response_buffer[128];

//extern float g_pitch_angle;
//extern int16_t g_motor_output_left;
//extern int16_t g_motor_output_right;
//extern float g_accel_angle; // 也可以显示加速度计角度
//extern float g_gyro_y_dps; // 也可以显示陀螺仪角速度

extern volatile int16_t g_accel_x_raw, g_accel_y_raw, g_accel_z_raw;
extern volatile int16_t g_gyro_x_raw, g_gyro_y_raw, g_gyro_z_raw;

extern TIM_HandleTypeDef htim5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void calibrate_mpu6500_bias(uint16_t num_samples); // Ensure this prototype is here
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 如果配置了 printf 重定向到 UART，可以保留这个函数
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/**
  * @brief  Calibrates MPU6500 sensor bias.
  *         Requires the robot to be held stationary and balanced during execution.
  * @param  num_samples: Number of samples to collect for averaging.
  * @retval None
  */
void calibrate_mpu6500_bias(uint16_t num_samples) {
    printf("\r\n--- MPU6500 Calibration Start ---\r\n");
    printf("Please hold the robot VERTICAL and STATIONARY for %d seconds...\r\n", num_samples / 1000); // Assuming 1ms delay per sample

    // Give user time to position the robot
    HAL_Delay(3000); // 3 seconds initial delay

    // Accumulate sums
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;
    float accel_pitch_angle_sum = 0.0f;

    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    float current_accel_pitch_angle;

    // Collect samples
    for (uint16_t i = 0; i < num_samples; i++) {
        // Read raw data
        if (mpu6500_read_accel_raw(&accel_x_raw, &accel_y_raw, &accel_z_raw) != 0) {
            printf("Calibration Error: Failed to read accel raw.\r\n");
            // Optionally skip this sample or break calibration
            continue; // Skip this sample
        }
        if (mpu6500_read_gyro_raw(&gyro_x_raw, &gyro_y_raw, &gyro_z_raw) != 0) {
            printf("Calibration Error: Failed to read gyro raw.\r\n");
            // Optionally skip this sample or break calibration
             continue; // Skip this sample
        }

        // Accumulate gyro raw sums
        gyro_x_sum += gyro_x_raw;
        gyro_y_sum += gyro_y_raw;
        gyro_z_sum += gyro_z_raw;

        // Calculate accelerometer pitch angle for this sample (based on Accel X and Z)
        // atan2f handles the case where the first parameter is 0.
        // If accel_z_raw is 0 (e.g., MPU horizontal), atan2f(accel_x_raw, 0) would give +/-90 or 0/180 depending on accel_x_raw sign.
        // In vertical calibration, accel_z_raw is expected to be large (~16384), so 0 is very unlikely.
        current_accel_pitch_angle = atan2f((float)accel_x_raw, (float)accel_z_raw) * (180.0f / M_PI);
        accel_pitch_angle_sum += current_accel_pitch_angle;

        // Add a small delay to ensure distinct readings
        HAL_Delay(1); // 1ms delay per sample
    }

    // Calculate averages (biases)
    // Ensure num_samples is not zero to avoid division by zero
    if (num_samples > 0) {
         g_gyro_x_bias_raw = (int16_t)(gyro_x_sum / num_samples);
         g_gyro_y_bias_raw = (int16_t)(gyro_y_sum / num_samples);
         g_gyro_z_bias_raw = (int16_t)(gyro_z_sum / num_samples);
         g_accel_pitch_bias_deg = accel_pitch_angle_sum / num_samples;
    } else {
         printf("Calibration Warning: num_samples is 0.\r\n");
         // Biases remain 0
    }


    printf("Calibration Complete.\r\n");
    printf("Gyro Bias Raw: X=%d, Y=%d, Z=%d\r\n", g_gyro_x_bias_raw, g_gyro_y_bias_raw, g_gyro_z_bias_raw);
    printf("Accel Pitch Bias (deg): %.2f\r\n", g_accel_pitch_bias_deg);
    printf("--- MPU6500 Calibration End ---\r\n");
}

void configure_hc05_via_at(void) {
    printf("\r\n--- HC-05 AT Configuration Mode ---\r\n");
//    ssd1306_Init(&hi2c1);
//    ssd1306_Fill(Black);
//    ssd1306_SetCursor(0,0);
//    ssd1306_WriteString("Reset",Font_7x10,White);
//    ssd1306_UpdateScreen(&hi2c1);

    hc05_enter_at_mode_hardware_reset(); // 进入AT模式，UART波特率会被设置为38400

    // 等待用户操作并确认模块已进入AT模式
    printf("Press any key in serial terminal to continue after HC-05 restarted in AT mode...\r\n");
    // 简单的等待按键示例 (需要你的printf对应的UART能接收)
    // char dummy_input;
    // HAL_UART_Receive(&huart2, (uint8_t*)&dummy_input, 1, HAL_MAX_DELAY); // 假设huart2用于调试

    HAL_Delay(2000); // 或者简单延时

    // 1. 测试AT指令
    if (hc05_send_at_command("AT\r\n", at_response_buffer, sizeof(at_response_buffer), 1000)) {
        // 检查响应是否为 "OK\r\n"
        if (strstr(at_response_buffer, "OK") != NULL) {

            printf("AT test successful!\r\n");
        } else {

            printf("AT test response not OK.\r\n");
        }
    } else {

        printf("AT test response not OK.\r\n");
        printf("AT test failed (no response or error).\r\n");
    }
    HAL_Delay(500);

    // 2. 查询版本信息
    hc05_send_at_command("AT+VERSION?\r\n", at_response_buffer, sizeof(at_response_buffer), 1000);
    HAL_Delay(500);

    // 3. 修改蓝牙名称为 "MySTM32_BT"
     hc05_send_at_command("AT+NAME=MySTM32_BT\r\n", at_response_buffer, sizeof(at_response_buffer), 1000);
     HAL_Delay(500);

    // 4. 修改波特率为9600, 1停止位, 无校验 (数据模式下的波特率)
    // 注意: AT+UART=<Param1>,<Param2>,<Param3>
    // Param1: 波特率 (e.g., 9600, 38400, 115200)
    // Param2: 停止位 (0代表1停止位, 1代表2停止位)
    // Param3: 校验位 (0代表None, 1代表Odd, 2代表Even)
     hc05_send_at_command("AT+UART=9600,0,0\r\n", at_response_buffer, sizeof(at_response_buffer), 1000);
     HAL_Delay(500);

    // 5. 修改配对码为 "0000"
     hc05_send_at_command("AT+PSWD=0000\r\n", at_response_buffer, sizeof(at_response_buffer), 1000);
     HAL_Delay(500);


    // 更多AT指令示例:
    // 查询当前名称: AT+NAME?\r\n
    // 查询当前配对码: AT+PSWD?\r\n
    // 查询当前UART配置: AT+UART?\r\n
    // 恢复出厂设置: AT+ORGL\r\n (慎用!)
    // 重启模块: AT+RESET\r\n (会退出AT模式)

//    printf("\r\n--- HC-05 AT Configuration Finished ---\r\n");
//    ssd1306_Fill(Black);
//    ssd1306_SetCursor(0,0);
//    ssd1306_WriteString("PowerOFF",Font_7x10,White);
//    ssd1306_UpdateScreen(&hi2c1);
    hc05_send_at_command("AT+RESET\r\n", at_response_buffer, sizeof(at_response_buffer), 1000);
    hc05_exit_at_mode_hardware_reset();


    // **重要**: 配置完成后，你需要将STM32的UART波特率改回HC-05数据模式下设置的波特率
    // 例如，如果你上面设置了 "AT+UART=9600,0,0\r\n"
    // if (hc05_reconfigure_uart_baudrate(9600) != HAL_OK) {
    //     printf("Failed to reconfigure UART to 9600 bps for data mode.\r\n");
    // } else {
    //     printf("UART reconfigured to 9600 bps for data mode.\r\n");
    // }
    // 之后再调用 hc05_init() (如果你的 hc05_init 是用于数据模式的)
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char oled_buffer[30]; // 用于在OLED上显示字符串，大小根据需要调整
  float gyro_z_value = 0.0f;




  // ==================================================

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // 启动编码器模式
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动电机1编码器
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动电机2编码器

    if (hc05_init() != HAL_OK) {
        printf("BT failed");
        // 初始化失败处理
        Error_Handler();
    }
    // ... hc05_init() OK ...
    const char* test_msg = "Hello from STM32!\r\n";
    if (hc05_transmit_data((uint8_t*)test_msg, strlen(test_msg)) == HAL_OK) {
        printf("Test message sent to Bluetooth.\r\n");
    } else {
        printf("Failed to send test message to Bluetooth.\r\n");
    }
    HAL_Delay(5000);
//  // 初始化OLED (使用 hi2c1)
  if (ssd1306_Init(&hi2c1) != 0 ){
      printf("OLED SSD1306 initialization failed!\r\n");
      Error_Handler(); // 初始化失败
  }
  printf("OLED SSD1306 initialized successfully.\r\n");


  // 初始化MPU6500 (使用 hi2c2, mpu6500.c 中应使用 extern I2C_HandleTypeDef hi2c2;)
  if(mpu6500_init()==-1){
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0,0);
      ssd1306_WriteString("MPU Fail", Font_7x10, White);
      ssd1306_UpdateScreen(&hi2c1);
      HAL_Delay(1000);
  } else {
	    printf("MPU6500 initialized.\r\n");
	}// 这个函数内部应该有打印信息指示是否成功
  // Calibrate MPU6500 bias
//   Need to hold the robot stationary and balanced during this process
  calibrate_mpu6500_bias(2000); // Collect 2000 samples (takes ~2 seconds + initial delay)

  // 清屏并显示初始文本
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("OLED Init OK", Font_7x10, White);
  ssd1306_UpdateScreen(&hi2c1);
  HAL_Delay(1000);
//    configure_hc05_via_at();
//    if (huart1.Init.BaudRate != HC05_DATA_MODE_BAUDRATE) {
//        printf("Current STM32 UART1 baud: %lu. Reconfiguring to %d for HC-05 data mode.\r\n",
//               huart1.Init.BaudRate, HC05_DATA_MODE_BAUDRATE);
//        HAL_UART_DeInit(&huart1);
//        huart1.Init.BaudRate = HC05_DATA_MODE_BAUDRATE;
//        if (HAL_UART_Init(&huart1) != HAL_OK) {
//            printf("Failed to re-init UART1 with new baud rate!\r\n");
//            Error_Handler();
//        }
//        printf("STM32 UART1 reconfigured to %d bps.\r\n", HC05_DATA_MODE_BAUDRATE);
//    }
//    if (hc05_init() == HAL_OK) {
//        printf("Bluetooth HC-05 Initialized.\r\n");
//        ssd1306_Fill(Black);
//        ssd1306_SetCursor(0,0);
//        ssd1306_WriteString("BT good", Font_7x10, White);
//        ssd1306_UpdateScreen(&hi2c1);
//        HAL_Delay(1000);
//    } else {
//        printf("Bluetooth HC-05 Init Failed!\r\n");
//        ssd1306_Fill(Black);
//        ssd1306_SetCursor(0,0);
//        ssd1306_WriteString("BT Fail", Font_7x10, White);
//        ssd1306_UpdateScreen(&hi2c1);
//        HAL_Delay(1000);
//        Error_Handler();
//    }

    // 初始化电机模块 (会启动 TIM1 PWM 通道)
    Motor_Init();
    printf("Motor module initialized.\r\n");
    HAL_Delay(500);

    // 启动控制循环定时器 TIM5 的中断
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK) {
    	printf("Failed to start control loop timer (TIM5)!\r\n");
    	Error_Handler(); // 启动定时器失败是严重错误
    }
    printf("Control Loop Timer (TIM5) Started.\r\n");




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      while (1)
      {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
          if (hc05_is_data_received()) {
              printf("success");
              uint8_t received_buffer[BLUETOOTH_RX_BUFFER_SIZE];
              uint16_t len = hc05_get_received_data(received_buffer, BLUETOOTH_RX_BUFFER_SIZE);
              if (len > 0) {
                  printf("BT:");
                  // 处理接收到的数据 received_buffer，长度为 len
                  // 例如，通过另一个 UART 打印出来，或者回传给蓝牙
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(0, 0);
                  char* string=received_buffer;
                  ssd1306_WriteString(string, Font_7x10, White);
                  ssd1306_UpdateScreen(&hi2c1);
                  printf("BT Rcvd: %s\n", (char*)received_buffer);
                  hc05_transmit_data(received_buffer, len); // 示例：回显数据
              }
          }


          // 主循环现在用于执行非实时的任务，例如：
          // - 更新 OLED 显示 (频率较低)
          // - 处理接收到的蓝牙命令 (设置目标速度、转向、模式等)
          // - 状态指示 (例如 LED 闪烁)

    	  // =======================================
    	  // mpu
    	  // Example: Print raw sensor data periodically for verification

    	      	  static uint32_t print_counter = 0;
    	  if (++print_counter % 100 == 0) { // Print approximately every 1 second (adjust frequency as needed)
    	       print_counter = 0;

//    	        Calculate current pitch error for printing (error is calculated in the IRQ, but recalculate here for printing convenience)
    	       float current_pitch_error = g_target_pitch_angle - g_pitch_angle;

    	       printf("FusedPitch: %.2f | AccelAng: %.2f | Gyro(corr): X=%.2f | Err: %.2f | CtrlOut: %.2f | Motor(LR): %d,%d\r\n",
    	              g_pitch_angle,        // Fused pitch angle
    	              g_accel_angle,        // Accel calculated pitch angle after bias correction
    	              g_gyro_x_dps,         // Corrected Gyro Pitch rate (deg/s)
    	              current_pitch_error,  // Pitch error
    	              g_balance_kp * current_pitch_error + g_balance_kd * g_gyro_x_dps, // Recalculate Control Output for printing (simplified PD)
    	              g_motor_output_left,  // Final motor output (left)
    	              g_motor_output_right  // Final motor output (right)
    	              );

//    	  }



//    	       static int16_t test_speed = 0;
//    	       static uint32_t speed_test_delay = 2000; // 每隔2秒增加一次速度
//
//    	       // 移除或注释掉之前的周期性打印原始数据的代码，以免干扰
//
//    	       static uint32_t last_test_time = 0;
//    	       if (HAL_GetTick() - last_test_time > speed_test_delay) {
//    	           last_test_time = HAL_GetTick();
//
//    	           // 逐渐增加测试速度
//    	           test_speed += 1; // 每次增加 1%
//    	           if (test_speed > 20) test_speed = 0; // 例如，测试到 20% 就重置
//
//    	           printf("Testing speed: %d%%\r\n", test_speed);
//
//    	           Car_Move(test_speed, test_speed); // 测试正向速度
//    	           // 你也可以测试负向速度 Car_Move(-test_speed, -test_speed);
//    	       }


//    	       	   	  Car_Forward(20); // 这行是活的
//    	           	  HAL_Delay(2000); // Keep a small delay to prevent WDT issues or busy waiting
//    	           	  Car_Backward(20); // 这行是活的
//    	           	  HAL_Delay(2000);
//    	           	  Car_TurnLeft(20); // 这行是活的
//    	           	  HAL_Delay(2000);
//    	           	  Car_TurnRight(20); // 这行是活的
//    	           	  HAL_Delay(2000); // Keep a small delay to prevent WDT issues or busy waiting



    	  // =======================================
    	  // motor

    	  // =======================================


//          // 示例: 更新 OLED 显示当前姿态和电机输出
//          static uint32_t oled_update_counter = 0;
//          char oled_buffer[40]; // 确保 buffer 足够大

//          // 降低 OLED 更新频率，例如每 50ms 更新一次 (TIM5 中断 10ms，计数 5 次)
//          if (++oled_update_counter % 5 == 0) {
//              oled_update_counter = 0; // 重置计数器
//              ssd1306_Fill(Black); // 清空屏幕缓冲区
//
//              // 显示 Pitch 角
//              ssd1306_SetCursor(0, 0); // 第一行
//              snprintf(oled_buffer, sizeof(oled_buffer), "P:%.1f (A%.1f)", g_pitch_angle, g_accel_angle); // 显示滤波后和加速度计角度
//              ssd1306_WriteString(oled_buffer, Font_7x10, White);
//
//              // 显示角速度
//              ssd1306_SetCursor(0, 12); // 第二行
//              snprintf(oled_buffer, sizeof(oled_buffer), "G:%.1f", g_gyro_y_dps);
//              ssd1306_WriteString(oled_buffer, Font_7x10, White);
//
//              // 显示电机 PWM 输出 (百分比)
//              ssd1306_SetCursor(0, 24); // 第三行
//              snprintf(oled_buffer, sizeof(oled_buffer), "L:%d R:%d %%", g_motor_output_left, g_motor_output_right);
//              ssd1306_WriteString(oled_buffer, Font_7x10, White);
//
//              ssd1306_UpdateScreen(&hi2c1); // 更新 OLED 显示
//         	 }

//          HAL_Delay(1); // 让主循环稍微延时一下，避免完全空转浪费资源
    	  }
  /* USER CODE END 3 */
      }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  UART接收完成回调函数
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 调用蓝牙模块的内部回调处理函数
    hc05_uart_rx_callback_handler(huart);


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  // 使用忙等待以确保LED在中断关闭时仍能闪烁
  // led_GPIO_Port 和 led_Pin 应该是在 main.h 中通过 CubeMX 定义的宏
  // 假设是 PA5 (Nucleo 板载 LED)
  // 请确保 led_GPIO_Port 和 led_Pin 宏定义正确
  // 或者直接使用 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  while (1)
  {
    HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin); // 使用 main.h 中定义的宏
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
   printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
