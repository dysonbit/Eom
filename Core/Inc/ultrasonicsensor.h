#ifndef __ULTRASONICSENSOR_H__
#define __ULTRASONICSENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* === HC‑SR04 引脚 (按实际接线修改端口/引脚) ========================== */
#define ULTRA_TRIG_GPIO_Port GPIOC
#define ULTRA_TRIG_Pin       GPIO_PIN_8

#define ULTRA_ECHO_GPIO_Port GPIOC
#define ULTRA_ECHO_Pin       GPIO_PIN_9
/* ======================================================================= */

/* 障碍阈值：小于该距离(cm) 触发避障 */
#define ULTRA_THRESHOLD_CM      15.0f

/* —— 接口 —— */
void  ultrasonic_init      (void);
void ultrasonic_get_cm(float *out);


#ifdef __cplusplus
}
#endif
#endif /* __ULTRASONICSENSOR_H__ */
