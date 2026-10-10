#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f1xx_hal.h" // 请根据你的芯片系列替换对应的头文件，如 stm32f4xx_hal.h

#ifdef __cplusplus
extern "C" {
#endif

void delay_init(void);          // 初始化DWT计数器（建议在系统时钟配置后调用）
void delay_ms(uint32_t ms);     // 毫秒延时（直接调用HAL库）
void delay_us(uint32_t us);     // 微秒延时（基于DWT，不占用定时器）

#ifdef __cplusplus
}
#endif

#endif