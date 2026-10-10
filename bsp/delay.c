#include "delay.h"

// DWT 控制寄存器和计数寄存器地址（Cortex-M内核通用）
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define DEMCR      (*(volatile uint32_t *)0xE000EDFCUL)

#define DEMCR_TRCENA_BIT (1UL << 24)  // 跟踪使能位
#define DWT_CTRL_CYCCNTENA_BIT (1UL << 0) // 周期计数器使能位

static uint32_t us_ticks = 0; // 每微秒对应的CPU周期数

/**
 * @brief   初始化DWT微秒延时计数器
 * @note    必须在系统时钟初始化完成后调用，且只需调用一次
 * @retval  无
 */
void delay_init(void)
{
    // 1. 使能 DWT 跟踪（解锁调试组件）
    if (!(DEMCR & DEMCR_TRCENA_BIT)) {
        DEMCR |= DEMCR_TRCENA_BIT;
    }
    
    // 2. 复位 CYCCNT 计数器（可选）
    DWT_CYCCNT = 0;
    
    // 3. 使能 CYCCNT 周期计数器
    if (!(DWT_CTRL & DWT_CTRL_CYCCNTENA_BIT)) {
        DWT_CTRL |= DWT_CTRL_CYCCNTENA_BIT;
    }
    
    // 4. 计算每微秒的CPU周期数（例如：168MHz主频则 us_ticks = 168）
    us_ticks = HAL_RCC_GetHCLKFreq() / 1000000UL;
}

/**
 * @brief   毫秒级延时（直接复用HAL库的HAL_Delay）
 * @param   ms: 延时毫秒数
 * @retval  无
 */
void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief   微秒级延时（阻塞式忙等待，基于DWT周期计数器）
 * @param   us: 延时微秒数（最大限制取决于主频，主频168MHz时最大约25秒）
 * @note    不占用任何外设定时器资源，精度为1个CPU周期
 * @retval  无
 */
void delay_us(uint32_t us)
{
    // 如果未初始化，自动调用初始化（确保安全）
    if (us_ticks == 0) {
        delay_init();
    }

    // 计算目标结束时的计数值 (使用64位防止溢出，但us不大时32位足够)
    uint32_t start_ticks = DWT_CYCCNT;
    uint32_t wait_ticks = us * us_ticks; 
    uint32_t end_ticks = start_ticks + wait_ticks;

    // 处理 CYCCNT 计数溢出（32位计数器回绕）
    if (end_ticks < start_ticks) { 
        // 如果发生了32位溢出，分两段等待
        while (DWT_CYCCNT >= start_ticks); // 等待直到计数器溢出回0
        while (DWT_CYCCNT < end_ticks);
    } else {
        while (DWT_CYCCNT < end_ticks);
    }
}