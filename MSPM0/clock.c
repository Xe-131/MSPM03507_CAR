#include "ti_msp_dl_config.h"
#include "clock.h"

// 初始化了SysTick ，每1ms 会触发一次中断
// 这是移植MPU6050 时的附带文件，用于被MPU6050 驱动文件调用
// 本项目并没有其他用途

volatile unsigned long tick_ms;
volatile uint32_t start_time;

int mspm0_delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}
