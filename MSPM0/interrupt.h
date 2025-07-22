#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

// ms
# define TIMER_PID_PERIOD 100
# define TIMER_INTOWHILE_PERIOD 20000
// 由于通用定时需要确定多个定时周期，因此其中断时间固定为100 ms
# define TIMER_GENERAL_PERIOD 100

// 中断flag
extern uint8_t into_wihle_flag;
extern uint8_t pid_timer_flag;
extern uint8_t flag_100ms;
extern uint8_t flag_1s; 
extern uint8_t S2_flag;

#endif  