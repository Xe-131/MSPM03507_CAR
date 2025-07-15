#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

// ms
# define TIMER_PID_PERIOD 100
# define TIMER_INTOWHILE_PERIOD 15000
# define TIMER_GENERAL_PERIOD 4000

extern uint8_t into_wihle_flag;
extern uint8_t pid_timer_flag;
extern uint8_t general_timer_flag;
// S2 
extern uint8_t S2_flag;

#endif  