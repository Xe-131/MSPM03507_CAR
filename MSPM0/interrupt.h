#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

// ms
# define TIMER_PID_PERIOD 100
# define TIMER_INTOWHILE_PERIOD 15000
# define TIMER_GENERAL_PERIOD 1000

extern uint8_t into_wihle_flag;
extern uint8_t pid_timer_flag;
extern uint8_t general_timer_flag;
// S2 
extern uint8_t S2_flag;

// mavlink 串口接收缓冲区
#define UART_RX_BUFFER_SIZE 50
extern uint8_t uart_rx_buffer[];
extern uint16_t uart_rx_head;
extern uint16_t uart_rx_tail;

#endif  