#ifndef __PID_h_
#define __PID_h_
#include "ti_msp_dl_config.h"

#define MAX_DUTY 100
#define MIN_DUTY -100

enum
{
  POSITION_PID = 0,  // λ��ʽ
  DELTA_PID,         // ����ʽ
};

typedef struct
{
	float target;	
	float now;
	float error[3];	
	float p,i,d;
	float pout;
	float dout;
	float iout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;

pid_t pid_motor_left;
pid_t pid_motor_right;
pid_t pid_angle;

// 初始化参数
void pid_init(pid_t *pid, uint32_t pid_mode, float p, float i, float d);
// 返回计算结果
float pid_calculate(pid_t *pid, int now, int target);
// pid 具体计算
void pid_cal(pid_t *pid);

#endif
