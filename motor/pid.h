#ifndef __PID_h_
#define __PID_h_

#include "ti_msp_dl_config.h"

#define MAX_DUTY 100
#define MIN_DUTY -100

// 编码器计数
extern int encode_cnt_left;
extern int encode_cnt_right;
// 实时速度 mm/s
extern float now_speed_left;
extern float now_speed_right;
// 目标速度
extern float target_speed_left;
extern float target_speed_right;

// 初始angle角
extern float	angle_init;
// 实时角度
#define now_angle  fmod((angle - angle_init + 360.0f), 360.0f)
// 目标角度
extern float	target_angle;

// 实际duty 值
extern float   motor_pid_right_value;
extern float   motor_pid_left_value;

enum
{
  POSITION_PID = 0,  
  DELTA_PID,         
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

extern pid_t pid_motor_left;
extern pid_t pid_motor_right;
extern pid_t pid_angle;

void pid_init(pid_t *pid, uint32_t pid_mode, float p, float i, float d);
float pid_calculate(pid_t *pid, float now, float target);
void pid_controal(void);
void reset_pid(pid_t* pid);
float get_angle_error(float target, float current);
float limit_float(float value, float min, float max);
void set_target_speed(float speed);
void set_target_angle(float angle);

#endif