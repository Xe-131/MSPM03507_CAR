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
// 目标角度
extern float	target_angle;

// 实际duty 值
extern float   motor_pid_right_value;
extern float   motor_pid_left_value;

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

extern pid_t pid_motor_left;
extern pid_t pid_motor_right;
extern pid_t pid_angle;

// 初始化参数
void pid_init(pid_t *pid, uint32_t pid_mode, float p, float i, float d);
// 返回计算结果
float pid_calculate(pid_t *pid, int now, int target);
// pid 具体计算
void pid_cal(pid_t *pid);

// 实际控制函数
void pid_controal(void);
// 重置PID 所有参数(除了参数p, i, d, pid_mode)
void reset_pid(pid_t* pid);
// 始终让误差在 [-180, 180] 区间内
float get_angle_error(float target, float current);
#endif
