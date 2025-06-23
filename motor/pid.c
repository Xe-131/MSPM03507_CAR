#include "pid.h"
#include "motor.h"
#include "interrupt.h"
#include "mpu6050.h"
#include "user.h"

pid_t pid_motor_left;
pid_t pid_motor_right;
pid_t pid_angle;

// 编码器计数
int encode_cnt_left     = 0;
int encode_cnt_right    = 0;
// 实时速度 mm/s
float now_speed_left    = 0;
float now_speed_right   = 0;
// 目标速度
float target_speed_left     = 0;
float target_speed_right    = 0;

// 初始angle
float   angle_init;
// 目标角度
float	target_angle	= 0;

// 实际duty 值
float   motor_pid_left_value;
float   motor_pid_right_value;

// 初始化参数
void pid_init(pid_t *pid, uint32_t pid_mode, float p,  float i, float d){
	pid->pid_mode = pid_mode;
	pid->p	= p;
	pid->d	= d;
	pid->i	= i;
}

// 返回计算结果
float pid_calculate(pid_t *pid, int now, int target){
	pid->target	= target;
	pid->now	= now;
	pid_cal(pid);

	return pid->out;
}

// pid 具体计算
void pid_cal(pid_t *pid)
{
	if(pid->pid_mode == DELTA_PID){
		pid->error[0] = pid->target - pid->now;
	}
	else{
		pid->error[0] = get_angle_error(pid->target, pid->now);
	}

	// ???????
	if(pid->pid_mode == DELTA_PID)  // ?????
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // ��???
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// ???????????
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

	// // ??????
	// if(pid->out >= MAX_DUTY)	
	// 	pid->out	= MAX_DUTY;
	// else if(pid->out <= MIN_DUTY)
	// 	pid->out	= MIN_DUTY;
	
}

// 实际控制函数
void pid_controal(void){
	float   angle_pid_value;

	// 计算转速              
	// mm/s
	now_speed_left  = ((encode_cnt_left*(1.0)) / TIMER_PID_PERIOD) * 1000 / 260 * 150.79;
	now_speed_right = ((encode_cnt_right*(1.0)) / TIMER_PID_PERIOD) * 1000 / 260 * 150.79;
	// 归零计数器
	encode_cnt_left     = 0;
	encode_cnt_right    = 0;
	
	// 计算角度PID: 输入角度差，输出速度值
	angle_pid_value         = pid_calculate(&pid_angle, fmod((angle - angle_init + 360), 360), target_angle);
	// angle_pid_value         = 0;
	// 计算速度PID：输入速度差，输出duty 值
	motor_pid_left_value    = pid_calculate(&pid_motor_left, now_speed_left, target_speed_left - angle_pid_value);
	motor_pid_right_value   = pid_calculate(&pid_motor_right, now_speed_right, target_speed_right + angle_pid_value);
	
	// 设置duty
	Set_Duty(LEFT, motor_pid_left_value);
	Set_Duty(RIGHT, motor_pid_right_value);
}

// 重置PID 所有参数(除了参数p, i, d, pid_mode)
void reset_pid(pid_t* pid){
	pid->target		= 0;
	pid->now		= 0;
	pid->error[0]	= 0;
	pid->error[1]	= 0;
	pid->error[2]	= 0;
	pid->dout		= 0;
	pid->iout		= 0;
	pid->pout		= 0;
	pid->out		= 0;
}

// 始终让误差在 [-180, 180] 区间内
float get_angle_error(float target, float current){
    float error = target - current;
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;
    return error;
}
