#include "pid.h"
#include "motor.h"

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
	// ????????
	pid->error[0] = pid->target - pid->now;

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

	// ??????
	if(pid->out >= MAX_DUTY)	
		pid->out	= MAX_DUTY;
	else if(pid->out <= MIN_DUTY)
		pid->out	= MIN_DUTY;
	
}


