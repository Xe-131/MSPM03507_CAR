#include "pid.h"
#include "motor.h"
#include "interrupt.h"
#include "mpu6050.h"
#include "user.h"

// 定义三个PID 对象
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
// 实时角度，定义在头文件中
// #define now_angle  fmod((angle - angle_init + 360.0f), 360.0f)
// 目标角度: 
float	target_angle	= 0;

// 实际duty 值
float   motor_pid_left_value;
float   motor_pid_right_value;

/**
 * @brief   初始化PID 参数
 * @param   pid PID 对象
 * @param   pid_mode PID 计算模式，分为角度和速度
 * @param   p
 * @param   i
 * @param   d
 * @return  无
 */
void pid_init(pid_t *pid, uint32_t pid_mode, float p,  float i, float d){
	pid->pid_mode = pid_mode;
	pid->p	= p;
	pid->d	= d;
	pid->i	= i;
}

/**
 * @brief   计算PID 的输出, 并把中间值保存到PID 对象中
 * @param   pid PID 对象
 * @param   now 目前值
 * @param   target 目标值
 * @return  pid->out PID 输出值
 * @note 这里的PID 输出并没有限制大小
 */
float pid_calculate(pid_t *pid, float now, float target){
	pid->target	= target;
	pid->now	= now;

	// 速度
	if(pid->pid_mode == DELTA_PID){
		// 直接作差
		pid->error[0] = pid->target - pid->now;
	}
	// 角度
	else if(pid->pid_mode == POSITION_PID){
		// 消除0-360 角度的震荡
		pid->error[0] = get_angle_error(pid->target, pid->now);
	}

	// 速度
	if(pid->pid_mode == DELTA_PID)
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	// 角度
	else if(pid->pid_mode == POSITION_PID)  
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];

	return pid->out;
}

/**
 * @brief   计算调用间隔的轮子速度，融合角度与速度PID 的输出，转化为duty，最后改变轮子的PWM
 * @return  无
 */
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
	angle_pid_value         = pid_calculate(&pid_angle, now_angle, target_angle);
	// 计算速度PID：输入速度差，输出duty 值, 并限幅
	motor_pid_left_value    = limit_float(pid_calculate(&pid_motor_left, now_speed_left, target_speed_left - angle_pid_value), -100, 100);
	motor_pid_right_value   = limit_float(pid_calculate(&pid_motor_right, now_speed_right, target_speed_right + angle_pid_value),-100, 100);
	
	// 设置duty
	Set_Duty(LEFT, motor_pid_left_value);
	Set_Duty(RIGHT, motor_pid_right_value);
}

/**
 * @brief   重置PID 所有中间变量(除了参数p, i, d, pid_mode)
 * @param   pid PID 对象
 * @return  无
 */
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

/**
 * @brief   始终让误差在 [-180, 180] 区间内，消除反复震荡
 * @param   target 目标角度
 * @param   now 实时角度
 * @return  无
 */
float get_angle_error(float target, float now){
    float error = target - now;
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;
    return error;
}

/**
 * @brief   限制value 到某个范围
 * @param   value 限制前的值
 * @param   min 最小值
 * @param   max 最大值
 * @return  value 限制后的值
 */
float limit_float(float value, float min, float max){
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief   改变目标速度
 * @param   speed 速度值，范围在[-1000, 1000]
 * @return  无
 * @note 这里的范围是软范围，具体得看小车在PWM 为100 时的速度值
 */
void set_target_speed(float speed){
	speed = limit_float(speed, -1000, 1000);

	target_speed_left	= speed;
	target_speed_right	= speed;
}

/**
 * @brief   改变目标角度
 * @param   angle 角度值，范围在[0, 360)
 * @return  无
 */
void set_target_angle(float angle){
	angle = limit_float(angle, 0, 359.99);

	target_angle	= angle;
}