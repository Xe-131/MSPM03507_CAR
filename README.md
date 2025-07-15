## Summary
LP-MSPM0G3507 为主控板，TB6612 为电机驱动，以及霍尔编码器进行速度反馈的四轮小车代码。现阶段完成了速度闭环和角度闭环的代码框架。
LP-MSPM0G3507引脚功能表.xlsx 文件表明了所有的引脚分配，配合empty.syscfg 文件即可进行硬件连线。
## Code Structure
- root
	- MPU6050/
		- 角度传感器驱动代码
	- MSPM0/
		- 中断以及精确时钟代码
	- OLED_Hardware_I2C/
		- OLED 驱动代码
	- motor/
		- 电机驱动代码和PID 代码
	- user/
		- 主函数文件以及用户自定义函数文件
	- empty.syscfg
		- 图形化配置文件
	- README.md
## Code Execution Logic
进入main 函数开始运行用户代码
- while 前： 
	- 硬件外设的初始化
	- 中断的使能
		- 三个定时器的中断
		- 编码器测速GPIO 中断
	- 各个模块初始化
		- MPU6050
		- PID
		- 电机
	- 开启等待MPU6050 的定时
- 进入while:轮训检测flag, 刚开始不会执行任何代码，等待MPU6050 定时结束后，会开启其他定时器的计数
## Example Usage
Compile, load and run the example.
