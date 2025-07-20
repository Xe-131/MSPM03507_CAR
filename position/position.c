#include "position.h"
#include "math.h"
#include "pid.h"
#include "mpu6050.h"
#include "user.h"
#include "pid.h"
#include "uwb.h"

// UWB 实时坐标：宏定义在头文件
// #define NOW_x (uwb.x + diff_x)
// #define NOW_y (uwb.y + diff_y)

float diff_x = 0;
float diff_y = 0;

// 目的坐标
float TARGET_x;
float TARGET_y;

// 导航状态
NavigationState_t nav_state = NAV_IDLE;

// 路径跟踪相关的静态全局变量
static const Point_t* g_current_path    = NULL; // 指向当前路径点数组的指针
static int g_total_path_points          = 0;          // 当前路径的总点数
static int g_current_point_index        = 0;        // 当前目标点在路径中的索引
static bool g_is_path_active            = false;        // 路径跟踪是否正在进行中

// 固定路线
Point_t path_1[] = {
    POINT_1,
    POINT_2,   
};
Point_t path_2[] = {
    POINT_1,
    POINT_2,   
};
Point_t path_3[] = {
    POINT_9,
    POINT_8,   
};
Point_t path_4[] = {
    POINT_9,
    POINT_8,
    POINT_6,
    POINT_7,   
};
Point_t path_5[] = {
    POINT_1,
    POINT_2,
    POINT_3,
    POINT_5,   
};
Point_t path_6[] = {
    POINT_1,
    POINT_2,
    POINT_3,
    POINT_4,   
};
// 反向路径
Point_t reverse_path_1[] = {
    POINT_1,    
    POINT_10,   
};
Point_t reverse_path_2[] = {
    POINT_1,    
    POINT_10,   
};
Point_t reverse_path_3[] = {
    POINT_9,    
    POINT_10,   
};
Point_t reverse_path_4[] = {
    POINT_6,   
    POINT_8,   
    POINT_9,   
    POINT_10,   
};
Point_t reverse_path_5[] = {
    POINT_3,    
    POINT_2,    
    POINT_1,    
    POINT_10,   
};
Point_t reverse_path_6[] = {
    POINT_3,    
    POINT_2,    
    POINT_1,    
    POINT_10,
};

// ---------- 公共接口函数 ----------
/*
下面的函数使用方法如下：
    1.导航到(X, Y):
        // 主循环之前
        set_target_point(X, Y);
        // 主循环
        while(1){
            // 周期调用
            if(){
                navigation_update();
            }
        }

    1.跑完包含N 个点的PATH 路径:
        // 主循环之前
        path_start(PATH, N);
        // 主循环
        while(1){
            // 周期调用
            if(){
                navigation_update();
                path_update();
            }
        }
*/

/**
 * @brief 导航到点(x, y)
 * @param x  
 * @param y  
 * @return 无
 * @note 此函数需要配合函数navigation_update 使用
 */
void set_target_point(float x, float y){
    TARGET_x    = x;
    TARGET_y    = y;
    nav_state   = NAV_ROTATING;
}

/**
 * @brief 由set_target_point 函数使能。完成一个目标点的导航
 * @param  无
 * @return 无
 * @note 需要定时调用此函数
 */
 void navigation_update(void) {
    // 角度稳定计数
    static int rotation_stable_counter = 0;
    // 距离稳定计数
    static int distance_stable_counter = 0;
    // 距离PID 输出
    float diatance_pid_out;

    // 计算当前位置到目标的向量和距离
    float dx = TARGET_x - NOW_x;
    float dy = TARGET_y - NOW_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    // 状态机核心逻辑
    switch (nav_state) {

        case NAV_IDLE:
            // 空闲状态，什么也不做，确保小车停止
            set_target_speed(0);
            set_target_angle(0);
            break;

        case NAV_ROTATING: {
            // --- 目标: 原地旋转对准目标点 ---
            
            // 1. 计算目标角度:这里注意坐标系，xy还是yx
            float math_angle_deg = atan2f(dx, dy) * 180.0f / M_PI;
            float angle_to_be = normalize_angle(math_angle_deg - 90.0f);
            
            // 2. 设置电机，速度为0，目标为计算出的角度
            set_target_speed(0);
            set_target_angle(angle_to_be);
            
            // 3. 检查是否对准
            float angle_error = angle_to_be - now_angle;
            // 处理 359 -> 1 度的角度跳变问题
            if (angle_error > 180.0f) angle_error -= 360.0f;
            if (angle_error < -180.0f) angle_error += 360.0f;

            if (fabsf(angle_error) < ANGLE_THRESHOLD) {
                // 如果角度在阈值内，增加稳定计数器
                rotation_stable_counter++;
            } else {
                // 如果角度超出阈值，立即清零计数器，因为稳定被打破
                rotation_stable_counter = 0;
            }

            // 4. 检查稳定计数器是否达到目标帧数
            if (rotation_stable_counter >= ROTATION_STABLE_FRAMES) {
                // 稳定时间足够长，可以安全地切换到前进状态
                nav_state = NAV_MOVING;     
                rotation_stable_counter = 0;  
                // 重置距离PID
                reset_pid(&pid_distance);   
                return;  
            }
            break;
        }

        case NAV_MOVING: {
            // --- 目标: 朝着目标点前进，并实时修正方向 ---
            
            // 1. 检查是否已到达
            if (distance_to_target < DISTANCE_THRESHOLD) {
                distance_stable_counter++;
            } else {
                distance_stable_counter = 0;
            }
            if (distance_stable_counter >= DISTANCE_STABLE_FRAMES) {
                nav_state = NAV_ARRIVED;     
                distance_stable_counter = 0;    
                return;   
            }
            // 2. 实时计算并更新目标角度，用于航向保持
            float math_angle_deg = atan2f(dx, dy) * 180.0f / M_PI;
            float target_angle = normalize_angle(math_angle_deg - 90.0f);
            set_target_angle(target_angle);
            
            // PID 调节速度:PID 输出为速度值
            diatance_pid_out = pid_calculate(&pid_distance, distance_to_target, 0);
            // 限幅
            if(diatance_pid_out >= 400){
                diatance_pid_out = 400;
            }
            else if(diatance_pid_out <= -400){
                diatance_pid_out = -400;
            }
            // 设置速度
            set_target_speed(diatance_pid_out);
            
            break;
        }

        case NAV_ARRIVED:
            // --- 目标: 停止小车 ---
            set_target_speed(0);
            // 可以在这里设置目标角度为当前角度，防止漂移
            set_target_angle(now_angle);

            // 转换到空闲状态，等待下一个指令
            nav_state = NAV_IDLE;
            break;
    }
}

/**
 * @brief 开始执行一条路径
 * @param path Point_t类型的数组，包含路径上所有点的坐标
 * @param num_points 路径中的点数量
 * @return 无
 * @note 需要配合path_update 函数使用
 */
void path_start(const Point_t path[], int num_points) {
    // 1. 输入有效性检查
    if (path == NULL || num_points <= 0) {
        // 如果路径无效，则什么也不做
        g_is_path_active = false;
        return;
    }

    // 2. 初始化路径跟踪状态
    g_current_path          = path;
    g_total_path_points     = num_points;
    g_current_point_index   = 0;
    g_is_path_active        = true;

    // 3. 导航到路径的第一个点
    // 获取第一个点的坐标
    Point_t first_point = g_current_path[g_current_point_index];
    // 调用你现有的函数来开始任务
    set_target_point(first_point.x, first_point.y);
    
    // 4. 将索引指向下一个点，为将来做准备
    g_current_point_index++;
    
    // printf("Path started. Navigating to point 1: (%.2f, %.2f)\n", first_point.x, first_point.y);
}

/**
 * @brief 路径跟踪更新函数，检查是否应前往下一个点
 * @param  无
 * @return 无
 * @note 此函数必须与 navigation_update() 一同在主循环中被周期性调用。
 */
void path_update(void) {
    // 1. 如果没有激活的路径任务，则直接返回
    if (!g_is_path_active) {
        return;
    }

    // 2. 检查单点导航是否完成 (关键触发器)
    if (nav_state == NAV_IDLE) {
        // 3. 检查路径中是否还有剩余的点
        if (g_current_point_index < g_total_path_points) {
            // 还有下一个点，开始导航
            Point_t next_point = g_current_path[g_current_point_index];
            set_target_point(next_point.x, next_point.y);

            // printf("Point %d reached. Navigating to point %d: (%.2f, %.2f)\n", 
            //        g_current_point_index, g_current_point_index + 1, next_point.x, next_point.y);

            g_current_point_index++;

        } else {
            // 4. 所有点都已完成，路径结束
            // printf("Path finished!\n");
            g_is_path_active    = false;
            g_current_path      = NULL; 
            g_total_path_points = 0;
        }
    }
}

/**
 * @brief 检查路径是否已经完成
 * @param  无
 * @return bool 如果路径已完成或未激活，返回true；否则返回false。
 */
bool path_is_finished(void) {
    return !g_is_path_active;
}

// ---------- 内部辅助函数 ----------

/**
 * @brief 将一个任意角度值标准化到 [0, 360) 的范围内
 * @param angle 输入的角度，可以是负数或大于360的数
 * @return 标准化后的角度，范围在 [0, 360)
 */
static float normalize_angle(float angle){
    // 如果角度是负数，则不断加360直到它变为正数
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    // 如果角度大于等于360，则不断减360直到它小于360
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}