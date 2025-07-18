#ifndef __POSITION_H__
#define __POSITION_H__


// UWB 实时坐标：通过串口中断实时更新
extern float NOW_x;
extern float NOW_y;
extern float NOW_z;

// 目的坐标
extern float TARGET_x;
extern float TARGET_y;
extern float TARGET_z;

// 导航状态枚举
typedef enum {
    NAV_IDLE,       // 空闲
    NAV_ROTATING,   // 旋转中
    NAV_MOVING,     // 移动中
    NAV_ARRIVED     // 已到达
} NavigationState_t;

// 导航状态
extern NavigationState_t nav_state;

// 定义一个二维坐标点结构体
typedef struct {
    float x;
    float y;
} Point_t;

// 角度偏差范围 单位：度
#define ANGLE_THRESHOLD 2
// 距离偏差范围 单位：cm
#define DISTANCE_THRESHOLD 10
// 角度稳定帧数：navigation_update 此函数执行一次为一帧
#define ROTATION_STABLE_FRAMES 5
// 距离稳定帧数：navigation_update 此函数执行一次为一帧
#define DISTANCE_STABLE_FRAMES 2

// ---------- 公共接口函数 ----------
void set_target_point(float x, float y);
void navigation_update(void);
void path_start(const Point_t path[], int num_points);
void path_update(void);

// ---------- 内部辅助函数 ----------
static float normalize_angle(float angle);

#endif