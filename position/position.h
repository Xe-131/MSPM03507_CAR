#ifndef __POSITION_H__
#define __POSITION_H__
#include <stdbool.h> // 使用 bool, true, false

// UWB 实时坐标：通过串口中断实时更新
extern float NOW_x;
extern float NOW_y;

extern float diff_x;
extern float diff_y;

#define NOW_x (uwb.x + diff_x)
#define NOW_y (uwb.y + diff_y)

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
#define ROTATION_STABLE_FRAMES 3
// 距离稳定帧数：navigation_update 此函数执行一次为一帧
#define DISTANCE_STABLE_FRAMES 5

// 目标点
#define POINT_1   (Point_t){80, 160}
#define POINT_2   (Point_t){160, 170}
#define POINT_3   (Point_t){230, 170}
#define POINT_4   (Point_t){230, 110}
#define POINT_5   (Point_t){230, 270}
#define POINT_6   (Point_t){230, 340}
#define POINT_7   (Point_t){230, 400}
#define POINT_8   (Point_t){140, 340}
#define POINT_9   (Point_t){80, 320}
#define POINT_10  (Point_t){30, 130}
// 路径
extern Point_t path_1[];
extern Point_t path_2[];
extern Point_t path_3[];
extern Point_t path_4[];
extern Point_t path_5[];
extern Point_t path_6[];

extern Point_t reverse_path_1[];
extern Point_t reverse_path_2[];
extern Point_t reverse_path_3[];
extern Point_t reverse_path_4[];
extern Point_t reverse_path_5[];
extern Point_t reverse_path_6[];
// ---------- 公共接口函数 ----------
void set_target_point(float x, float y);
void navigation_update(void);
void path_start(const Point_t path[], int num_points);
void path_update(void);
bool path_is_finished(void);
// ---------- 内部辅助函数 ----------
static float normalize_angle(float angle);

#endif