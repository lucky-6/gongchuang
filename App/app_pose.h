/**
  ******************************************************************************
  * @file    app_pose.h
  * @brief   里程计 + 陀螺仪融合 → 世界坐标（绝对坐标）
  *
  *   问题：两个测量轮装在车身上，测到的是**体坐标**位移（沿车身前后/左右）。
  *         车一转 90°，再沿原方向走，增量就从 X 轴跳到 Y 轴 —— 这不是绝对坐标。
  *
  *   做法：把体坐标位移用陀螺仪的航向旋转到世界系，再累加。
  *         世界系 = 上电那一刻的车身坐标系（上电朝向 = θ0，坐标原点 = 0,0）。
  *
  *   约定（与本工程 app_chassis.c 一致）：
  *     x 前、y 右、θ 逆时针(CCW)为正  →  地平面左手系，旋转取转置
  *         dx_w =  cosθ·dx_b + sinθ·dy_b
  *         dy_w = -sinθ·dx_b + cosθ·dy_b
  ******************************************************************************
  */
#ifndef APP_POSE_H
#define APP_POSE_H

#include <stdint.h>

/** @brief 初始化：记下里程计基准和初始航向（上电调用一次） */
void Pose_Init(void);

/** @brief 周期调用（内部 10 ms 分频），推进世界坐标 */
void Pose_Update(void);

/** @brief 把当前世界坐标强制设为给定值（例如摆到已知点后重新锚定） */
void Pose_Reset(float x_mm, float y_mm, float yaw_deg);

/** @brief 取世界坐标：x 前 / y 右（相对上电位置），yaw 连续角度（°） */
void Pose_Get(float *x_mm, float *y_mm, float *yaw_deg);

/** @brief 取世界系速度 (mm/s) */
void Pose_GetVel(float *vx_mms, float *vy_mms);

extern volatile float g_pose_x_mm;      /* 世界系 X (mm) */
extern volatile float g_pose_y_mm;      /* 世界系 Y (mm) */
extern volatile float g_pose_yaw_deg;   /* 连续航向 (°)，可超出 ±180 */
extern volatile float g_pose_vx_mms;    /* 世界系速度 (mm/s) */
extern volatile float g_pose_vy_mms;

#endif /* APP_POSE_H */
