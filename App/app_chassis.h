/**
  ******************************************************************************
  * @file    app_chassis.h
  * @brief   麦轮底盘运动控制
  *
  *   三个反馈源：
  *     航向  -> HWT101 陀螺仪（保持/转到目标朝向）
  *     位移  -> 两个 AS5600 里程计（前后 X / 左右 Y），用于回到上电原点
  *     四轮  -> MKS 驱动板（速度模式）
  *
  *   四轮指令 = 底盘运动学(vx, vy, ω) → 安装符号 → 驱动板速度值
  ******************************************************************************
  */
#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdint.h>
#include <stdbool.h>

#define CHASSIS_MOTOR_NUM   4u    /* 轮位顺序固定 FL, FR, RL, RR */

/** @brief 控制模式 */
typedef enum
{
  CHASSIS_MODE_AUTO = 0,   /* 航向闭环（默认）；返航开启时叠加位置闭环 */
  CHASSIS_MODE_OPEN        /* 开环：直接下发四轮转速，用于标定 */
} Chassis_Mode_t;

/** @brief 初始化：使能四台电机、以当前朝向为目标航向、里程原点清零 */
void  Chassis_Init(void);

/** @brief 控制任务，主循环反复调用（内部按固定周期执行） */
void  Chassis_Task(void);

/* ---- 朝向控制 ---- */

/** @brief 设置目标航向 (°)，自动归一化到 ±180；显式设置后不再自动取零 */
void  Chassis_SetTargetYaw(float deg);
float Chassis_GetTargetYaw(void);
float Chassis_GetYawError(void);

/* ---- 回到上电原点 ---- */

/**
  * @brief  启动返航：用两轴里程计反馈把车开回上电位置（里程计零点）
  * @note   返航过程中航向闭环同时工作（边走边保持上电时的朝向）；
  *         里程计掉线、超时或到达容差内会自动结束。
  */
void  Chassis_ReturnToOrigin(void);

/** @brief 取消返航（只保持航向，不再平移） */
void  Chassis_CancelReturn(void);

/** @brief 是否正在返航 */
bool  Chassis_IsReturning(void);

/** @brief 距上电原点的直线距离 (mm) */
float Chassis_GetHomeDistance(void);

/**
  * @brief  开/关自动回原点（位置保持）
  * @note   默认由 CHASSIS_AUTO_HOME 决定（1 = 上电即启用）：
  *         被移动偏离原点超过 HOME_RETRIGGER_MM 自动返航，
  *         回到 HOME_TOL_MM 内停止，可反复触发。
  */
void    Chassis_SetAutoHome(uint8_t enable);
uint8_t Chassis_GetAutoHome(void);

/* ---- 开环与停止 ---- */

void  Chassis_SetMode(Chassis_Mode_t mode);
void  Chassis_SetOpenLoop(const float wheel_rpm[CHASSIS_MOTOR_NUM]);
void  Chassis_Stop(void);

/* ---- 调试观察用 ---- */

extern volatile uint8_t  g_chassis_mode;        /* 0 = AUTO，1 = OPEN */
extern volatile uint8_t  g_chassis_ready;       /* 1 = 陀螺仪在线、航向闭环工作 */
extern volatile float    g_chassis_yaw_err;     /* 航向误差 (°) */
extern volatile float    g_chassis_omega;       /* 目标自转角速度 (°/s) */

extern volatile uint8_t  g_chassis_homing;      /* 1 = 正在返航 */
extern volatile uint8_t  g_chassis_home_done;   /* 1 = 已到达原点 */
extern volatile uint8_t  g_chassis_home_abort;  /* 1 = 返航被中止（里程计掉线/超时） */
extern volatile float    g_chassis_home_mm;     /* 距原点距离 (mm) */
extern volatile float    g_chassis_vx;          /* 目标前后速度 (mm/s)，前进为正 */
extern volatile float    g_chassis_vy;          /* 目标左右速度 (mm/s)，右移为正 */

extern volatile uint8_t  g_chassis_enable_ok[CHASSIS_MOTOR_NUM]; /* 上电使能应答 */
extern volatile uint8_t  g_chassis_online[CHASSIS_MOTOR_NUM];    /* 运行中是否在线 */
extern volatile uint8_t  g_chassis_online_count;
extern volatile uint8_t  g_chassis_fault_mask;
extern volatile float    g_chassis_openloop[CHASSIS_MOTOR_NUM];

#endif /* APP_CHASSIS_H */
