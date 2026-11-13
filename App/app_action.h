/**
  ******************************************************************************
  * @file    app_action.h
  * @brief   原子动作接口（纯执行层）
  *
  *   设计约定：
  *     · 这里只有"发起一个动作"和"查询动作结果"，**不含任何流程/策略**
  *     · 没有超时兜底、没有重试、没有降级、没有看门狗、没有自动停车
  *     · 什么时候调用、失败了怎么办、要不要重试，全部由你的规划代码决定
  *
  *   动作都是"两段式"：
  *     1) 调 Act_XXX(...)  发起，立即返回
  *     2) 每 tick 查 Act_XXXResult()，RUNNING 就等着，DONE/FAIL 才往下走
  *
  *   ★唯一需要注意的：MKS 驱动板是速度模式，主机不发指令它就保持上一次速度。
  *     所以"不动了"必须由你的代码显式调 Act_Stop()，否则车会一直跑。
  *
  *   ★现状：导航 / 机械臂 / 视觉 的底层模块还没有，对应函数是空壳，
  *     调用会打印"未实现"并返回 FAIL。位姿 / 夹爪 / 停车 / 日志 / 显示 是可用的。
  ******************************************************************************
  */
#ifndef APP_ACTION_H
#define APP_ACTION_H

#include <stdint.h>
#include <stdbool.h>

/* 区域编号（和你的标定表用同一套编号即可） */
typedef enum
{
  ACT_Z_QR = 0,
  ACT_Z_RAW,        /* 原料区（转盘） */
  ACT_Z_ROUGH,      /* 粗加工区 */
  ACT_Z_TEMP,       /* 暂存区 */
  ACT_Z_STACK,      /* 暂存区码垛层 */
  ACT_Z_HOME,       /* 启停区 */
  ACT_Z_NUM
} ActZone_t;

/* 视觉给出的一次性偏置与时机 */
typedef struct
{
  float dx;         /* mm */
  float dy;         /* mm */
  float eta_ms;     /* 预测还要多久才到抓取窗口；0=立刻 */
} ActBias_t;

/* 动作结果 */
typedef enum
{
  ACT_RUNNING = 0,  /* 还在执行 */
  ACT_DONE,         /* 完成 */
  ACT_FAIL          /* 失败（由实现方置位） */
} ActResult_t;

/* ==================== 系统 ==================== */
void       Act_Init(void);
void       Act_Task(void);           /* 主循环里周期调用，推进各动作 */
uint32_t   Act_Now(void);            /* 毫秒时基 */
void       Act_Log(const char *fmt, ...);

/* ==================== 导航 ==================== */

/**
  * @brief 直接给车体速度（**立刻生效，不走导航、不做航向闭环**）
  * @param vx_mms 前后速度，前进为正 (mm/s)
  * @param vy_mms 左右速度，**左移为正** (mm/s)
  * @param w_dps  自转角速度，逆时针为正 (°/s)
  * @note  ★第一次调用会自动把底盘切到速度模式。
  *         速度变化率内部有限幅（约 0.8 m/s²），不会猛冲。
  *         这是"持续保持"的速度，不会自己变 ACT_DONE；要停调 Act_Stop()。
  */
void       Act_Move(float vx_mms, float vy_mms, float w_dps);

/** @brief 发起"走到世界坐标 (x,y) 并转到 yaw"
 *  @param tmo_ms 0 = 不限时（自己算好时间就填 0） */
void       Act_NavGoto(float x_mm, float y_mm, float yaw_deg, uint32_t tmo_ms);
ActResult_t Act_NavResult(void);

/** @brief 立刻停住四个轮子（MKS 速度模式必须显式停） */
void       Act_Stop(void);

/** @brief 取世界坐标：x 前 / y 左（相对上电位置），yaw 连续角度（°，逆时针正） */
void       Act_PoseGet(float *x_mm, float *y_mm, float *yaw_deg);

/* ==================== 机械臂 ==================== */

/** @brief 臂舵机数量（ARM0~ARM3） */
#define ACT_ARM_NUM      4u

/**
  * @brief  读某个臂舵机的当前位置（单位：度，0~360）
  * @param  idx 0~3 对应 ARM0(0xA0) / ARM1(0xA1) / ARM2(0xA2) / ARM3(0xA3)
  * @param  deg 输出角度
  * @retval true  = 已收到该舵机的回传
  *         false = idx 越界，或该舵机还没回过帧
  *                 （上电后约 30ms 才有第一帧；地址不对/没接线则一直 false）
  * @note   位置来自驱动板的周期回传，由 ServoBus_Poll() 在主循环里刷新。
  *         要一次读 4 个，循环调用即可：
  *             float a[ACT_ARM_NUM];
  *             for (uint8_t i = 0; i < ACT_ARM_NUM; i++) { (void)Act_ArmAngle(i, &a[i]); }
  */
bool Act_ArmAngle(uint8_t idx, float *deg);

/** @brief 夹爪舵机当前角度（度）。false = 还没收到回传 */
bool Act_GripAngle(float *deg);

/** @brief 累计收到的舵机回传帧数。上电后一直为 0 = 回传没开或 CAN 接线不对 */
uint32_t Act_ArmRxCount(void);

/* ---- 动作 ---- */

/**
  * @brief  手动指定 4 个关节的角度，让机械臂转过去
  * @param  deg[0..3] 对应 ARM0~ARM3 的目标角度，单位：度（0~360）
  *                   越界会被底层自动夹到合法范围
  * @param  speed     速度 0~255（0x80 约中速，0xFF 最快）
  * @note   ★**发完立刻返回**，不等舵机到位。
  *         要判断到位请轮询 Act_ArmAtTarget()，或者自己延时。
  *   ★可以每拍调用：内部会记住上次下发的角度，
  *     只有变化超过 0.2° 的关节才真的发帧（避免刷爆 CAN 总线）。
  *     想强制重发同一个角度（比如被手掰过），先把数组改成别的值再改回来。
  *
  * @code
  *   float a[ACT_ARM_NUM] = { 90.0f, 45.0f, 120.0f, 0.0f };
  *   Act_ArmSetAngles(a, 0x80u);
  * @endcode
  */
void Act_ArmSetAngles(const float deg[ACT_ARM_NUM], uint8_t speed);

/**
  * @brief  4 个关节是否都已经到达上次 Act_ArmSetAngles() 设定的目标
  * @param  tol_deg 容差（度），常用 2~5
  * @retval false = 还没到、没设过目标、或某个关节收不到位置回传
  * @note   依赖舵机的周期位置回传（ServoBus_Poll 在主循环里刷新）。
  */
bool Act_ArmAtTarget(float tol_deg);

/** @brief 移到某个环上方（bias 可为 NULL；stack=1 表示码垛层） */
void       Act_ArmMove(ActZone_t zone, int8_t ring, const ActBias_t *bias, uint8_t stack);
/** @brief 垂直下放到该环的放/取高度（只走 Z） */
void       Act_ArmDescend(ActZone_t zone, int8_t ring, uint8_t stack);
/** @brief 垂直抬起 mm 毫米 */
void       Act_ArmLift(float mm);
ActResult_t Act_ArmResult(void);

/** @brief 夹爪 */
void       Act_GripOpen(void);
void       Act_GripClose(void);
uint8_t    Act_GripState(void);      /* 0=松开 1=夹紧 2=夹空 */

/* ==================== 视觉（两段式） ==================== */
bool       Act_VisCodeStart(void);
bool       Act_VisCodePoll(char out[20]);              /* out 形如 "156+123+516+231" */

bool       Act_VisBiasStart(ActZone_t zone, uint8_t idx);
bool       Act_VisBiasPoll(ActBias_t *out);

bool       Act_VisVerifyStart(ActZone_t zone, int8_t ring);
bool       Act_VisVerifyPoll(uint8_t *ok);              /* 1 = 确实放进去了 */

/* ==================== 显示 ==================== */
void       Act_DispCode(const char *code);
void       Act_DispCounts(uint16_t a, uint16_t b);
void       Act_DispMsg(const char *msg);

/* ==================== 调试开关 ==================== */
/** 1 = 打印每个动作；0 = 静音 */
extern volatile uint8_t g_act_verbose;

#endif /* APP_ACTION_H */
