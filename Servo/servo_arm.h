/**
  ******************************************************************************
  * @file    servo_arm.h
  * @brief   四轴机械臂示教 / 回放（示教点方式），RAM 存储，掉电丢失
  *
  *   舵机：0xA0~0xA3（SERVO_ID_ARM0~3）
  *   按键：PC5 = 示教   长按 → 开始示教（舵机泄力，可手动摆位）
  *                      短按 → 记录当前姿态为一个示教点
  *                      长按 → 结束示教
  *         PC4 = 回放   短按 → 依次走到各示教点
  *         两个按键均为上拉输入，按下 = 低电平
  *
  *   重复精度：示教点存的是舵机原始 16 位位置值（0~4095），回放时原值透传、
  *             不做浮点换算，每个点都是绝对位置，因此误差不会逐点累积。
  ******************************************************************************
  */
#ifndef SERVO_ARM_H
#define SERVO_ARM_H

#include <stdint.h>

#define SERVO_ARM_SERVO_NUM       4u      /* 机械臂舵机数 */
#define SERVO_ARM_TEAM_NUM        8u      /* 动作组数 */
#define SERVO_ARM_POINT_MAX       64u     /* 每组最多示教点（占 512B/组） */

#define SERVO_ARM_LONG_PRESS_MS   800u    /* 长按判定阈值 */
#define SERVO_ARM_PLAY_SPEED      0xFFu   /* 回放速度：0x00~0xFF，越大越快，0xFF 最快 */

/*
 * 到位判定：靠舵机的周期位置回传（0x19）比较"当前位置 vs 目标位置"。
 * 残差 <= 容差即认为到位，不再靠固定延时猜时间。
 * 若位置回传没收到（示教时从来读不到位置），到位判定会一直不满足，
 * 最终由超时兜底，退化成"等到超时再走下一个点"。
 */
#define SERVO_ARM_ARRIVE_TOL         20u     /* 到位容差（原始位置单位，4096≈360°，20≈1.8°） */
#define SERVO_ARM_ARRIVE_TIMEOUT_MS  1000u   /* 单点到位超时，超时即继续下一个点 */
#define SERVO_ARM_HOLD_MS            2000u   /* 到达每个位置后保持的时间，再走下一个点 */

/*
 * 转向控制：0x03 指令只给目标位置，不给转向，走哪条弧由舵机固件决定。
 * 舵机默认按单向累加走，目标在当前值的"正向"超过 180° 时就会绕远路（优弧）。
 * 位置字段是 16 位而单圈只占低 12 位，高位通常表示圈数：
 * 发 -96 而不是 4000，就是让舵机"到 4000 那个位置，但反着走"。
 * 打开后每步都转成离当前位置最近的多圈值，即总是走劣弧（最短路径）。
 * 若舵机不支持多圈（内部对 16 位取模，行为不变；若做钳位则会停在圈边界），
 * 把它改回 0 即可恢复原行为。
 */
#define SERVO_ARM_MULTI_TURN         1u
#define SERVO_ARM_PROGRESS_DEBUG     0u      /* 1 = 打印等待期间的实时残差（调试用，会刷屏） */

/** @brief 初始化：配置按键 PC5/PC4，并给 4 个舵机开启周期位置回传 */
void ServoArm_Init(void);

/**
  * @brief 主循环调用：收位置回传 + 处理按键
  * @note  必须在主循环里高频调用，按键消抖与长按判定依赖它的调用间隔
  */
void ServoArm_Poll(void);

/**
  * @brief 回放：依次走到该动作组的各示教点（阻塞）
  * @param speed 0~255，点间由 SERVO_ARM_POINT_SETTLE_MS 控制到位等待
  */
void ServoArm_Play(uint8_t team_num, uint8_t speed);

/** @brief 取某动作组已记录的示教点数 */
uint16_t ServoArm_GetPointCount(uint8_t team_num);

/** @brief 选择当前动作组（默认 0），范围 0~SERVO_ARM_TEAM_NUM-1 */
void ServoArm_SetTeam(uint8_t team_num);

/** @brief 是否正在示教 */
uint8_t ServoArm_IsTeaching(void);

/** @brief 是否正在回放 */
uint8_t ServoArm_IsPlaying(void);

/**
  * @brief  该示教点的 4 个舵机是否都已到位（残差 <= SERVO_ARM_ARRIVE_TOL）
  * @note   数据源是周期回传的当前位置，可在任意时刻调用做进度查询
  */
uint8_t ServoArm_IsArrived(uint8_t team_num, uint16_t point_idx);

/** @brief 取某舵机最近一次回传的原始位置（0~4095），idx 0~3 对应 0xA0~0xA3 */
uint16_t ServoArm_GetPos(uint8_t servo_idx);

#endif /* SERVO_ARM_H */
