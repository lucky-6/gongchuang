#ifndef APP_PORTS_H
#define APP_PORTS_H

#include <stdint.h>
#include <stdbool.h>

/* 视觉给出的"一次性"偏置与时机 */
typedef struct
{
  float dx;
  float dy;
  float eta_ms;
} VisBias;

/* 区域编号（与标定表一致） */
typedef enum
{
  Z_QR = 0,
  Z_RAW,
  Z_ROUGH,
  Z_TEMP,
  Z_STACK,
  Z_HOME,
  Z_NUM
} ZoneId;

/**
 * @brief 业务层唯一感知的"外部世界"（端口 / ports）。
 *        业务逻辑只 include 这个头，绝不 include 任何 HAL / 驱动头。
 *        约定：所有 *_state() 返回 0=运行中 1=到位/成功 2=失败/超时。
 */
typedef struct
{
  /* ---------- 时间与日志（唯一时间源，便于仿真） ---------- */
  uint32_t (*now_ms)(void);
  void      (*log)(const char *fmt, ...);

  /* ---------- 底盘 ---------- */
  void      (*nav_goto)(float x_mm, float y_mm, float yaw_deg, uint32_t tmo_ms);
  uint8_t   (*nav_state)(void);
  void      (*chassis_stop)(void);
  void      (*pose_get)(float *x_mm, float *y_mm, float *yaw_deg);
  uint8_t   (*pose_bad)(void);          /* 1 = 位姿不可信 */

  /* ---------- 机械臂 ---------- */
  void      (*arm_move)(ZoneId zone, int8_t ring, const VisBias *bias, uint8_t stack);
  void      (*arm_descend)(ZoneId zone, int8_t ring, uint8_t stack);
  void      (*arm_lift)(float mm);
  void      (*grip_close)(void);
  void      (*grip_open)(void);
  uint8_t   (*grip_state)(void);        /* 0=松 1=紧 2=夹空 */
  uint8_t   (*arm_state)(void);

  /* ---------- 视觉（start / poll 两段式） ---------- */
  bool      (*vis_code_start)(void);
  bool      (*vis_code_poll)(char out[20]);
  bool      (*vis_bias_start)(ZoneId zone, uint8_t act);  /* act = 视觉台账唯一槽位 */
  bool      (*vis_bias_poll)(VisBias *out);
  bool      (*vis_verify_start)(ZoneId zone, int8_t ring);
  bool      (*vis_verify_poll)(uint8_t *ok);

  /* ---------- 显示 / 安全 ---------- */
  void      (*disp_code)(const char *code);
  void      (*disp_counts)(uint16_t pick, uint16_t place);
  void      (*disp_msg)(const char *msg);
  void      (*feed_motion)(void);       /* 有物理动作时喂"存活" */
  void      (*liveness_kick)(void);     /* 框架请求：做一次看得见的小动作 */
  void      (*estop)(uint8_t code);
} Ports;

extern const Ports *P;

#endif /* APP_PORTS_H */
