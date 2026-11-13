/**
  ******************************************************************************
  * @file    app_pose.c
  * @brief   里程计 + 陀螺仪融合 → 世界坐标
  *
  *   每 10 ms 做一次：
  *     1) 取体坐标位移增量 dxb, dyb（两个测量轮的弧长差）
  *     2) 取航向 yaw，算本拍的航向增量（处理 ±180 翻转）
  *     3) 用**中点航向**把体坐标增量旋转到世界系
  *     4) 累加
  *
  *   用中点航向是因为车是"边走边转"的：用起点或终点航向都会带
  *   ½·Δθ·ds 的一阶误差，中点法把它降成二阶，代价只是一次乘法。
  ******************************************************************************
  */
#include "app_pose.h"

#include "main.h"
#include "app_odom.h"
#include "hwt101.h"

#include <math.h>

/* ==================== 标定参数 ==================== */
/*
 * ★两个符号必须分开！
 *
 * POSE_YAW_SIGN   —— 航向角(yaw)的符号，用于世界坐标变换。
 *                    判据：把车拿起来，**用手向左(俯视逆时针)转 90°**，
 *                    串口打印的 yaw 应该是 +90。若显示 -90，改成 -1.0f。
 *
 * POSE_GYRO_SIGN  —— 陀螺角速度(gz)的符号，**只用于杠杆臂补偿**。
 *                    这个已由实测确认：用 +1.0f 时，原地转 360° 的
 *                    幻影圆从半径 30.7mm 压到 0.86mm（补偿有效率 97%）。
 *                    不要跟着上面那个一起改，否则补偿会变成"反向加倍误差"。
 */
#define POSE_YAW_SIGN        (+1.0f)
#define POSE_GYRO_SIGN       (+1.0f)

/* 刻度因子：真实位移 / 测得位移。直线推 2000mm 比对读数后填（1.00 = 不修正） */
#define POSE_K_X             (1.333f)
#define POSE_K_Y             (1.333f)

/* 测量轮偏心（mm）：X 轮的横向偏置 b、Y 轮的纵向偏置 a。
 *
 * ▶ 现象：车原地转 θ 时，轮子被拖着自己的圆弧，读出的弧长是 |b|·θ，
 *   代码误把它当成"底盘平移"，于是**纯旋转会画出一个圆**。
 *
 * ▶ 本例实测（从一次"原地转 360°"的日志最小二乘反解，模型：
 *        x(θ) = b·sinθ + a·(cosθ−1)
 *        y(θ) = b·(cosθ−1) − a·sinθ )
 *       补偿前：转一圈画半径 30.7mm 的圆，中途最大位置误差 59.5mm
 *       拟合残差 RMS 2.37mm（现象几乎被这两项完全解释）
 *
 * ▶ 注意：这两个值是在 POSE_K_X / POSE_K_Y 缩放**之后**测出来的。
 *   若以后改了 K，必须重新标定这两个数（它们会按 K 成比例变化）。
 *
 * ▶ 重新标定法（不需要尺子）：抬离地面、原地慢慢转满 360°，
 *   保存整段 x,y,yaw 日志，用上面两个公式做最小二乘，改这两个数即可。
 */
#define POSE_LEVER_B_MM      (-30.568f)  /* X 轮横向偏心（post-K 单位） */
#define POSE_LEVER_A_MM      (-3.244f)   /* Y 轮纵向偏心（post-K 单位） */

/* ==================== 内部参数 ==================== */
#define POSE_PERIOD_MS       10u
#define POSE_DEG2RAD         (3.14159265f / 180.0f)
#define POSE_VEL_LPF         0.30f      /* 世界速度一阶低通 */

/* ==================== 输出 ==================== */
volatile float g_pose_x_mm     = 0.0f;
volatile float g_pose_y_mm     = 0.0f;
volatile float g_pose_yaw_deg  = 0.0f;
volatile float g_pose_vx_mms   = 0.0f;
volatile float g_pose_vy_mms   = 0.0f;

/* ==================== 内部状态 ==================== */
static float s_odom_x_last = 0.0f;      /* 上一次的体坐标读数，用于取增量 */
static float s_odom_y_last = 0.0f;
static float s_yaw_last    = 0.0f;      /* 上一次的航向（原始，-180~180） */

static float Pose_Wrap180(float d)
{
  while (d > 180.0f)  { d -= 360.0f; }
  while (d < -180.0f) { d += 360.0f; }
  return d;
}

void Pose_Init(void)
{
  g_pose_x_mm    = 0.0f;
  g_pose_y_mm    = 0.0f;
  g_pose_yaw_deg = 0.0f;
  g_pose_vx_mms  = 0.0f;
  g_pose_vy_mms  = 0.0f;

  /* 以当前里程计和当前朝向为基准：上电位置 = 世界原点，上电朝向 = 0° */
  s_odom_x_last = AppOdom_GetX();
  s_odom_y_last = AppOdom_GetY();
  s_yaw_last    = HWT101_GetYaw() * POSE_YAW_SIGN;
}

void Pose_Reset(float x_mm, float y_mm, float yaw_deg)
{
  g_pose_x_mm    = x_mm;
  g_pose_y_mm    = y_mm;
  g_pose_yaw_deg = yaw_deg;
  g_pose_vx_mms  = 0.0f;
  g_pose_vy_mms  = 0.0f;

  /* 以当前里程计为新的起点，避免下一次更新带上历史位移 */
  s_odom_x_last = AppOdom_GetX();
  s_odom_y_last = AppOdom_GetY();
  s_yaw_last    = HWT101_GetYaw() * POSE_YAW_SIGN;
}

void Pose_Update(void)
{
  static uint32_t last_ms = 0u;
  float yaw_now, d_yaw, th_mid;
  float dxb, dyb, dxw, dyw;
  float c, s;
  float dt_s;
  uint32_t now;
  uint32_t elapsed;

  now = HAL_GetTick();
  elapsed = now - last_ms;
  if (elapsed < POSE_PERIOD_MS)
  {
    return;
  }
  last_ms = now;

  /*
   * ★必须用**实际经过的时间**，不能用标称周期。
   *   POSE_PERIOD_MS 只是"最快多久跑一次"的门限；主循环实际周期取决于
   *   Chassis_Task 等四台应答的耗时和串口打印，实测约 40 ms（≈24 Hz）。
   *   以前这里写死 10 ms，导致杠杆臂补偿只按 10/40 = 25% 施加，
   *   残余幻影正好是原来的 75%（实测反解 24.4%，完全吻合）。
   */
  dt_s = (float)elapsed * 0.001f;
  if ((dt_s <= 0.0f) || (dt_s > 0.5f))       /* 异常保护：退回标称值 */
  {
    dt_s = (float)POSE_PERIOD_MS * 0.001f;
  }

  /* ---------- 1) 体坐标位移增量 ---------- */
  dxb = AppOdom_GetX() - s_odom_x_last;
  dyb = AppOdom_GetY() - s_odom_y_last;
  s_odom_x_last = AppOdom_GetX();
  s_odom_y_last = AppOdom_GetY();

  /* 刻度修正在体坐标里做（若放到旋转后做会把两个轴混在一起） */
  dxb *= POSE_K_X;
  dyb *= POSE_K_Y;

  /* ---------- 2) 航向增量（处理 ±180 翻转） ---------- */
  yaw_now = HWT101_GetYaw() * POSE_YAW_SIGN;
  d_yaw   = Pose_Wrap180(yaw_now - s_yaw_last);
  g_pose_yaw_deg += d_yaw;                  /* 连续角度，不折叠 */

  /* ---------- 3) 杠杆臂补偿（测量轮不在旋转中心时，转起来会被读出假平移） ----------
   *   轮子测的是接触点速度分量：v_A = vx + ω·b，v_B = vy − ω·a
   *   所以 vx = v_A − ω·b，vy = v_B + ω·a
   *   ω 用陀螺的角速度 gz（°/s），换成 rad/s。
   */
  {
    float w = g_hwt101.gz * POSE_GYRO_SIGN * POSE_DEG2RAD;   /* rad/s */

    dxb -= w * POSE_LEVER_B_MM * dt_s;
    dyb += w * POSE_LEVER_A_MM * dt_s;
  }

  /* ---------- 4) 用中点航向把体坐标增量转到世界系并累加 ----------
   *   标准右手系旋转（x 前、y 左、θ 逆时针为正）：
   *       dx_w = cosθ·dx_b - sinθ·dy_b
   *       dy_w = sinθ·dx_b + cosθ·dy_b
   *
   *   ★以前这里用的是"y 右"的左手系形式（dx_w = c·dxb + s·dyb 那套），
   *     与实车方向约定相反，表现为：转 90° 之后所有平移的世界方向都镜像。
   *     实测证据：转 90° 后导航下发"向世界 +y 平移"，里程计报 y 一路涨到 91.7，
   *     而车实际退回了原点 —— 报告与物理运动相反，就是这里的旋转方向反了。
   *     θ=0 时两种形式结果相同，所以之前的横移测试发现不了。
   */
  th_mid = (s_yaw_last + (0.5f * d_yaw)) * POSE_DEG2RAD;
  c = cosf(th_mid);
  s = sinf(th_mid);

  dxw = (c * dxb) - (s * dyb);
  dyw = (s * dxb) + (c * dyb);

  g_pose_x_mm += dxw;
  g_pose_y_mm += dyw;

  /* 世界速度（低通，单位 mm/s），同样用实际 dt */
  g_pose_vx_mms += POSE_VEL_LPF * ((dxw / dt_s) - g_pose_vx_mms);
  g_pose_vy_mms += POSE_VEL_LPF * ((dyw / dt_s) - g_pose_vy_mms);

  s_yaw_last = yaw_now;
}

void Pose_Get(float *x_mm, float *y_mm, float *yaw_deg)
{
  if (x_mm)     { *x_mm = g_pose_x_mm; }
  if (y_mm)     { *y_mm = g_pose_y_mm; }
  if (yaw_deg)  { *yaw_deg = g_pose_yaw_deg; }
}

void Pose_GetVel(float *vx_mms, float *vy_mms)
{
  if (vx_mms) { *vx_mms = g_pose_vx_mms; }
  if (vy_mms) { *vy_mms = g_pose_vy_mms; }
}
