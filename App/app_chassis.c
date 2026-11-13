/**
  ******************************************************************************
  * @file    app_chassis.c
  * @brief   麦轮底盘：航向闭环 + 平移 + 回到上电原点
  *
  *   运动学（底盘坐标系：x 前、y 右、ω 逆时针为正）：
  *     u_FL = vx - vy - k·ω        u_FR = vx + vy + k·ω
  *     u_RL = vx + vy - k·ω        u_RR = vx - vy + k·ω
  *     k = (轴距 + 轮距) / 2，u 为轮子线速度 (m/s)
  *
  *   本车电机为左右两侧镜像安装（实测），所以驱动板指令要再乘安装符号：
  *     c_i = mount_sign[i] × u_i
  *   mount_sign = (+,-,+,-) 是按实车标定的结果，见《底盘控制说明.md》第二节推导。
  *
  *   返航：位置误差 → 目标速度 → 叠加到 (vx, vy)，同时航向闭环继续工作。
  *
  *   总线时序：每台"发一帧 → 等它的应答"再发下一台（半双工，不能连发）。
  ******************************************************************************
  */
#include "app_chassis.h"
#include "main.h"
#include "bsp_rs485.h"
#include "bsp_debug.h"
#include "mks_protocol.h"
#include "hwt101.h"
#include "app_odom.h"
#include <math.h>

/* ===== 机械参数：实测后替换（单位 m）===== */
#define CHASSIS_WHEEL_R      0.03f    /* 麦轮半径 */
#define CHASSIS_WHEELBASE_L  0.16f    /* 前后轮中心距 */
#define CHASSIS_TRACK_W      0.20f    /* 左右轮中心距 */
#define CHASSIS_K            ((CHASSIS_WHEELBASE_L + CHASSIS_TRACK_W) / 2.0f)

/* 轮子线速度 (m/s) → 转速 (RPM)：rpm = v · 60 / (2πR) */
#define CHASSIS_RPM_PER_MPS  (60.0f / (2.0f * 3.14159265f * CHASSIS_WHEEL_R))

/* ===== 航向闭环参数 ===== */
#define CHASSIS_MSTEP        128u     /* 驱动板细分（必须和板子 MStep 设置一致） */
/*
 * 驱动板加减速参数 (0~255)。手册公式：每个速度单位 (256-acc)×50µs，
 * 128 细分下 1 个速度单位 = 0.125 RPM。
 *   acc=250 -> 300µs/单位 -> 约 417 RPM/s -> 折算成车身角速度只有 278 °/s²
 *   acc=255 ->  50µs/单位 -> 约 2500 RPM/s -> 约 1668 °/s²
 *
 * 为什么必须是 255：
 *   导航航向环是 P 控制 ω = 7×误差，要求执行机构减速度 > 7×ω。
 *   acc=250 时只有 278 °/s²，ω 超过 40°/s 就跟不上命令；
 *   从 120°/s 刹停要滑 120²/(2×278) ≈ 26° ——
 *   实测原地转 90° 超调到 101.6°、再回摆到 86.5°，就是这个原因。
 *   acc=255 后减速度 1668 °/s²，240°/s 以内都能精确跟随，
 *   加减速曲线完全由主机侧斜坡限幅器决定，驱动板变成"透明"执行器。
 */
#define CHASSIS_ACC          255u
#define CHASSIS_KP           0.8f     /* 航向误差 (°) → 目标角速度 (°/s) */
#define CHASSIS_KD           0.8f     /* 角速度阻尼 */
#define CHASSIS_OMEGA_MAX    180.0f   /* 自转角速度上限 (°/s) */
#define CHASSIS_DEADBAND_DEG 1.0f     /* 误差死区 (°) */

/*
 * 角速度指令变化率限幅 (°/s²)。
 *   vx/vy 一直有 HOME_ACC_MPS2 限幅，但 ω 以前是**直接**灌进运动学的：
 *   导航每 10 ms 用带噪声的 yaw 算一次 ω，跳变会原封不动甩给四台驱动板。
 *   acc=255 后驱动板能到 1668 °/s²，这里给 1600 让主机侧成为主导：
 *   既不拖慢刹车，也不会把 yaw 噪声直接甩下去。
 */
#define CHASSIS_OMEGA_ACC    1600.0f

/* 轮速死区 (RPM)：低于这个值直接发 0，避免方向位在 0 附近反复翻转 */
#define CHASSIS_RPM_DEADBAND 0.30f

/* ===== 返航参数 ===== */
/*
 * 位置环用 PD：
 *   v = KP·位置误差 − KD·实测速度
 * 只用比例项时，接近原点仍以全速前进，命令归零后靠惯性滑行（250mm/s 时约 100mm），
 * 远超容差 → 冲过头 → 再次触发 → 前后晃动。加速度阻尼项后末端是"主动刹车"而不是滑行。
 */
#define HOME_KP              2.5f     /* 位置误差 (m) → 速度 (m/s) */
#define HOME_KD              2.0f     /* 速度阻尼（速度由里程计差分估计） */
#define HOME_V_MAX_MPS       0.25f    /* 平移速度上限 (m/s) */
#define HOME_V_MIN_MPS       0.05f    /* 平移最小速度 (m/s)，防静摩擦推不动 */
#define HOME_MIN_APPLY_MM    60.0f    /* 只有距原点超过该值才施加最小速度，末端不用 */

/*
 * 前后方向(vx)的符号修正。
 *   标定判据：发 Act_Move(100,0,0)（只给前后速度，不给导航），
 *     车应朝车头方向前进、且串口打印的 x 增大。
 *   · 车朝前走、x 却减小        → 是**里程计**符号问题，
 *                                 改 app_odom.c 的 ODOM_X_SIGN 为 -1.0f，这里保持 +1
 *   · 车往后退、x 增大          → 是**指令**符号问题，把这里改成 -1.0f
 * 注意这只翻前后，不影响横移(vy)和旋转(ω) —— 那两路已验证正确。
 */
#define CHASSIS_FWD_SIGN     (1.0f)
#define HOME_ACC_MPS2        0.5f     /* 指令速度最大变化率 (m/s²)，让起步/停车柔和 */
#define HOME_TOL_MM          5.0f     /* 到位判定：距原点小于该值即认为到达 (mm) */
#define HOME_TIMEOUT_MS      30000u   /* 返航超时 (ms) */

/*
 * 返航 = 纯手动功能。
 *   上电/运行中【不会】自动回原点（原来的 CHASSIS_AUTO_HOME 自动触发已删除）。
 *   只有显式调用 Chassis_ReturnToOrigin() 才会开始返航，
 *   回到 HOME_TOL_MM 内、里程计掉线、或超过 HOME_TIMEOUT_MS 时结束。
 */

/* ===== 总线与控制周期 ===== */
#define CHASSIS_PERIOD_MS    20u      /* 控制周期 (ms) */
#define CHASSIS_TX_TIMEOUT   20u      /* 单帧发送超时 (ms) */
/*
 * 等应答超时 (ms)。
 *   驱动板正常应答约 1ms（7 字节 @115200 ≈ 0.6ms + 芯片转向时间）。
 *   以前是 10ms × 4 台 = 40ms，把主循环硬拖到 24Hz —— 实测主循环周期约 41ms
 *   正好等于 4×10.6ms。降到 4ms 后最坏 4×4.6 = 18.4ms，仍在 20ms 周期内。
 *   ⚠ 若发现 g_chassis_online_count 长期为 0（说明驱动板应答真的慢），
 *     把它调回 6~8ms；这个超时只影响"在线标志"，不影响速度指令下发。
 */
#define CHASSIS_ACK_TIMEOUT  4u       /* 等应答超时 (ms) */
#define CHASSIS_FAIL_LIMIT   3u       /* 连续丢应答判离线次数 */

/*
 * 上电默认模式。
 *
 * ★现在默认 CHASSIS_MODE_VELOCITY：上电后车体速度目标全为 0，
 *   四台驱动板只会收到"速度 0"指令，**完全不会自己动**。
 *   （以前默认 AUTO，AUTO 会跑航向保持，用陀螺仪输出角速度去锁车头，
 *     看起来就像"上电自己动/自己回原点"。自动返航已在前一步删除。）
 *
 * 其它模式没有删、随时可用：
 *   · Act_Move() 会自己切到 VELOCITY（和以前一样）
 *   · 想要上电就锁航向：显式调 Chassis_SetMode(CHASSIS_MODE_AUTO)
 *   · 想开环标定：显式调 Chassis_SetMode(CHASSIS_MODE_OPEN)
 *
 * 标定提示：若改动接线或机械装配，先切 OPEN 再重新标定，
 * 否则运动学/符号不对时闭环会变成正反馈把车开走。
 */
#define CHASSIS_DEFAULT_MODE  CHASSIS_MODE_VELOCITY

/* UART5 状态打印周期 (ms)；置 0 关闭 */
#define CHASSIS_PRINT_PERIOD_MS   0u

/* ============ 两张按实车标定的表 ============ */

/* 表 1：轮位 → 驱动板地址（索引顺序固定 FL, FR, RL, RR）
   本车实测：1=左前、3=右前、4=左后、2=右后 */
static const uint8_t s_wheel_addr[CHASSIS_MOTOR_NUM] =
{
  0x01u,   /* 左前 FL */
  0x03u,   /* 右前 FR */
  0x04u,   /* 左后 RL */
  0x02u    /* 右后 RR */
};

/* 表 2：安装符号 —— 驱动板正方向对应"底盘前进"为 +1，相反为 -1
   本车左右两侧电机镜像安装，实测为 (+,-,+,-)。
   （推导：旋转指令若用标准 X 型符号会变成前后平移，见说明文档第三节）
   改接线/换电机后要重新标定：向前推车看四轮转向，或按开环模式逐轮核对。 */
static const int8_t s_mount_sign[CHASSIS_MOTOR_NUM] =
{
  -1,   /* 左前 FL */
  +1,   /* 右前 FR */
  -1,   /* 左后 RL */
  +1    /* 右后 RR */
};

/* ==================================================================== */

static float   s_target_yaw    = 0.0f;
static uint8_t s_target_locked = 0u;
static float    s_odom_x_last = 0.0f;   /* 上一拍里程计位置，用于差分测速 */
static float    s_odom_y_last = 0.0f;
static float    s_vel_x       = 0.0f;   /* 车体实测速度 (m/s)，一阶低通后 */
static float    s_vel_y       = 0.0f;
static float    s_vx_cmd      = 0.0f;   /* 平滑后的平移指令 (m/s) */
static float    s_vy_cmd      = 0.0f;
static float    s_omega_cmd   = 0.0f;   /* 平滑后的自转指令 (°/s) */

/* 速度模式（CHASSIS_MODE_VELOCITY）的目标，由 Chassis_SetVelocity 写入 */
static float    s_cmd_vx      = 0.0f;   /* m/s，前进为正 */
static float    s_cmd_vy      = 0.0f;   /* m/s，右移为正 */
static float    s_cmd_w       = 0.0f;   /* °/s，逆时针为正 */

static uint8_t s_fail_cnt[CHASSIS_MOTOR_NUM] = {0};

volatile uint8_t g_chassis_mode      = (uint8_t)CHASSIS_DEFAULT_MODE;
volatile uint8_t g_chassis_ready     = 0u;
volatile float   g_chassis_yaw_err   = 0.0f;
volatile float   g_chassis_omega     = 0.0f;
volatile uint8_t g_chassis_homing    = 0u;
volatile uint8_t g_chassis_home_done = 0u;
volatile uint8_t g_chassis_home_abort = 0u;
volatile float   g_chassis_home_mm   = 0.0f;
volatile float   g_chassis_vx        = 0.0f;
volatile float   g_chassis_vy        = 0.0f;
volatile uint8_t g_chassis_enable_ok[CHASSIS_MOTOR_NUM] = {0};
volatile uint8_t g_chassis_online[CHASSIS_MOTOR_NUM]    = {0};
volatile uint8_t g_chassis_online_count = 0u;
volatile uint8_t g_chassis_fault_mask   = 0u;
volatile float   g_chassis_openloop[CHASSIS_MOTOR_NUM] = {0.0f};

static uint32_t s_home_start_ms = 0u;

/* 角度归一化到 ±180 */
static float Chassis_Wrap180(float deg)
{
  while (deg > 180.0f)
  {
    deg -= 360.0f;
  }
  while (deg < -180.0f)
  {
    deg += 360.0f;
  }
  return deg;
}

static float Chassis_Clamp(float v, float lo, float hi)
{
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

/* 周期打印状态，便于不用调试器也能判断（默认关闭） */
static void Chassis_ReportStatus(float err, float omega)
{
#if (CHASSIS_PRINT_PERIOD_MS > 0u)
  static uint32_t last_ms = 0u;

  if ((HAL_GetTick() - last_ms) >= CHASSIS_PRINT_PERIOD_MS)
  {
    last_ms = HAL_GetTick();
    Debug_Printf("[CHS] md=%u err=%6.1f om=%6.1f vx=%6.0f vy=%6.0f hm=%u d=%5.0f ol=%u rd=%u\r\n",
                 (unsigned)g_chassis_mode, (double)err, (double)omega,
                 (double)g_chassis_vx, (double)g_chassis_vy,
                 (unsigned)g_chassis_homing, (double)g_chassis_home_mm,
                 (unsigned)g_chassis_online_count, (unsigned)g_chassis_ready);
  }
#else
  (void)err;
  (void)omega;
#endif
}

/*
 * 按轮位给对应驱动板下发速度，并等它应答。
 * 返回值表示驱动板是否应答（用于判在线），不表示指令是否生效。
 */
static bool Chassis_SendWheel(uint8_t wheel, float rpm)
{
  uint8_t  frame[12];
  uint8_t  data;
  uint8_t  addr = s_wheel_addr[wheel];
  uint16_t len;

  /* 死区：极小速度直接发 0，避免方向位在 0 附近反复翻转（换向抖动） */
  if (fabsf(rpm) < CHASSIS_RPM_DEADBAND)
  {
    rpm = 0.0f;
  }

  /* 用浮点版组帧：按"速度值"精度取整（128 细分时 1 单位 = 0.125 RPM），
   * 不再先量化到 1 RPM（那样会一次跳 8 个单位，低速时四轮不协调） */
  len = MKS_BuildSpeedF(addr, rpm, CHASSIS_MSTEP, CHASSIS_ACC, frame);

  RS485_FlushRx();                       /* 先丢掉总线上的残留字节 */
  if (!RS485_Send(frame, len, CHASSIS_TX_TIMEOUT))
  {
    return false;
  }
  return RS485_WaitAck(addr, MKS_FC_SPEED, &data, CHASSIS_ACK_TIMEOUT);
}

void Chassis_Init(void)
{
  uint8_t  frame[12];
  uint8_t  data;
  uint16_t len;

  RS485_Init();

  /* 逐台使能（串口控制模式下使能由指令控制，不受 En 引脚影响） */
  for (uint8_t w = 0u; w < CHASSIS_MOTOR_NUM; w++)
  {
    len = MKS_BuildEnable(s_wheel_addr[w], true, frame);

    RS485_FlushRx();
    if (RS485_Send(frame, len, CHASSIS_TX_TIMEOUT) &&
        RS485_WaitAck(s_wheel_addr[w], MKS_FC_ENABLE, &data, 50u))
    {
      g_chassis_enable_ok[w] = 1u;
      g_chassis_online[w]    = 1u;
    }
    else
    {
      g_chassis_enable_ok[w] = 0u;
      g_chassis_online[w]    = 0u;
    }
    s_fail_cnt[w] = 0u;
  }

  /* 目标航向等陀螺仪第一帧有效数据时再锁定（保证"上电朝向 = 0°"） */
  s_target_yaw    = 0.0f;
  s_target_locked = 0u;

  g_chassis_mode       = (uint8_t)CHASSIS_DEFAULT_MODE;
  g_chassis_omega      = 0.0f;
  g_chassis_yaw_err    = 0.0f;
  g_chassis_homing     = 0u;
  g_chassis_home_done  = 0u;
  g_chassis_home_abort = 0u;
  g_chassis_vx         = 0.0f;
  g_chassis_vy         = 0.0f;
}

void Chassis_Task(void)
{
  static uint32_t last_ms = 0u;
  HWT101_Data_t   imu;
  float           err    = 0.0f;
  float           omega  = 0.0f;
  float           vx     = 0.0f;   /* m/s，车体坐标：前为正 */
  float           vy     = 0.0f;   /* m/s，车体坐标：右为正 */
  float           vx_target = 0.0f;   /* 位置环算出的目标速度，之后过限幅 */
  float           vy_target = 0.0f;
  float           dt_s;
  float           max_dv;
  float           w_rad;
  float           u[CHASSIS_MOTOR_NUM];
  float           cmd;
  uint8_t         online = 0u;
  uint8_t         fault  = 0u;
  uint8_t         stop   = 0u;

  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_ms) < CHASSIS_PERIOD_MS)
  {
    return;
  }
  dt_s   = (float)(now_ms - last_ms) / 1000.0f;
  last_ms = now_ms;

  /* ---- 用里程计差分估计车体实测速度（一阶低通，抑制测量轮噪声）---- */
  {
    float raw_vx = (g_odom_x_mm - s_odom_x_last) / 1000.0f / dt_s;
    float raw_vy = (g_odom_y_mm - s_odom_y_last) / 1000.0f / dt_s;

    s_odom_x_last = g_odom_x_mm;
    s_odom_y_last = g_odom_y_mm;
    s_vel_x += 0.35f * (raw_vx - s_vel_x);
    s_vel_y += 0.35f * (raw_vy - s_vel_y);
  }

  if (g_chassis_mode == (uint8_t)CHASSIS_MODE_VELOCITY)
  {
    /* ---- 速度模式：直接用车体速度目标，不做航向闭环、不做返航 ---- */
    vx_target = s_cmd_vx;          /* m/s */
    vy_target = s_cmd_vy;
    omega     = s_cmd_w;           /* °/s */
  }
  else if (g_chassis_mode == (uint8_t)CHASSIS_MODE_OPEN)
  {
    /* 开环：下面直接用 g_chassis_openloop[w] */
  }
  else
  {
    /* ---- 航向闭环（需要陀螺仪） ---- */
    if (!HWT101_GetData(&imu) || !HWT101_IsOnline())
    {
      g_chassis_ready = 0u;
      stop = 1u;
    }
    else
    {
      g_chassis_ready = 1u;

      if (s_target_locked == 0u)
      {
        s_target_yaw    = Chassis_Wrap180(imu.yaw);
        s_target_locked = 1u;
      }

      err = Chassis_Wrap180(s_target_yaw - imu.yaw);
      if (fabsf(err) < CHASSIS_DEADBAND_DEG)
      {
        err = 0.0f;
      }
      omega = Chassis_Clamp((CHASSIS_KP * err) - (CHASSIS_KD * imu.gz),
                            -CHASSIS_OMEGA_MAX, CHASSIS_OMEGA_MAX);

      /* ---- 返航：位置闭环（需要两轴里程计） ----
       * ★自动回原点已删除：这里不再有任何自发触发。
       *   只有显式调用 Chassis_ReturnToOrigin() 才会进入下面的闭环。 */
      if (g_chassis_homing != 0u)
      {
        if ((g_odom_online[0] == 0u) || (g_odom_online[1] == 0u))
        {
          g_chassis_homing    = 0u;      /* 里程计掉线：放弃返航，避免盲走 */
          g_chassis_home_abort = 1u;
        }
        else if ((HAL_GetTick() - s_home_start_ms) > HOME_TIMEOUT_MS)
        {
          g_chassis_homing    = 0u;      /* 超时 */
          g_chassis_home_abort = 1u;
        }
        else
        {
          float ex = g_odom_x_mm;        /* 目标就是原点 (0,0) */
          float ey = g_odom_y_mm;

          g_chassis_home_mm = sqrtf((ex * ex) + (ey * ey));

          if (g_chassis_home_mm < HOME_TOL_MM)
          {
            g_chassis_homing    = 0u;    /* 到位 */
            g_chassis_home_done = 1u;
          }
          else
          {
            /* 误差 (mm) → 速度 (m/s)：朝原点方向走 */
            /* PD：比例项把车拉回原点，阻尼项用实测速度做主动刹车，避免冲过头 */
            vx_target = (-HOME_KP * ex / 1000.0f) - (HOME_KD * s_vel_x);
            vy_target = (-HOME_KP * ey / 1000.0f) - (HOME_KD * s_vel_y);
            vx_target = Chassis_Clamp(vx_target, -HOME_V_MAX_MPS, HOME_V_MAX_MPS);
            vy_target = Chassis_Clamp(vy_target, -HOME_V_MAX_MPS, HOME_V_MAX_MPS);

            /* 最小速度只在离原点还远时施加（防静摩擦），末端不加，免得蹭动 */
            if (g_chassis_home_mm > HOME_MIN_APPLY_MM)
            {
              if ((vx_target > -HOME_V_MIN_MPS) && (vx_target < HOME_V_MIN_MPS) && (vx_target != 0.0f))
              {
                vx_target = (vx_target > 0.0f) ? HOME_V_MIN_MPS : -HOME_V_MIN_MPS;
              }
              if ((vy_target > -HOME_V_MIN_MPS) && (vy_target < HOME_V_MIN_MPS) && (vy_target != 0.0f))
              {
                vy_target = (vy_target > 0.0f) ? HOME_V_MIN_MPS : -HOME_V_MIN_MPS;
              }
            }
          }
        }
      }
    }
  }

  /*
   * 速度指令变化率限幅：起步和停住都按 HOME_ACC_MPS2 渐变，
   * 目标为 0（到位/取消返航）时也会在这里平滑地收回去，而不是瞬间断掉。
   */
  max_dv = HOME_ACC_MPS2 * dt_s;
  if (vx_target > (s_vx_cmd + max_dv))      { s_vx_cmd += max_dv; }
  else if (vx_target < (s_vx_cmd - max_dv)) { s_vx_cmd -= max_dv; }
  else                                      { s_vx_cmd = vx_target; }
  if (vy_target > (s_vy_cmd + max_dv))      { s_vy_cmd += max_dv; }
  else if (vy_target < (s_vy_cmd - max_dv)) { s_vy_cmd -= max_dv; }
  else                                      { s_vy_cmd = vy_target; }

  vx = CHASSIS_FWD_SIGN * s_vx_cmd;   /* 前后方向单独可翻（判据见宏定义处） */
  vy = s_vy_cmd;

  /* ω 也做变化率限幅：命令跳变不再直接甩到驱动板上 */
  {
    float max_dw = CHASSIS_OMEGA_ACC * dt_s;

    if (omega > (s_omega_cmd + max_dw))      { s_omega_cmd += max_dw; }
    else if (omega < (s_omega_cmd - max_dw)) { s_omega_cmd -= max_dw; }
    else                                     { s_omega_cmd = omega; }
    omega = s_omega_cmd;
  }

  g_chassis_yaw_err = err;
  g_chassis_omega   = omega;
  g_chassis_vx      = vx * 1000.0f;   /* 记录成 mm/s 便于观察 */
  g_chassis_vy      = vy * 1000.0f;

  /* ---- 底盘运动学：vx, vy, ω → 四轮线速度 → 转速 → 驱动板指令 ---- */
  w_rad = omega * 3.14159265f / 180.0f;
  u[0] = vx - vy - (CHASSIS_K * w_rad);   /* FL */
  u[1] = vx + vy + (CHASSIS_K * w_rad);   /* FR */
  u[2] = vx + vy - (CHASSIS_K * w_rad);   /* RL */
  u[3] = vx - vy + (CHASSIS_K * w_rad);   /* RR */

  /* ---- 逐台下发并核对应答 ---- */
  for (uint8_t w = 0u; w < CHASSIS_MOTOR_NUM; w++)
  {
    if (g_chassis_mode == (uint8_t)CHASSIS_MODE_OPEN)
    {
      cmd = g_chassis_openloop[w];
    }
    else if (stop != 0u)
    {
      cmd = 0.0f;
    }
    else
    {
      cmd = (float)s_mount_sign[w] * u[w] * CHASSIS_RPM_PER_MPS;
    }

    if (Chassis_SendWheel(w, cmd))
    {
      s_fail_cnt[w] = 0u;
      g_chassis_online[w] = 1u;
    }
    else
    {
      if (s_fail_cnt[w] < 255u)
      {
        s_fail_cnt[w]++;
      }
      if (s_fail_cnt[w] >= CHASSIS_FAIL_LIMIT)
      {
        g_chassis_online[w] = 0u;
      }
    }

    if (g_chassis_online[w] != 0u)
    {
      online++;
    }
    else
    {
      fault |= (uint8_t)(1u << w);
    }
  }

  g_chassis_online_count = online;
  g_chassis_fault_mask   = fault;

  Chassis_ReportStatus(err, omega);
}

void Chassis_SetTargetYaw(float deg)
{
  s_target_yaw    = Chassis_Wrap180(deg);
  s_target_locked = 1u;
}

float Chassis_GetTargetYaw(void)
{
  return s_target_yaw;
}

float Chassis_GetYawError(void)
{
  return g_chassis_yaw_err;
}

void Chassis_ReturnToOrigin(void)
{
  g_chassis_homing     = 1u;
  g_chassis_home_done  = 0u;
  g_chassis_home_abort = 0u;
  s_home_start_ms      = HAL_GetTick();
}

void Chassis_CancelReturn(void)
{
  g_chassis_homing = 0u;
}

bool Chassis_IsReturning(void)
{
  return (g_chassis_homing != 0u);
}

float Chassis_GetHomeDistance(void)
{
  return g_chassis_home_mm;
}

void Chassis_SetMode(Chassis_Mode_t mode)
{
  g_chassis_mode = (uint8_t)mode;

  /* 切模式时把所有"目标"清零，避免切过去就冲出去 */
  s_cmd_vx    = 0.0f;
  s_cmd_vy    = 0.0f;
  s_cmd_w     = 0.0f;
  s_vx_cmd    = 0.0f;
  s_vy_cmd    = 0.0f;
  s_omega_cmd = 0.0f;

  if (mode == CHASSIS_MODE_OPEN)
  {
    g_chassis_homing = 0u;
    for (uint8_t w = 0u; w < CHASSIS_MOTOR_NUM; w++)
    {
      g_chassis_openloop[w] = 0.0f;
    }
  }
  else if (mode == CHASSIS_MODE_VELOCITY)
  {
    g_chassis_homing = 0u;
    for (uint8_t w = 0u; w < CHASSIS_MOTOR_NUM; w++)
    {
      g_chassis_openloop[w] = 0.0f;
    }
  }
  else
  {
    s_target_yaw    = 0.0f;             /* 切回闭环时重新取当前朝向为 0° */
    s_target_locked = 0u;
  }
}

void Chassis_SetVelocity(float vx_mms, float vy_mms, float w_dps)
{
  /* 只在速度模式下有意义；别的模式下记下来但不生效 */
  s_cmd_vx = vx_mms * 0.001f;           /* mm/s -> m/s */
  s_cmd_vy = vy_mms * 0.001f;
  s_cmd_w  = w_dps;
}

void Chassis_SetOpenLoop(const float wheel_rpm[CHASSIS_MOTOR_NUM])
{
  if (wheel_rpm == NULL)
  {
    return;
  }
  for (uint8_t w = 0u; w < CHASSIS_MOTOR_NUM; w++)
  {
    g_chassis_openloop[w] = wheel_rpm[w];
  }
}

void Chassis_Stop(void)
{
  g_chassis_homing = 0u;

  /* 把速度模式的目标也清掉，否则下一拍 Chassis_Task 又会开回去 */
  s_cmd_vx    = 0.0f;
  s_cmd_vy    = 0.0f;
  s_cmd_w     = 0.0f;
  s_vx_cmd    = 0.0f;
  s_vy_cmd    = 0.0f;
  s_omega_cmd = 0.0f;

  for (uint8_t w = 0u; w < CHASSIS_MOTOR_NUM; w++)
  {
    g_chassis_openloop[w] = 0.0f;
    (void)Chassis_SendWheel(w, 0.0f);
  }
  g_chassis_omega = 0.0f;
  g_chassis_vx    = 0.0f;
  g_chassis_vy    = 0.0f;
}
