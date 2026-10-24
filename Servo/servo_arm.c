/**
  ******************************************************************************
  * @file    servo_arm.c
  * @brief   四轴机械臂示教 / 回放实现（示教点方式）
  ******************************************************************************
  */
#include "servo_arm.h"
#include "servo_bus.h"
#include "can.h"
#include "bsp_debug.h"

/* 机械臂 4 个舵机的 CAN ID（与 servo_bus.h 的 SERVO_ID_ARM0~3 一致） */
static const uint32_t s_arm_id[SERVO_ARM_SERVO_NUM] =
{
  0xA0u, 0xA1u, 0xA2u, 0xA3u
};

/* 一个示教点 = 4 个舵机的原始位置（0~4095） */
typedef struct
{
  uint16_t pos[SERVO_ARM_SERVO_NUM];
} ServoArmPoint_t;

static ServoArmPoint_t s_team[SERVO_ARM_TEAM_NUM][SERVO_ARM_POINT_MAX];
static uint16_t        s_point_cnt[SERVO_ARM_TEAM_NUM];

static uint16_t s_pos[SERVO_ARM_SERVO_NUM];   /* 各舵机当前原始位置 */
static uint32_t s_rx_count;                   /* 累计收到的有效位置回传帧数 */

/* 运行状态 */
static uint8_t s_team_cur;      /* 当前动作组号 */
static uint8_t s_teaching;      /* 示教中 */
static uint8_t s_playing;       /* 回放中 */

/* ---------------- 按键 ---------------- */

/* 上拉输入，按下 = 低电平 */
#define SERVO_ARM_TEACH_PIN    GPIO_PIN_5
#define SERVO_ARM_TEACH_PORT   GPIOC
#define SERVO_ARM_REPLAY_PIN   GPIO_PIN_4
#define SERVO_ARM_REPLAY_PORT  GPIOC

#define KEY_DEBOUNCE_MS        20u

#define KEY_EVENT_NONE         0u
#define KEY_EVENT_SHORT        1u
#define KEY_EVENT_LONG         2u

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       level;       /* 消抖后的稳定电平（0 = 按下） */
  uint8_t       raw_last;    /* 上次读到的原始电平 */
  uint32_t      t_raw;       /* 原始电平最近一次变化的时刻 */
  uint32_t      t_press;     /* 稳定按下的时刻 */
  uint8_t       down;        /* 当前处于稳定按下状态 */
  uint8_t       long_fired;  /* 本次按住是否已触发过长按 */
} ServoArmKey_t;

static ServoArmKey_t s_key_teach  = { SERVO_ARM_TEACH_PORT,  SERVO_ARM_TEACH_PIN,  1u, 1u, 0u, 0u, 0u, 0u };
static ServoArmKey_t s_key_replay = { SERVO_ARM_REPLAY_PORT, SERVO_ARM_REPLAY_PIN, 1u, 1u, 0u, 0u, 0u, 0u };

static void ServoArm_KeyReset(ServoArmKey_t *k, uint32_t now)
{
  k->level      = 1u;    /* 视为已松开，避免按键状态跨过阻塞段后被误判 */
  k->raw_last   = 1u;
  k->t_raw      = now;
  k->t_press    = now;
  k->down       = 0u;
  k->long_fired = 0u;
}

/**
  * @brief  按键扫描，返回 KEY_EVENT_NONE / SHORT / LONG
  * @note   长按在按住达到阈值时就地触发；短按在松开时判定，
  *         这样短按不会在刚按下就误触发。
  */
static uint8_t ServoArm_KeyScan(ServoArmKey_t *k, uint32_t now)
{
  uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 0u : 1u;

  /* 原始电平还在跳变：重新计时，暂不采信 */
  if (raw != k->raw_last)
  {
    k->raw_last = raw;
    k->t_raw    = now;
    return KEY_EVENT_NONE;
  }

  /* 电平尚未稳定够久 */
  if ((now - k->t_raw) < KEY_DEBOUNCE_MS)
  {
    return KEY_EVENT_NONE;
  }

  if (raw == k->level)
  {
    /* 电平稳定且未变：按住期间检查长按 */
    if (k->down && !k->long_fired && ((now - k->t_press) >= SERVO_ARM_LONG_PRESS_MS))
    {
      k->long_fired = 1u;
      return KEY_EVENT_LONG;
    }
    return KEY_EVENT_NONE;
  }

  /* 稳定电平发生变化 */
  k->level = raw;

  if (raw == 0u)
  {
    /* 按下 */
    k->down       = 1u;
    k->t_press    = now;
    k->long_fired = 0u;
    return KEY_EVENT_NONE;
  }

  /* 松开：未触发过长按才算短按 */
  {
    uint8_t was_down = k->down;
    uint8_t fired    = k->long_fired;

    k->down       = 0u;
    k->long_fired = 0u;

    return (was_down && !fired) ? KEY_EVENT_SHORT : KEY_EVENT_NONE;
  }
}

/* ---------------- 位置回读 ---------------- */

/* 从 CAN FIFO 取回传帧，更新对应舵机当前位置 */
static void ServoArm_UpdatePositions(void)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t  data[8];
  uint32_t id;

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) != HAL_OK)
    {
      break;
    }
    if (rx.DLC < 3u)
    {
      continue;
    }

    /*
     * 只按 CAN ID 匹配，不检查命令码 —— 与参考工程 Servo_position() 一致。
     * 之前限定 data[0] 必须是 0x02/0x1A，若舵机报告帧的命令码不是这两个值，
     * 回传会被全部滤掉，s_pos 恒为 0，录下来的示教点就是全零。
     */
    id = (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId;
    for (uint8_t i = 0u; i < SERVO_ARM_SERVO_NUM; i++)
    {
      if (id == s_arm_id[i])
      {
        uint16_t raw = (uint16_t)(((uint16_t)data[2] << 8) | data[1]);
        if (raw > SERVO_RAW_MAX)
        {
          raw = SERVO_RAW_MAX;
        }
        s_pos[i] = raw;
        s_rx_count++;
        break;
      }
    }
  }
}

/**
  * @brief  丢弃 FIFO 里的旧帧，再等一轮周期报告，保证 s_pos 是最新位置
  * @note   只在记录示教点时调用；阻塞约 35ms（29ms 报告周期 + 余量），
  *         目的是避免记到几十毫秒前的旧角度。
  */
static void ServoArm_RefreshPositions(void)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) != HAL_OK)
    {
      break;
    }
  }

  HAL_Delay(35u);
  ServoArm_UpdatePositions();
}

/* ---------------- 示教 ---------------- */

/* 开始示教：释放舵机力矩，清空当前动作组 */
static void ServoArm_TeachStart(void)
{
  uint8_t free_frame[8] = { SERVO_CMD_STOP_FREE, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

  for (uint8_t i = 0u; i < SERVO_ARM_SERVO_NUM; i++)
  {
    (void)ServoBus_SendRaw(s_arm_id[i], free_frame, 8u);
    HAL_Delay(2u);
  }

  s_point_cnt[s_team_cur] = 0u;
  s_teaching = 1u;

  Debug_Printf("[ARM] teach start team=%u (servos released, short-press PC5 to record)\r\n",
               (unsigned)s_team_cur);
}

/* 结束示教 */
static void ServoArm_TeachStop(void)
{
  s_teaching = 0u;

  Debug_Printf("[ARM] teach stop team=%u points=%u\r\n",
               (unsigned)s_team_cur, (unsigned)s_point_cnt[s_team_cur]);
}

/* 记录当前姿态为一个示教点 */
static void ServoArm_RecordPoint(void)
{
  uint16_t idx = s_point_cnt[s_team_cur];

  if (idx >= SERVO_ARM_POINT_MAX)
  {
    Debug_Printf("[ARM] point full (%u), ignored\r\n", (unsigned)SERVO_ARM_POINT_MAX);
    return;
  }

  /* 取最新位置再记，否则会记到几十毫秒前的旧角度 */
  ServoArm_RefreshPositions();

  for (uint8_t s = 0u; s < SERVO_ARM_SERVO_NUM; s++)
  {
    s_team[s_team_cur][idx].pos[s] = s_pos[s];
  }
  s_point_cnt[s_team_cur] = (uint16_t)(idx + 1u);

  Debug_Printf("[ARM] point %u: %u,%u,%u,%u\r\n",
               (unsigned)idx,
               (unsigned)s_pos[0], (unsigned)s_pos[1],
               (unsigned)s_pos[2], (unsigned)s_pos[3]);
}

/* ---------------- 转向控制 ---------------- */

#if (SERVO_ARM_MULTI_TURN != 0u)
/* 各舵机的连续指令位置（多圈）。每次回放开始时以实际回读位置为基准 */
static int32_t s_cmd_multi[SERVO_ARM_SERVO_NUM];

/**
  * @brief  把示教点的单圈值转成"离当前位置最近"的多圈值
  * @note   相邻两次的差值被归一化到 [-2048, 2048]，即最短路径（劣弧）。
  *         于是 100 -> 4000 会算成 -196（反向走近路），而不是 +3900（绕远路）。
  */
static int16_t ServoArm_ToNearestMulti(uint8_t s, uint16_t target_single)
{
  int32_t cur    = s_cmd_multi[s];
  int32_t cur_in = ((cur % 4096) + 4096) % 4096;   /* 当前指令位置在圈内的位置 */
  int32_t d      = (int32_t)target_single - cur_in;

  if (d > 2048)
  {
    d -= 4096;
  }
  else if (d < -2048)
  {
    d += 4096;
  }

  cur += d;

  /* 防止长期单向旋转把 16 位撑爆（±8 圈以外才触发，实际动作组到不了） */
  if (cur > 32767)
  {
    cur -= 4096;
  }
  else if (cur < -32768)
  {
    cur += 4096;
  }

  s_cmd_multi[s] = cur;
  return (int16_t)cur;
}
#endif

/* 取该舵机本步实际要发送的角度值 */
static int16_t ServoArm_CommandValue(uint8_t s, uint16_t target_single)
{
#if (SERVO_ARM_MULTI_TURN != 0u)
  return ServoArm_ToNearestMulti(s, target_single);
#else
  (void)s;
  return (int16_t)target_single;
#endif
}

/* ---------------- 到位判定 ---------------- */

/* 该点 4 个舵机里最大的残差（当前回传位置与目标位置之差，原始位置单位） */
static uint16_t ServoArm_PointMaxErr(uint8_t team_num, uint16_t point_idx)
{
  uint16_t max_err = 0u;

  for (uint8_t s = 0u; s < SERVO_ARM_SERVO_NUM; s++)
  {
    int32_t  d   = (int32_t)s_pos[s] - (int32_t)s_team[team_num][point_idx].pos[s];
    uint16_t err = (uint16_t)((d < 0) ? -d : d);

    if (err > max_err)
    {
      max_err = err;
    }
  }
  return max_err;
}

uint8_t ServoArm_IsArrived(uint8_t team_num, uint16_t point_idx)
{
  if ((team_num >= SERVO_ARM_TEAM_NUM) || (point_idx >= s_point_cnt[team_num]))
  {
    return 0u;
  }
  return (ServoArm_PointMaxErr(team_num, point_idx) <= SERVO_ARM_ARRIVE_TOL) ? 1u : 0u;
}

/**
  * @brief  等该点 4 个舵机都到位
  * @return 1 = 到位，0 = 超时
  * @note   位置回传不正常时残差永远不满足，靠 timeout_ms 兜底退出
  */
static uint8_t ServoArm_WaitArrive(uint8_t team_num, uint16_t point_idx, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
#if (SERVO_ARM_PROGRESS_DEBUG != 0u)
  uint32_t t_print = t0;
#endif

  for (;;)
  {
    ServoArm_UpdatePositions();

    if (ServoArm_IsArrived(team_num, point_idx))
    {
      return 1u;
    }

    if ((HAL_GetTick() - t0) >= timeout_ms)
    {
      return 0u;
    }

#if (SERVO_ARM_PROGRESS_DEBUG != 0u)
    if ((HAL_GetTick() - t_print) >= 100u)
    {
      t_print = HAL_GetTick();
      Debug_Printf("[ARM]   wait: max_err=%u\r\n",
                   (unsigned)ServoArm_PointMaxErr(team_num, point_idx));
    }
#endif

    HAL_Delay(5u);
  }
}

/* ---------------- 对外接口 ---------------- */

void ServoArm_Init(void)
{
  GPIO_InitTypeDef g = {0};
  uint8_t frame[3] = { SERVO_CMD_ENABLE_RSP, 0x1Du, 0x00u };

  /* 示教按键 PC5、回放按键 PC4：上拉输入 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;

  g.Pin = SERVO_ARM_TEACH_PIN;
  HAL_GPIO_Init(SERVO_ARM_TEACH_PORT, &g);

  g.Pin = SERVO_ARM_REPLAY_PIN;
  HAL_GPIO_Init(SERVO_ARM_REPLAY_PORT, &g);

  ServoArm_KeyReset(&s_key_teach,  HAL_GetTick());
  ServoArm_KeyReset(&s_key_replay, HAL_GetTick());

  s_team_cur = 0u;
  s_teaching = 0u;
  s_playing  = 0u;

  /* 给 4 个机械臂舵机开启周期位置回传（0x19 1D 00 = 29ms 周期） */
  for (uint8_t i = 0u; i < SERVO_ARM_SERVO_NUM; i++)
  {
    (void)ServoBus_SendRaw(s_arm_id[i], frame, 3u);
    HAL_Delay(5u);
  }
}

void ServoArm_Poll(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t  ev;

  /* 位置回读要先做，保证记录时 s_pos 是最新的 */
  ServoArm_UpdatePositions();

  /* PC5：长按开始/结束示教，短按记录一个示教点 */
  ev = ServoArm_KeyScan(&s_key_teach, now);
  if (ev == KEY_EVENT_LONG)
  {
    if (s_playing)
    {
      /* 回放中忽略，避免和阻塞的回放冲突 */
    }
    else if (s_teaching)
    {
      ServoArm_TeachStop();
    }
    else
    {
      ServoArm_TeachStart();
    }
  }
  else if (ev == KEY_EVENT_SHORT)
  {
    if (s_teaching && !s_playing)
    {
      ServoArm_RecordPoint();
    }
    /* 非示教状态下的短按无动作，避免误记 */
  }

  /* PC4：短按回放当前动作组（示教中不响应） */
  ev = ServoArm_KeyScan(&s_key_replay, now);
  if ((ev == KEY_EVENT_SHORT) && !s_teaching && !s_playing)
  {
    ServoArm_Play(s_team_cur, SERVO_ARM_PLAY_SPEED);

    /* 回放是阻塞的，期间累积的按键状态已过期，两个键都重新开始消抖 */
    ServoArm_KeyReset(&s_key_teach,  HAL_GetTick());
    ServoArm_KeyReset(&s_key_replay, HAL_GetTick());
  }
}

void ServoArm_Play(uint8_t team_num, uint8_t speed)
{
  uint8_t frame[8] = { 0u };

  if ((team_num >= SERVO_ARM_TEAM_NUM) || (s_point_cnt[team_num] == 0u))
  {
    return;
  }

  /* 一条回传都没收到过：到位判定不可能成立，会退化成按超时等待 */
  if (s_rx_count == 0u)
  {
    Debug_Printf("[ARM] warn: no position feedback, arrival check falls back to timeout\r\n");
  }

#if (SERVO_ARM_MULTI_TURN != 0u)
  /* 以当前实际位置为多圈基准，保证第一个点也按最短路径走 */
  ServoArm_UpdatePositions();
  for (uint8_t s = 0u; s < SERVO_ARM_SERVO_NUM; s++)
  {
    s_cmd_multi[s] = (int32_t)s_pos[s];
  }
#endif

  s_playing = 1u;

  for (uint16_t p = 0u; p < s_point_cnt[team_num]; p++)
  {
    uint32_t t_send = HAL_GetTick();
    uint8_t  arrived;
    int16_t  cmd_used[SERVO_ARM_SERVO_NUM];

    /* 4 个舵机在同一轮里下发，保证构成同一个姿态 */
    for (uint8_t s = 0u; s < SERVO_ARM_SERVO_NUM; s++)
    {
      uint16_t raw = s_team[team_num][p].pos[s];        /* 示教记录的单圈值 */
      int16_t  cmd = ServoArm_CommandValue(s, raw);     /* 实际发送值（可能是多圈） */

      cmd_used[s] = cmd;

      frame[0] = SERVO_CMD_POSITION;
      frame[1] = (uint8_t)(cmd & 0xFF);
      frame[2] = (uint8_t)((cmd >> 8) & 0xFF);
      frame[3] = speed;

      /* 手册要求同一帧连发两次（间隔 2ms），只发一次容易丢动作 */
      (void)ServoBus_SendRaw(s_arm_id[s], frame, 8u);
      HAL_Delay(2u);
      (void)ServoBus_SendRaw(s_arm_id[s], frame, 8u);
    }

    /* 按实际到位情况等待，而不是固定延时：到位就走，超时才放弃 */
    arrived = ServoArm_WaitArrive(team_num, p, SERVO_ARM_ARRIVE_TIMEOUT_MS);

    /* cmd 是实际下发的角度值：与示教值不同就说明这一步走了多圈（反向走近路） */
    Debug_Printf("[ARM] point %u/%u %s in %lums (max_err=%u) cmd=%d,%d,%d,%d\r\n",
                 (unsigned)(p + 1u), (unsigned)s_point_cnt[team_num],
                 (arrived != 0u) ? "arrived" : "TIMEOUT",
                 (unsigned long)(HAL_GetTick() - t_send),
                 (unsigned)ServoArm_PointMaxErr(team_num, p),
                 (int)cmd_used[0], (int)cmd_used[1],
                 (int)cmd_used[2], (int)cmd_used[3]);

    /* 到达后保持 SERVO_ARM_HOLD_MS 再走下一个点（给抓取/拍照等外部动作留时间） */
    HAL_Delay(SERVO_ARM_HOLD_MS);
  }

  s_playing = 0u;
}

uint16_t ServoArm_GetPointCount(uint8_t team_num)
{
  if (team_num >= SERVO_ARM_TEAM_NUM)
  {
    return 0u;
  }
  return s_point_cnt[team_num];
}

void ServoArm_SetTeam(uint8_t team_num)
{
  if (team_num < SERVO_ARM_TEAM_NUM)
  {
    s_team_cur = team_num;
  }
}

uint8_t ServoArm_IsTeaching(void)
{
  return s_teaching;
}

uint8_t ServoArm_IsPlaying(void)
{
  return s_playing;
}

uint16_t ServoArm_GetPos(uint8_t servo_idx)
{
  if (servo_idx >= SERVO_ARM_SERVO_NUM)
  {
    return 0u;
  }
  return s_pos[servo_idx];
}
