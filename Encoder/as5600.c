/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 磁编码器驱动实现
  ******************************************************************************
  */
#include "as5600.h"
#include "i2c.h"
#include <string.h>

static AS5600_t s_enc[AS5600_AXIS_NUM];

/* ---------- 底层 I2C ---------- */

bool AS5600_ReadBlock(I2C_HandleTypeDef *hi2c, uint8_t *buf)
{
  if ((hi2c == NULL) || (buf == NULL))
  {
    return false;
  }

  /* 一次事务读完 STATUS + RAW + ANG：
     分开读两次会出现"读高字节后又翻转"的撕裂值，这里一并读掉 */
  return HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, AS5600_REG_BLOCK,
                          I2C_MEMADD_SIZE_8BIT, buf, AS5600_BLOCK_LEN,
                          AS5600_I2C_TIMEOUT) == HAL_OK;
}

bool AS5600_ReadRaw(I2C_HandleTypeDef *hi2c, uint16_t *raw)
{
  uint8_t buf[AS5600_BLOCK_LEN];

  if ((raw == NULL) || !AS5600_ReadBlock(hi2c, buf))
  {
    return false;
  }

  *raw = (uint16_t)(((uint16_t)(buf[1] & 0x0Fu) << 8) | buf[2]);
  return true;
}

bool AS5600_IsMagnetOk(I2C_HandleTypeDef *hi2c)
{
  uint8_t buf[AS5600_BLOCK_LEN];

  /* 状态寄存器 bit5 = MD（Magnet Detected） */
  return AS5600_ReadBlock(hi2c, buf) && ((buf[0] & 0x20u) != 0u);
}

/* ---------- 角度累计 ---------- */

/*
 * 单圈 12 位，软件累计圈数：
 *   本次与上次之差若超过半圈（2048），说明是绕过了 0/4095 边界，按整圈修正。
 *   两次调用之间转动不能超过半圈（本车 100Hz 更新，允许 50 圈/秒，足够）。
 */
static void AS5600_Accumulate(AS5600_t *e, uint16_t raw)
{
  int16_t delta = (int16_t)(raw - e->last_raw);

  if (delta > 2048)
  {
    delta = (int16_t)(delta - 4096);
  }
  else if (delta < -2048)
  {
    delta = (int16_t)(delta + 4096);
  }

  e->accum_cnt += delta;
  e->accum_rad  = (float)e->accum_cnt * AS5600_TWO_PI / (float)AS5600_CNT_PER_REV;
  e->last_raw   = raw;
}

/* ---------- 对外接口 ---------- */

void AS5600_Init(void)
{
  memset(s_enc, 0, sizeof(s_enc));
  s_enc[AS5600_AXIS_X].hi2c = &hi2c1;
  s_enc[AS5600_AXIS_Y].hi2c = &hi2c3;

  for (uint8_t i = 0u; i < AS5600_AXIS_NUM; i++)
  {
    AS5600_UpdateAxis(i);          /* 取一次基准值（last_raw），后续都在此基础上累计 */
    AS5600_Zero(i);                /* 上电累计角度归零 */
  }
}

void AS5600_UpdateAxis(uint8_t axis)
{
  AS5600_t *e;
  uint8_t   buf[AS5600_BLOCK_LEN];

  if (axis >= AS5600_AXIS_NUM)
  {
    return;
  }
  e = &s_enc[axis];

  if (!AS5600_ReadBlock(e->hi2c, buf))
  {
    if (e->fail_cnt < 255u)
    {
      e->fail_cnt++;
    }
    if (e->fail_cnt >= AS5600_FAIL_LIMIT)
    {
      e->online = 0u;
    }
    e->err_cnt++;
    return;
  }

  e->fail_cnt   = 0u;
  e->online     = 1u;
  e->magnet     = ((buf[0] & 0x20u) != 0u) ? 1u : 0u;
  e->raw        = (uint16_t)(((uint16_t)(buf[1] & 0x0Fu) << 8) | buf[2]);
  e->raw_filt   = (uint16_t)(((uint16_t)(buf[3] & 0x0Fu) << 8) | buf[4]);

  /* 用未滤波的原始值累计，避免滤波带来的额外滞后 */
  AS5600_Accumulate(e, e->raw);
}

void AS5600_Update(void)
{
  for (uint8_t i = 0u; i < AS5600_AXIS_NUM; i++)
  {
    AS5600_UpdateAxis(i);
  }
}

const AS5600_t *AS5600_Get(uint8_t axis)
{
  if (axis >= AS5600_AXIS_NUM)
  {
    return NULL;
  }
  return &s_enc[axis];
}

float AS5600_GetAccumRad(uint8_t axis)
{
  if (axis >= AS5600_AXIS_NUM)
  {
    return 0.0f;
  }
  return s_enc[axis].accum_rad;
}

void AS5600_Zero(uint8_t axis)
{
  AS5600_t *e;

  if (axis >= AS5600_AXIS_NUM)
  {
    return;
  }
  e = &s_enc[axis];

  e->accum_cnt = 0;
  e->accum_rad = 0.0f;
  e->last_raw  = e->raw;      /* 以当前角度为新的累计起点 */
}

void AS5600_ZeroAll(void)
{
  for (uint8_t i = 0u; i < AS5600_AXIS_NUM; i++)
  {
    AS5600_Zero(i);
  }
}
