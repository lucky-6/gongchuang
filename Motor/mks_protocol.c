/**
  ******************************************************************************
  * @file    mks_protocol.c
  * @brief   MKS RS485 协议组帧与换算
  ******************************************************************************
  */
#include "mks_protocol.h"

uint8_t MKS_Checksum(const uint8_t *buf, uint16_t len)
{
  uint8_t sum = 0u;

  for (uint16_t i = 0u; i < len; i++)
  {
    sum = (uint8_t)(sum + buf[i]);
  }
  return sum;
}

uint16_t MKS_RpmToSpeedValue(uint32_t rpm_actual, uint16_t mstep)
{
  uint32_t num;
  uint32_t val;

  if (mstep == 0u)
  {
    mstep = MKS_SPEED_BASE_MSTEP;
  }

  /* 驱动板速度值以 16 细分为基准：value = rpm × 细分 / 16（四舍五入）
     手册示例：16 细分下 speed=640 即 640 RPM；128 细分下 speed=1200 实际 150 RPM */
  num = rpm_actual * (uint32_t)mstep;
  val = (num + (MKS_SPEED_BASE_MSTEP / 2u)) / MKS_SPEED_BASE_MSTEP;

  if (val > MKS_SPEED_MAX)
  {
    val = MKS_SPEED_MAX;
  }
  return (uint16_t)val;
}

uint32_t MKS_SpeedValueToRpm(uint16_t speed_value, uint16_t mstep)
{
  if (mstep == 0u)
  {
    mstep = MKS_SPEED_BASE_MSTEP;
  }
  return ((uint32_t)speed_value * MKS_SPEED_BASE_MSTEP) / (uint32_t)mstep;
}

uint32_t MKS_RevsToPulses(uint16_t revs, uint16_t mstep)
{
  return (uint32_t)revs * MKS_FULL_STEP_PER_REV * (uint32_t)mstep;
}

uint16_t MKS_BuildSpeed(uint8_t addr, int32_t rpm_actual, uint16_t mstep,
                        uint8_t acc, uint8_t *out)
{
  uint32_t mag   = (rpm_actual < 0) ? (uint32_t)(-rpm_actual) : (uint32_t)rpm_actual;
  uint16_t speed = MKS_RpmToSpeedValue(mag, mstep);
  uint8_t  dir   = (rpm_actual < 0) ? 1u : 0u;   /* b7：0 = CW，1 = CCW */

  out[0] = MKS_HEAD_DOWN;
  out[1] = addr;
  out[2] = MKS_FC_SPEED;
  out[3] = (uint8_t)((dir << 7) | ((uint16_t)(speed >> 8) & 0x0Fu));
  out[4] = (uint8_t)(speed & 0xFFu);
  out[5] = acc;
  out[6] = MKS_Checksum(out, 6u);
  return 7u;
}

uint16_t MKS_BuildMovePulses(uint8_t addr, int32_t pulses, int32_t rpm_actual,
                             uint16_t mstep, uint8_t acc, uint8_t *out)
{
  uint32_t mag_rpm = (rpm_actual < 0) ? (uint32_t)(-rpm_actual) : (uint32_t)rpm_actual;
  uint16_t speed   = MKS_RpmToSpeedValue(mag_rpm, mstep);
  uint8_t  dir     = (pulses < 0) ? 1u : 0u;
  uint32_t mag     = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)pulses;

  out[0]  = MKS_HEAD_DOWN;
  out[1]  = addr;
  out[2]  = MKS_FC_MOVE_PULSE;
  out[3]  = (uint8_t)((dir << 7) | ((uint16_t)(speed >> 8) & 0x0Fu));
  out[4]  = (uint8_t)(speed & 0xFFu);
  out[5]  = acc;
  out[6]  = (uint8_t)((mag >> 24) & 0xFFu);   /* 32 位脉冲数，大端 */
  out[7]  = (uint8_t)((mag >> 16) & 0xFFu);
  out[8]  = (uint8_t)((mag >> 8) & 0xFFu);
  out[9]  = (uint8_t)(mag & 0xFFu);
  out[10] = MKS_Checksum(out, 10u);
  return 11u;
}

uint16_t MKS_BuildStop(uint8_t addr, uint8_t mode, uint8_t acc, uint8_t *out)
{
  if (mode == MKS_FC_MOVE_PULSE)
  {
    out[0]  = MKS_HEAD_DOWN;
    out[1]  = addr;
    out[2]  = MKS_FC_MOVE_PULSE;
    out[3]  = 0u;
    out[4]  = 0u;
    out[5]  = acc;
    out[6]  = 0u;
    out[7]  = 0u;
    out[8]  = 0u;
    out[9]  = 0u;
    out[10] = MKS_Checksum(out, 10u);
    return 11u;
  }

  out[0] = MKS_HEAD_DOWN;
  out[1] = addr;
  out[2] = MKS_FC_SPEED;
  out[3] = 0u;
  out[4] = 0u;
  out[5] = acc;                 /* acc = 0 立即停，非 0 减速停 */
  out[6] = MKS_Checksum(out, 6u);
  return 7u;
}

uint16_t MKS_BuildEnable(uint8_t addr, bool enable, uint8_t *out)
{
  out[0] = MKS_HEAD_DOWN;
  out[1] = addr;
  out[2] = MKS_FC_ENABLE;
  out[3] = enable ? 0x01u : 0x00u;
  out[4] = MKS_Checksum(out, 4u);
  return 5u;                    /* FA addr F3 01 CRC = 5 字节 */
}

uint16_t MKS_BuildQuery(uint8_t addr, uint8_t code, uint8_t *out)
{
  out[0] = MKS_HEAD_DOWN;
  out[1] = addr;
  out[2] = code;
  out[3] = MKS_Checksum(out, 3u);
  return 4u;
}
