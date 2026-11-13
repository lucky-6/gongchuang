/**
  ******************************************************************************
  * @file    user_plan.c
  * @brief   你的任务规划：空壳，怎么填由你决定
  *
  *   ── 动作接口的用法（两段式）────────────────────────────────────────
  *
  *     发起：Act_NavGoto / Act_ArmMove / Act_GripClose / Act_VisBiasStart ...
  *     查询：Act_NavResult() / Act_ArmResult()  →  ACT_RUNNING / ACT_DONE / ACT_FAIL
  *
  *   典型一个步骤长这样：
  *
  *     static uint8_t s_phase = 0;
  *
  *     void Plan_Task(void)
  *     {
  *       switch (s_stage) {
  *       case 0:                                   // 发起
  *         Act_NavGoto(600.0f, 0.0f, 0.0f, 5000u);
  *         s_stage = 1;
  *         break;
  *       case 1:                                   // 等结果
  *         if (Act_NavResult() == ACT_RUNNING) { return; }
  *         s_stage = (Act_NavResult() == ACT_DONE) ? 2 : 9;   // 成功/失败分流由你定
  *         break;
  *       ...
  *       }
  *     }
  *
  *   ── 三个必须你自己处理的事（这里没有兜底）──────────────────────────
  *
  *     1) 停车：MKS 是速度模式，不发指令就保持上一次速度。
  *        该停的地方要显式 Act_Stop()。
  *     2) 超时：Act_*Result() 只会返回 RUNNING/DONE/FAIL，不会自己超时。
  *        要限时就自己记 Act_Now() 的差。
  *     3) 异常：没有任何自动降级 / 自动回启停区 / 看门狗。
  *
  *   ── 调试 ──────────────────────────────────────────────────────────
  *     g_act_verbose = 1 打印每个动作；= 0 静音
  *     导航 / 机械臂 / 视觉 目前是空壳，调了会打"未实现"并返回 FAIL
  ******************************************************************************
  */
#include "user_plan.h"
#include "app_action.h"

/* 把下面改成 1 可以看到一个最小示范（走 2 步 → 夹一下 → 停），仅供对照接口，
 * 看完请改回 0，然后自己写。 */
#define PLAN_DEMO   0

struct pos {
    float x;
    float y;
    float yaw;
} position;

void Plan_Init(void) {
    Act_Log("[PLAN] init\r\n");
}

void Plan_Task(void) {
#if (PLAN_DEMO != 0)
    static uint8_t s_stage = 0;

    switch (s_stage) {
        case 0:
            Act_NavGoto(600.0f, 0.0f, 0.0f, 5000u);
            s_stage = 1;
            break;

        case 1:
            if (Act_NavResult() == ACT_RUNNING) { return; }
            Act_GripClose();
            s_stage = 2;
            break;

        case 2:
            if (Act_ArmResult() == ACT_RUNNING) { return; }
            Act_Stop();
            Act_DispMsg("DEMO DONE");
            s_stage = 3;
            break;

        default:
            break;
    }
#else
    /* ==== 机械臂在两个位置之间来回移动 ==== */
    #define ARM_SPEED       0x20u    /* 0x20 慢速、0x80 中速、0xFF 最快 */
    #define ARM_TOL_DEG     2.0f     /* 到位容差（度）。别小于 1°，会永远判不到位 */
    #define ARM_TIMEOUT_MS  3000u    /* 单程超时，防止被挡住时死等 */

    static const float k_posA[ACT_ARM_NUM] = { 270.3f, 215.3f, 242.5f, 195.9f };
    static const float k_posB[ACT_ARM_NUM] = { 270.5f, 237.0f, 243.1f, 166.6f };

    static uint32_t s_print_ms = 0u;
    static uint8_t  s_pending  = 0u;     /* 1 = 有新目标要下发 */
    static uint8_t  s_to_b     = 0u;     /* 0 = 目标 A，1 = 目标 B */
    static uint32_t s_t0       = 0u;

    Act_PoseGet(&position.x, &position.y, &position.yaw);

    if (s_pending != 0u) {
        Act_ArmSetAngles((s_to_b != 0u) ? k_posB : k_posA, ARM_SPEED);
        s_t0      = Act_Now();
        s_pending = 0u;
    }
    else if (Act_ArmAtTarget(ARM_TOL_DEG)) {
        /* ★到位：换方向，下一拍下发另一个位置 */
        s_to_b ^= 1u;
        s_pending = 1u;
    }
    else if ((Act_Now() - s_t0) > ARM_TIMEOUT_MS) {
        /* 超时（被挡住 / 位置回读不通）：照样换向，避免永远卡在这一侧 */
        Act_Log("[PLAN] arm timeout -> switch\r\n");
        s_to_b ^= 1u;
        s_pending = 1u;
    }

    /* ---- 100ms 打印一次实测角度，方便看有没有到位 ---- */
    if ((Act_Now() - s_print_ms) >= 100u) {
        float   a[ACT_ARM_NUM];
        float   g = -1.0f;
        uint8_t i;

        s_print_ms = Act_Now();

        for (i = 0u; i < ACT_ARM_NUM; i++) {
            a[i] = -1.0f;
            (void)Act_ArmAngle(i, &a[i]);
        }
        (void)Act_GripAngle(&g);

        Act_Log("%.1f,%.1f,%.1f,%.1f | grip=%.1f | rx=%lu | %s\r\n",
                a[0], a[1], a[2], a[3], g,
                (unsigned long)Act_ArmRxCount(),
                (s_to_b != 0u) ? "->B" : "->A");
    }
#endif
}
