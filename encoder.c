#include "encoder.h"

Encoder encoder_;

void Encoder_Init(Encoder *enc) {
  enc->count_LF = 0;
  enc->count_LR = 0;
  enc->count_RF = 0;
  enc->count_RR = 0;
  enc->vel_LF = 0.0f;
  enc->vel_LR = 0.0f;
  enc->vel_RF = 0.0f;
  enc->vel_RR = 0.0f;
}

// 计算速度：用一个周期内收集到的脉冲数除以时间 dt
void Encoder_UpdateVelocity(Encoder *enc, float dt) {
  if (enc == NULL || dt <= 0.0f)
    return;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t count_LF = enc->count_LF;
  uint32_t count_LR = enc->count_LR;
  uint32_t count_RF = enc->count_RF;
  uint32_t count_RR = enc->count_RR;

  enc->count_LF = 0;
  enc->count_LR = 0;
  enc->count_RF = 0;
  enc->count_RR = 0;

  if (primask == 0U) {
    __enable_irq();
  }

  // 转换为每秒脉冲数 (PPS)，再由主循环换算成 m/s
  enc->vel_LF = (float)count_LF / dt;
  enc->vel_LR = (float)count_LR / dt;
  enc->vel_RF = (float)count_RF / dt;
  enc->vel_RR = (float)count_RR / dt;
}

float Encoder_GetLeftFrontVel(Encoder *enc) { return enc->vel_LF; }
float Encoder_GetLeftRearVel(Encoder *enc) { return enc->vel_LR; }
float Encoder_GetRightFrontVel(Encoder *enc) { return enc->vel_RF; }
float Encoder_GetRightRearVel(Encoder *enc) { return enc->vel_RR; }

void TIMG0_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(M1_INST)) {
  case DL_TIMER_IIDX_CC0_DN:
    encoder_.count_RF++;
    break;
  default:
    break;
  }
  DL_TimerG_clearInterruptStatus(M1_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
}

void TIMG6_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(M2_INST)) {
  case DL_TIMER_IIDX_CC0_DN:
    encoder_.count_LF++;
    break;
  default:
    break;
  }
  DL_TimerG_clearInterruptStatus(M2_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
}

void TIMG7_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(M3_INST)) {
  case DL_TIMER_IIDX_CC0_DN:
    encoder_.count_LR++;
    break;
  default:
    break;
  }
  DL_TimerG_clearInterruptStatus(M3_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
}

void TIMA1_IRQHandler(void) {
  switch (DL_TimerA_getPendingInterrupt(M4_INST)) {
  case DL_TIMER_IIDX_CC0_DN:
    encoder_.count_RR++;
    break;
  default:
    break;
  }
  DL_TimerA_clearInterruptStatus(M4_INST, DL_TIMERA_INTERRUPT_CC0_DN_EVENT);
}
