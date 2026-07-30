#include "encoder.h"
#include "motor.h"
#include "pid_motor.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define ENCODER_CAPTURE_LOAD_VALUE (0xFFFFU)
#define CONTROL_DT_SEC (0.01f)
#define DEFAULT_TARGET_SPEED_MPS (0.20f)

volatile float vel_LF_mps = 0.0f;
volatile float vel_LR_mps = 0.0f;
volatile float vel_RF_mps = 0.0f;
volatile float vel_RR_mps = 0.0f;

static volatile bool control_tick = false;

void MyGPTIMER1_INST_IRQHandler(void);
static void Encoder_StartCaptureTimers(void);
static bool Control_ConsumeTick(void);

int main(void) {
  SYSCFG_DL_init();
  Encoder_Init(&encoder_);
  Encoder_StartCaptureTimers();

  NVIC_EnableIRQ(MyGPTIMER1_INST_INT_IRQN);
  NVIC_SetPriority(MyGPTIMER1_INST_INT_IRQN, 1);
  DL_TimerG_startCounter(MyGPTIMER1_INST);

  DL_GPIO_setPins(GPIOB, GPIO_CTRL_POWER_PB19_PIN);
  Motor_StopAll();
  DL_TimerA_startCounter(PWM_MOTOR_INST);

  PID_Motor_Init();
  PID_Motor_SetTargetSpeed(DEFAULT_TARGET_SPEED_MPS, DEFAULT_TARGET_SPEED_MPS,
                           DEFAULT_TARGET_SPEED_MPS, DEFAULT_TARGET_SPEED_MPS);

  while (1) {
    if (Control_ConsumeTick()) {
      PID_Motor_Update(CONTROL_DT_SEC);
    }

    __WFI();
  }
}

void MyGPTIMER1_INST_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(MyGPTIMER1_INST)) {
  case DL_TIMER_IIDX_ZERO:
    Encoder_UpdateVelocity(&encoder_, CONTROL_DT_SEC);

    vel_LF_mps = encoder_.vel_LF * ENC_TO_MPS_K;
    vel_LR_mps = encoder_.vel_LR * ENC_TO_MPS_K;
    vel_RF_mps = encoder_.vel_RF * ENC_TO_MPS_K;
    vel_RR_mps = encoder_.vel_RR * ENC_TO_MPS_K;
    control_tick = true;

    DL_TimerG_clearInterruptStatus(MyGPTIMER1_INST,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);
    break;

  default:
    break;
  }
}

static bool Control_ConsumeTick(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  bool tick = control_tick;
  control_tick = false;

  if (primask == 0U) {
    __enable_irq();
  }

  return tick;
}

static void Encoder_StartCaptureTimers(void) {
  DL_TimerG_setLoadValue(M1_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerG_setTimerCount(M1_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerG_setLoadValue(M2_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerG_setTimerCount(M2_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerG_setLoadValue(M3_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerG_setTimerCount(M3_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerA_setLoadValue(M4_INST, ENCODER_CAPTURE_LOAD_VALUE);
  DL_TimerA_setTimerCount(M4_INST, ENCODER_CAPTURE_LOAD_VALUE);

  DL_TimerG_clearInterruptStatus(M1_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
  DL_TimerG_clearInterruptStatus(M2_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
  DL_TimerG_clearInterruptStatus(M3_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
  DL_TimerA_clearInterruptStatus(M4_INST, DL_TIMERA_INTERRUPT_CC0_DN_EVENT);

  DL_TimerG_enableInterrupt(M1_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
  DL_TimerG_enableInterrupt(M2_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
  DL_TimerG_enableInterrupt(M3_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
  DL_TimerA_enableInterrupt(M4_INST, DL_TIMERA_INTERRUPT_CC0_DN_EVENT);

  NVIC_SetPriority(M1_INST_INT_IRQN, 0);
  NVIC_SetPriority(M2_INST_INT_IRQN, 0);
  NVIC_SetPriority(M3_INST_INT_IRQN, 0);
  NVIC_SetPriority(M4_INST_INT_IRQN, 0);
  NVIC_ClearPendingIRQ(M1_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(M2_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(M3_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(M4_INST_INT_IRQN);
  NVIC_EnableIRQ(M1_INST_INT_IRQN);
  NVIC_EnableIRQ(M2_INST_INT_IRQN);
  NVIC_EnableIRQ(M3_INST_INT_IRQN);
  NVIC_EnableIRQ(M4_INST_INT_IRQN);

  DL_TimerG_startCounter(M1_INST);
  DL_TimerG_startCounter(M2_INST);
  DL_TimerG_startCounter(M3_INST);
  DL_TimerA_startCounter(M4_INST);
}
