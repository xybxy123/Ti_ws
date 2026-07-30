#include "motor.h"

/**
 * @brief 內部輔助函數：將 -100~100 的速度轉換為 0~1000 的 PWM 比較值
 */
static uint32_t Calculate_PWM_Value(int8_t speed) {
  /* 安全保護：限制輸入範圍在 -100 到 100 之間 */
  if (speed > 100) {
    speed = 100;
  } else if (speed < -100) {
    speed = -100;
  }

  /*
   * 數學映射：
   * speed: [-100, 100]
   * speed * 5: [-500, 500]
   * (speed * 5) + 500: [0, 1000]
   */
  int32_t pwm_val = (speed * 5) + 500;

  return (uint32_t)pwm_val;
}

// ---------------------------------------------------------
// 獨立輪控制接口
// ---------------------------------------------------------

/* 左前輪 M2 -> PA9 (Channel 1) */
void Motor_SetLeftFront(int8_t speed) {
  if (speed < 0) {
    speed = 0;
  }
  uint32_t val = Calculate_PWM_Value(speed);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val, DL_TIMER_CC_1_INDEX);
}

/* 左後輪 M3 -> PB17 (Channel 2) */
void Motor_SetLeftRear(int8_t speed) {
  if (speed < 0) {
    speed = 0;
  }
  uint32_t val = Calculate_PWM_Value(speed);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val, DL_TIMER_CC_2_INDEX);
}

/* 右前輪 M1 -> PA8 (Channel 0)
 * 備註：因為硬體安裝對稱的關係，右側馬達通常需要反轉，所以傳入 -speed
 */
void Motor_SetRightFront(int8_t speed) {
  if (speed < 0) {
    speed = 0;
  }
  uint32_t val = Calculate_PWM_Value(-speed);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val, DL_TIMER_CC_0_INDEX);
}

/* 右後輪 M4 -> PB2 (Channel 3)
 * 備註：右側馬達反轉
 */
void Motor_SetRightRear(int8_t speed) {
  if (speed < 0) {
    speed = 0;
  }
  uint32_t val = Calculate_PWM_Value(-speed);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val, DL_TIMER_CC_3_INDEX);
}

// ---------------------------------------------------------
// 綜合控制接口
// ---------------------------------------------------------

void Motor_SetSpeed(int8_t speed_LF, int8_t speed_LR, int8_t speed_RF,
                    int8_t speed_RR) {
  Motor_SetLeftFront(speed_LF);
  Motor_SetLeftRear(speed_LR);
  Motor_SetRightFront(speed_RF);
  Motor_SetRightRear(speed_RR);
}

void Motor_StopAll(void) { Motor_SetSpeed(0, 0, 0, 0); }