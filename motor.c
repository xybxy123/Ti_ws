#include "motor.h"

/**
 * @brief 內部輔助函數：將 -100~100 的速度轉換為 0~1000 的 PWM 比較值
 */
static uint32_t Calculate_PWM_Value(int8_t speed)
{
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

void Motor_SetSpeed(int8_t speed_m1, int8_t speed_m2, int8_t speed_m3, int8_t speed_m4)
{
    uint32_t val_m1 = Calculate_PWM_Value(speed_m1);
    uint32_t val_m2 = Calculate_PWM_Value(speed_m2);
    uint32_t val_m3 = Calculate_PWM_Value(speed_m3);
    uint32_t val_m4 = Calculate_PWM_Value(speed_m4);

    /* M1 -> PA8 (Channel 0) */
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val_m1, DL_TIMER_CC_0_INDEX);
    /* M2 -> PA9 (Channel 1) */
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val_m2, DL_TIMER_CC_1_INDEX);
    /* M3 -> PB17 (Channel 2) */
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val_m3, DL_TIMER_CC_2_INDEX);
    /* M4 -> PB2 (Channel 3) */
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, val_m4, DL_TIMER_CC_3_INDEX);
}

void Motor_StopAll(void)
{
    /* 傳入 0 就會自動計算為 500 (50% 佔空比) */
    Motor_SetSpeed(0, 0, 0, 0);
}
