#include "motor.h"
#include "ti_msp_dl_config.h"

int main(void) {
  /* 1. 初始化所有硬體 (時鐘, GPIO, 定時器, UART 等) */
  SYSCFG_DL_init();

  /* ==================================================== */
  /* 2. 拉高 PB19 使能引腳，啟動馬達驅動板               */
  /* ==================================================== */
  DL_GPIO_setPins(GPIOB, GPIO_CTRL_POWER_PB19_PIN);
  /* 3. 預設先讓所有馬達停止 (輸出 50% 佔空比) */
  Motor_StopAll();

  /* 4. 啟動 PWM 定時器計數器，開始輸出波形 */
  DL_TimerA_startCounter(PWM_MOTOR_INST);

  while (1) {
    /* --- 測試 1：全速正轉運行 2 秒 --- */
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN); // 亮燈提示
    Motor_SetSpeed(50, 50, 50, 50);
    delay_cycles(64000000); // 延時約 2 秒 (32MHz * 2)

    /* --- 測試 2：停止運行 2 秒 --- */
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN); // 熄燈
    Motor_StopAll();
    delay_cycles(64000000);

    /* --- 測試 3：半速反轉運行 2 秒 --- */
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN); // 亮燈提示
    Motor_SetSpeed(-50, -50, -50, -50);
    delay_cycles(64000000);

    /* --- 測試 4：停止運行 2 秒 --- */
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN); // 熄燈
    Motor_StopAll();
    delay_cycles(64000000);

    /* --- 測試 5：原地打轉 (左側輪向前，右側輪向後) --- */
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN); // 亮燈提示
    // Motor_SetSpeed(80, 80, -80, -80);
    delay_cycles(64000000);

    Motor_StopAll();
    delay_cycles(64000000);
  }
}