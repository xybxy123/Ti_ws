// 撥杆在左邊 按5下 發100 啓動任務1
// 撥杆在右邊 按10下 發200 啓動任務2
// 接收串口速度指令

#include "encoder.h"
#include "motor.h"
#include "pid_motor.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ENCODER_CAPTURE_LOAD_VALUE (0xFFFFU)
#define CONTROL_DT_SEC (0.01f)
#define DEFAULT_TARGET_SPEED_MPS (0.0f)
#define State_0 (0)
#define State_1 (1)
#define KEY_DEBOUNCE_TICKS (3U)
#define UART_RX_LINE_LEN (48U)
#define LED_FLASH_INTERVAL_TICKS (10U)
#define SWITCH_STATE_1 (1U)
#define SWITCH_STATE_2 (2U)

volatile float vel_LF_mps = 0.0f;
volatile float vel_LR_mps = 0.0f;
volatile float vel_RF_mps = 0.0f;
volatile float vel_RR_mps = 0.0f;
static volatile uint8_t control_state = State_0;
static volatile bool control_tick = false;
static volatile char uart_rx_line[UART_RX_LINE_LEN];
static volatile uint8_t uart_rx_len = 0;
static volatile bool uart_line_ready = false;
static uint8_t led_flash_edges = 0;
static uint8_t led_flash_tick_count = 0;

// ================= 新增：按键与开关的全局变量 =================
volatile uint8_t g_switch_state = SWITCH_STATE_2;
volatile int g_key_count = 0;
// ==========================================================

void MyGPTIMER1_INST_IRQHandler(void);
void UART_0_DEBUG_INST_IRQHandler(void);
static void Encoder_StartCaptureTimers(void);
static bool Control_ConsumeTick(void);
static bool Key_ReadPressed(void);
static void UART0_CommInit(void);
static void UART0_SendString(const char *text);
static void UART0_ConsumeRxLine(void);
static void UART0_ReceiveByte(uint8_t data);
static bool ParseTwistLine(char *line, float *vx, float *vyaw);
static char *SkipSeparators(char *text);
static void Led_StartFlash(uint8_t flashes);
static void Led_ProcessTick(void);
static void App_ProcessReport(void);

int main(void) {
  SYSCFG_DL_init();

  DL_GPIO_initDigitalInputFeatures(
      GPIO_CTRL_KEY_KEY_IOMUX, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
      DL_GPIO_WAKEUP_DISABLE);
  Encoder_Init(&encoder_);
  Encoder_StartCaptureTimers();

  NVIC_EnableIRQ(MyGPTIMER1_INST_INT_IRQN);
  NVIC_SetPriority(MyGPTIMER1_INST_INT_IRQN, 1);
  DL_TimerG_startCounter(MyGPTIMER1_INST);

  UART0_CommInit();

  DL_GPIO_setPins(GPIOB, GPIO_CTRL_POWER_PB19_PIN);
  Motor_StopAll();
  DL_TimerA_startCounter(PWM_MOTOR_INST);

  PID_Motor_Init();
  PID_Motor_SetTargetSpeed(DEFAULT_TARGET_SPEED_MPS, DEFAULT_TARGET_SPEED_MPS,
                           DEFAULT_TARGET_SPEED_MPS, DEFAULT_TARGET_SPEED_MPS);

  bool debounced_key_pressed = Key_ReadPressed();
  bool last_raw_key_pressed = debounced_key_pressed;
  uint8_t key_debounce_ticks = 0;
  while (1) {
    // 处理电机 PID 控制
    if (Control_ConsumeTick()) {
      PID_Motor_Update(CONTROL_DT_SEC);
      Led_ProcessTick();
    }

    UART0_ConsumeRxLine();

    if (DL_GPIO_readPins(GPIO_CTRL_KEY_SWITCH_PORT, GPIO_CTRL_KEY_SWITCH_PIN)) {
      g_switch_state = SWITCH_STATE_1;
    } else {
      g_switch_state = SWITCH_STATE_2;
    }

    bool raw_key_pressed = Key_ReadPressed();
    if (raw_key_pressed == last_raw_key_pressed) {
      if (key_debounce_ticks < KEY_DEBOUNCE_TICKS) {
        key_debounce_ticks++;
      }
    } else {
      last_raw_key_pressed = raw_key_pressed;
      key_debounce_ticks = 0;
    }

    if ((key_debounce_ticks >= KEY_DEBOUNCE_TICKS) &&
        (raw_key_pressed != debounced_key_pressed)) {
      debounced_key_pressed = raw_key_pressed;

      if (debounced_key_pressed) {
        g_key_count++;
        App_ProcessReport();
      }
    }

    App_ProcessReport();

    // 进入休眠，等待下一次 10ms 的 MyGPTIMER1 或 UART0 中断唤醒
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

void UART_0_DEBUG_INST_IRQHandler(void) {
  uint8_t data = 0U;

  switch (DL_UART_Main_getPendingInterrupt(UART_0_DEBUG_INST)) {
  case DL_UART_MAIN_IIDX_RX:
  case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
    while (DL_UART_Main_receiveDataCheck(UART_0_DEBUG_INST, &data)) {
      UART0_ReceiveByte(data);
    }
    DL_UART_Main_clearInterruptStatus(UART_0_DEBUG_INST,
                                      DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    break;

  case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
  case DL_UART_MAIN_IIDX_BREAK_ERROR:
  case DL_UART_MAIN_IIDX_PARITY_ERROR:
  case DL_UART_MAIN_IIDX_FRAMING_ERROR:
    DL_UART_Main_clearInterruptStatus(UART_0_DEBUG_INST,
                                      DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                          DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                          DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                          DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
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

static bool Key_ReadPressed(void) {
  return ((DL_GPIO_readPins(GPIO_CTRL_KEY_KEY_PORT, GPIO_CTRL_KEY_KEY_PIN) &
           GPIO_CTRL_KEY_KEY_PIN) == 0U);
}

static void UART0_CommInit(void) {
  DL_UART_Main_enableInterrupt(UART_0_DEBUG_INST,
                               DL_UART_MAIN_INTERRUPT_RX |
                                   DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
  NVIC_ClearPendingIRQ(UART_0_DEBUG_INST_INT_IRQN);
  NVIC_SetPriority(UART_0_DEBUG_INST_INT_IRQN, 2);
  NVIC_EnableIRQ(UART_0_DEBUG_INST_INT_IRQN);
}

static void UART0_SendString(const char *text) {
  while (*text != '\0') {
    DL_UART_Main_transmitDataBlocking(UART_0_DEBUG_INST, (uint8_t)*text);
    text++;
  }
}

static void UART0_ConsumeRxLine(void) {
  char line[UART_RX_LINE_LEN];
  float vx = 0.0f;
  float vyaw = 0.0f;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (!uart_line_ready) {
    if (primask == 0U) {
      __enable_irq();
    }
    return;
  }

  for (uint8_t i = 0; i < UART_RX_LINE_LEN; i++) {
    line[i] = (char)uart_rx_line[i];
    if (line[i] == '\0') {
      break;
    }
  }
  uart_line_ready = false;

  if (primask == 0U) {
    __enable_irq();
  }

  if (ParseTwistLine(line, &vx, &vyaw)) {
    PID_Motor_SetTwist(vx, vyaw);
  }
}

static void UART0_ReceiveByte(uint8_t data) {
  if (data == '\r') {
    return;
  }

  if (data == '\n') {
    if (uart_rx_len > 0U) {
      uart_rx_line[uart_rx_len] = '\0';
      uart_line_ready = true;
      uart_rx_len = 0U;
    }
    return;
  }

  if (uart_line_ready) {
    return;
  }

  if (uart_rx_len < (UART_RX_LINE_LEN - 1U)) {
    uart_rx_line[uart_rx_len] = (char)data;
    uart_rx_len++;
  } else {
    uart_rx_len = 0U;
  }
}

static bool ParseTwistLine(char *line, float *vx, float *vyaw) {
  char *end = line;
  char *p = SkipSeparators(line);

  if ((*p == 'T') || (*p == 't') || (*p == 'V') || (*p == 'v')) {
    p = SkipSeparators(p + 1);
  }

  *vx = strtof(p, &end);
  if (end == p) {
    return false;
  }

  p = SkipSeparators(end);
  *vyaw = strtof(p, &end);
  if (end == p) {
    return false;
  }

  return true;
}

static char *SkipSeparators(char *text) {
  while ((*text == ' ') || (*text == '\t') || (*text == ',') ||
         (*text == ':')) {
    text++;
  }
  return text;
}

static void Led_StartFlash(uint8_t flashes) {
  if (flashes == 0U) {
    return;
  }

  DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN);
  led_flash_edges = (uint8_t)((flashes * 2U) - 1U);
  led_flash_tick_count = LED_FLASH_INTERVAL_TICKS;
}

static void Led_ProcessTick(void) {
  if (led_flash_edges == 0U) {
    return;
  }

  if (led_flash_tick_count > 0U) {
    led_flash_tick_count--;
    return;
  }

  DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN);
  led_flash_edges--;
  led_flash_tick_count = LED_FLASH_INTERVAL_TICKS;

  if (led_flash_edges == 0U) {
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_LED_A27_PIN);
  }
}

static void App_ProcessReport(void) {
  static bool report_100_sent = false;
  static bool report_200_sent = false;

  if ((g_switch_state == SWITCH_STATE_1) && (g_key_count >= 5) &&
      !report_100_sent) {
    UART0_SendString("100\r\n");
    Led_StartFlash(2U);
    report_100_sent = true;
  }

  if ((g_switch_state == SWITCH_STATE_2) && (g_key_count >= 10) &&
      !report_200_sent) {
    UART0_SendString("200\r\n");
    Led_StartFlash(4U);
    report_200_sent = true;
  }
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
