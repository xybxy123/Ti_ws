// 撥杆在左邊 按5下 發100 啓動任務1
// 撥杆在右邊 按10下 發200 啓動任務2
// 接收串口速度指令

#include "encoder.h"
#include "motor.h"
#include "pid_motor.h"
#include "pis_pos.h"
#include "ti_msp_dl_config.h"
#include <math.h>
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
#define PATH_START_KEY_COUNT (5)
#define PATH_MAX_POINTS (180U)
#define PATH_STEP_M (0.05f)
#define PATH_LOOKAHEAD_M (0.20f)
#define PATH_FINISH_POS_M (0.05f)
#define PATH_FINISH_YAW_RAD (0.05f)
#define PATH_ALIGN_YAW_RAD (0.35f)
#define PATH_AB_LEN_M (1.50f)
#define PATH_BC_RADIUS_M (0.75f)
#define PATH_CD_LEN_M (1.50f)
#define PATH_DA_RADIUS_M (0.75f)
#define PATH_EXTRA_FORWARD_M (0.0f)
#define LSM6DSR_WHO_AM_I_VAL (0x6BU)
#define LSM6DSR_REG_WHO_AM_I (0x0FU)
#define LSM6DSR_REG_CTRL2_G (0x11U)
#define LSM6DSR_REG_CTRL3_C (0x12U)
#define LSM6DSR_REG_CTRL9_XL (0x18U)
#define LSM6DSR_REG_OUTZ_L_G (0x26U)
#define LSM6DSR_CTRL3_C_BDU (1U << 6)
#define LSM6DSR_CTRL3_C_IF_INC (1U << 2)
#define LSM6DSR_CTRL3_C_SW_RESET (1U << 0)
#define LSM6DSR_CTRL9_XL_I3C_DISABLE (1U << 1)
#define LSM6DSR_GYRO_ODR_104HZ (0x04U)
#define LSM6DSR_GYRO_FS_250DPS (0x00U)
#define LSM6DSR_GYRO_SENS_250DPS (0.00875f)
#define IMU_CALIB_SAMPLES (200U)
#define IMU_CALIB_SAMPLE_DELAY_MS (5U)
#define IMU_GYRO_Z_DEADBAND_DPS (0.15f)
#define IMU_YAW_SIGN (1.0f)
#define DEG_TO_RAD (PI / 180.0f)
#define ENCODER_REVERSE_GPIO_PINS                                              \
  (GPIO_ENCODERS_ENCODER_M1_B_PIN | GPIO_ENCODERS_ENCODER_M2_B_PIN |           \
   GPIO_ENCODERS_ENCODER_M3_B_PIN | GPIO_ENCODERS_ENCODER_M4_B_PIN)

volatile float vel_LF_mps = 0.0f;
volatile float vel_LR_mps = 0.0f;
volatile float vel_RF_mps = 0.0f;
volatile float vel_RR_mps = 0.0f;
volatile float reverse_vel_LF_mps = 0.0f;
volatile float reverse_vel_LR_mps = 0.0f;
volatile float reverse_vel_RF_mps = 0.0f;
volatile float reverse_vel_RR_mps = 0.0f;
volatile float odom_x_m = 0.0f;
volatile float odom_y_m = 0.0f;
volatile float odom_yaw_rad = 0.0f;
volatile float odom_vx_mps = 0.0f;
volatile float odom_vyaw_radps = 0.0f;
volatile float imu_yaw_rad = 0.0f;
volatile float imu_gyro_z_dps = 0.0f;
volatile float imu_gyro_z_bias_dps = 0.0f;
volatile bool imu_ready = false;

static volatile uint8_t control_state = State_0;
static volatile bool control_tick = false;
static volatile char uart_rx_line[UART_RX_LINE_LEN];
static volatile uint8_t uart_rx_len = 0;
static volatile bool uart_line_ready = false;
static uint8_t led_flash_edges = 0;
static uint8_t led_flash_tick_count = 0;
typedef struct {
  float x_m;
  float y_m;
  float yaw_rad;
} PathPoint;

static PosPid pos_dist_pid;
static PosPid pos_yaw_pid;
static bool path_nav_active = false;
static bool path_nav_finished = false;
static PathPoint path_points[PATH_MAX_POINTS];
static uint16_t path_point_count = 0U;
static uint16_t path_current_index = 0U;
static float path_origin_x_m = 0.0f;
static float path_origin_y_m = 0.0f;
static float path_origin_yaw_rad = 0.0f;

// ================= 新增：按键与开关的全局变量 =================
volatile uint8_t g_switch_state = SWITCH_STATE_2;
volatile int g_key_count = 0;
// ==========================================================

void MyGPTIMER1_INST_IRQHandler(void);
void UART1_INST_IRQHandler(void);
void GROUP1_IRQHandler(void);
static void Encoder_StartCaptureTimers(void);
static void Encoder_StartReverseGpioInputs(void);
static bool Control_ConsumeTick(void);
static bool Key_ReadPressed(void);
static void UART1_CommInit(void);
static void UART1_SendString(const char *text);
static void UART1_ConsumeRxLine(void);
static void UART1_ReceiveByte(uint8_t data);
static bool ParseTwistLine(char *line, float *vx, float *vyaw);
static char *SkipSeparators(char *text);
static void Odom_Update(float dt);
static float Odom_GetSignedWheelSpeed(float forward_mps, float reverse_mps);
static float Odom_NormalizeYaw(float yaw_rad);
static void IMU_Init(void);
static void IMU_Update(float dt);
static void IMU_CalibrateGyroZ(void);
static void IMU_DelayMs(uint32_t ms);
static bool IMU_ReadReg(uint8_t reg, uint8_t *value);
static bool IMU_WriteReg(uint8_t reg, uint8_t value);
static bool IMU_ReadMulti(uint8_t reg, uint8_t *buf, uint16_t len);
static int16_t IMU_ReadGyroZRaw(void);
static void IMU_CsLow(void);
static void IMU_CsHigh(void);
static uint8_t IMU_SpiTransfer(uint8_t data);
static void PathNav_Init(void);
static void PathNav_Start(void);
static void PathNav_Update(float dt);
static void PathNav_Generate(void);
static void PathNav_Clear(void);
static void PathNav_AddLocalPoint(float local_x_m, float local_y_m,
                                  float local_yaw_rad);
static void PathNav_AppendLocalLine(float sx, float sy, float ex, float ey,
                                    float step);
static void PathNav_AppendLocalArc(float cx, float cy, float radius,
                                   float start_angle, float end_angle,
                                   float step);
static float PathNav_YawErrorAbs(float target_yaw_rad, float real_yaw_rad);
static void Led_StartFlash(uint8_t flashes);
static void Led_ProcessTick(void);
static void App_ProcessReport(void);

int main(void) {
  SYSCFG_DL_init();

  DL_GPIO_initDigitalInputFeatures(
      GPIO_CTRL_KEY_KEY_IOMUX, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
      DL_GPIO_WAKEUP_DISABLE);
  DL_GPIO_disableOutput(GPIO_CTRL_KEY_KEY_PORT, GPIO_CTRL_KEY_KEY_PIN);
  DL_GPIO_initDigitalInputFeatures(
      GPIO_CTRL_KEY_SWITCH_IOMUX, DL_GPIO_INVERSION_DISABLE,
      DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
      DL_GPIO_WAKEUP_DISABLE);
  DL_GPIO_disableOutput(GPIO_CTRL_KEY_SWITCH_PORT, GPIO_CTRL_KEY_SWITCH_PIN);
  NVIC_SetPriority(GPIOA_INT_IRQn, 0);
  Encoder_Init(&encoder_);
  Encoder_StartCaptureTimers();
  Encoder_StartReverseGpioInputs();
  IMU_Init();

  NVIC_EnableIRQ(MyGPTIMER1_INST_INT_IRQN);
  NVIC_SetPriority(MyGPTIMER1_INST_INT_IRQN, 1);
  DL_TimerG_startCounter(MyGPTIMER1_INST);

  UART1_CommInit();

  DL_GPIO_setPins(GPIOB, GPIO_CTRL_POWER_PB19_PIN);
  Motor_StopAll();
  DL_TimerA_startCounter(PWM_MOTOR_INST);

  PID_Motor_Init();
  PID_Motor_SetTargetSpeed(DEFAULT_TARGET_SPEED_MPS, DEFAULT_TARGET_SPEED_MPS,
                           DEFAULT_TARGET_SPEED_MPS, DEFAULT_TARGET_SPEED_MPS);
  PathNav_Init();

  bool debounced_key_pressed = Key_ReadPressed();
  bool last_raw_key_pressed = debounced_key_pressed;
  uint8_t key_debounce_ticks = 0;
  while (1) {
    // 处理位置闭环和电机 PID 控制
    if (Control_ConsumeTick()) {
      PathNav_Update(CONTROL_DT_SEC);
      PID_Motor_Update(CONTROL_DT_SEC);
      Led_ProcessTick();
    }

    UART1_ConsumeRxLine();

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
        if (g_key_count == PATH_START_KEY_COUNT) {
          PathNav_Start();
        }
        App_ProcessReport();
      }
    }

    App_ProcessReport();

    // 进入休眠，等待下一次 10ms 的 MyGPTIMER1 或 UART1 中断唤醒
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
    reverse_vel_LF_mps = encoder_.reverse_vel_LF * ENC_TO_MPS_K;
    reverse_vel_LR_mps = encoder_.reverse_vel_LR * ENC_TO_MPS_K;
    reverse_vel_RF_mps = encoder_.reverse_vel_RF * ENC_TO_MPS_K;
    reverse_vel_RR_mps = encoder_.reverse_vel_RR * ENC_TO_MPS_K;
    IMU_Update(CONTROL_DT_SEC);
    Odom_Update(CONTROL_DT_SEC);
    control_tick = true;

    DL_TimerG_clearInterruptStatus(MyGPTIMER1_INST,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);
    break;

  default:
    break;
  }
}

void GROUP1_IRQHandler(void) {
  uint32_t status = DL_GPIO_getEnabledInterruptStatus(
      GPIO_ENCODERS_PORT, ENCODER_REVERSE_GPIO_PINS);

  if ((status & GPIO_ENCODERS_ENCODER_M1_B_PIN) != 0U) {
    encoder_.reverse_count_RF++;
  }
  if ((status & GPIO_ENCODERS_ENCODER_M2_B_PIN) != 0U) {
    encoder_.reverse_count_LF++;
  }
  if ((status & GPIO_ENCODERS_ENCODER_M3_B_PIN) != 0U) {
    encoder_.reverse_count_LR++;
  }
  if ((status & GPIO_ENCODERS_ENCODER_M4_B_PIN) != 0U) {
    encoder_.reverse_count_RR++;
  }

  DL_GPIO_clearInterruptStatus(GPIO_ENCODERS_PORT, status);
}

void UART1_INST_IRQHandler(void) {
  uint8_t data = 0U;

  switch (DL_UART_Main_getPendingInterrupt(UART1_INST)) {
  case DL_UART_MAIN_IIDX_RX:
  case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
    while (DL_UART_Main_receiveDataCheck(UART1_INST, &data)) {
      UART1_ReceiveByte(data);
    }
    DL_UART_Main_clearInterruptStatus(UART1_INST,
                                      DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    break;

  case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
  case DL_UART_MAIN_IIDX_BREAK_ERROR:
  case DL_UART_MAIN_IIDX_PARITY_ERROR:
  case DL_UART_MAIN_IIDX_FRAMING_ERROR:
    DL_UART_Main_clearInterruptStatus(UART1_INST,
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

static void UART1_CommInit(void) {
  DL_UART_Main_enableInterrupt(UART1_INST,
                               DL_UART_MAIN_INTERRUPT_RX |
                                   DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
  NVIC_ClearPendingIRQ(UART1_INST_INT_IRQN);
  NVIC_SetPriority(UART1_INST_INT_IRQN, 2);
  NVIC_EnableIRQ(UART1_INST_INT_IRQN);
}

static void UART1_SendString(const char *text) {
  while (*text != '\0') {
    DL_UART_Main_transmitDataBlocking(UART1_INST, (uint8_t)*text);
    text++;
  }
}

static void UART1_ConsumeRxLine(void) {
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

  if (ParseTwistLine(line, &vx, &vyaw) && !path_nav_active) {
    PID_Motor_SetTwist(vx, vyaw);
  }
}

static void UART1_ReceiveByte(uint8_t data) {
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

static void Odom_Update(float dt) {
  if (dt <= 0.0f) {
    return;
  }

  float signed_vel_LF =
      Odom_GetSignedWheelSpeed(vel_LF_mps, reverse_vel_LF_mps);
  float signed_vel_LR =
      Odom_GetSignedWheelSpeed(vel_LR_mps, reverse_vel_LR_mps);
  float signed_vel_RF =
      Odom_GetSignedWheelSpeed(vel_RF_mps, reverse_vel_RF_mps);
  float signed_vel_RR =
      Odom_GetSignedWheelSpeed(vel_RR_mps, reverse_vel_RR_mps);

  float left_mps = (signed_vel_LF + signed_vel_LR) * 0.5f;
  float right_mps = (signed_vel_RF + signed_vel_RR) * 0.5f;
  float vx_mps = (left_mps + right_mps) * 0.5f;
  float previous_yaw_rad = odom_yaw_rad;

  odom_yaw_rad = imu_yaw_rad;
  odom_vyaw_radps = imu_gyro_z_dps * DEG_TO_RAD;

  float delta_yaw_rad = Odom_NormalizeYaw(odom_yaw_rad - previous_yaw_rad);
  float mid_yaw_rad = previous_yaw_rad + (delta_yaw_rad * 0.5f);

  odom_x_m += vx_mps * cosf(mid_yaw_rad) * dt;
  odom_y_m += vx_mps * sinf(mid_yaw_rad) * dt;
  odom_vx_mps = vx_mps;
}

static float Odom_GetSignedWheelSpeed(float forward_mps, float reverse_mps) {
  return (reverse_mps > forward_mps) ? -reverse_mps : forward_mps;
}

static float Odom_NormalizeYaw(float yaw_rad) {
  while (yaw_rad > PI) {
    yaw_rad -= 2.0f * PI;
  }
  while (yaw_rad < -PI) {
    yaw_rad += 2.0f * PI;
  }
  return yaw_rad;
}

static void IMU_Init(void) {
  uint8_t whoami = 0U;

  IMU_CsHigh();
  IMU_DelayMs(10U);

  if (!IMU_ReadReg(LSM6DSR_REG_WHO_AM_I, &whoami) ||
      (whoami != LSM6DSR_WHO_AM_I_VAL)) {
    imu_ready = false;
    return;
  }

  IMU_WriteReg(LSM6DSR_REG_CTRL3_C, LSM6DSR_CTRL3_C_SW_RESET);
  IMU_DelayMs(100U);
  IMU_WriteReg(LSM6DSR_REG_CTRL9_XL, LSM6DSR_CTRL9_XL_I3C_DISABLE);
  IMU_WriteReg(LSM6DSR_REG_CTRL3_C,
               LSM6DSR_CTRL3_C_BDU | LSM6DSR_CTRL3_C_IF_INC);
  IMU_WriteReg(LSM6DSR_REG_CTRL2_G,
               (uint8_t)((LSM6DSR_GYRO_ODR_104HZ << 4) |
                         LSM6DSR_GYRO_FS_250DPS));
  IMU_DelayMs(100U);

  IMU_CalibrateGyroZ();
  imu_yaw_rad = 0.0f;
  imu_gyro_z_dps = 0.0f;
  imu_ready = true;
}

static void IMU_Update(float dt) {
  if (!imu_ready || (dt <= 0.0f)) {
    return;
  }

  int16_t raw_gz = IMU_ReadGyroZRaw();
  float gz_dps = ((float)raw_gz * LSM6DSR_GYRO_SENS_250DPS) -
                 imu_gyro_z_bias_dps;

  if (fabsf(gz_dps) < IMU_GYRO_Z_DEADBAND_DPS) {
    gz_dps = 0.0f;
  }

  imu_gyro_z_dps = IMU_YAW_SIGN * gz_dps;
  imu_yaw_rad = Odom_NormalizeYaw(imu_yaw_rad +
                                  (imu_gyro_z_dps * DEG_TO_RAD * dt));
}

static void IMU_CalibrateGyroZ(void) {
  float sum_gz_dps = 0.0f;

  for (uint32_t i = 0U; i < IMU_CALIB_SAMPLES; i++) {
    int16_t raw_gz = IMU_ReadGyroZRaw();
    sum_gz_dps += (float)raw_gz * LSM6DSR_GYRO_SENS_250DPS;
    IMU_DelayMs(IMU_CALIB_SAMPLE_DELAY_MS);
  }

  imu_gyro_z_bias_dps = sum_gz_dps / (float)IMU_CALIB_SAMPLES;
}

static void IMU_DelayMs(uint32_t ms) {
  while (ms > 0U) {
    DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
    ms--;
  }
}

static bool IMU_ReadReg(uint8_t reg, uint8_t *value) {
  return IMU_ReadMulti(reg, value, 1U);
}

static bool IMU_WriteReg(uint8_t reg, uint8_t value) {
  IMU_CsLow();
  (void)IMU_SpiTransfer((uint8_t)(reg & 0x7FU));
  (void)IMU_SpiTransfer(value);
  IMU_CsHigh();
  return true;
}

static bool IMU_ReadMulti(uint8_t reg, uint8_t *buf, uint16_t len) {
  if ((buf == NULL) || (len == 0U)) {
    return false;
  }

  IMU_CsLow();
  (void)IMU_SpiTransfer((uint8_t)(reg | 0x80U));
  for (uint16_t i = 0U; i < len; i++) {
    buf[i] = IMU_SpiTransfer(0xFFU);
  }
  IMU_CsHigh();
  return true;
}

static int16_t IMU_ReadGyroZRaw(void) {
  uint8_t buf[2] = {0U, 0U};

  if (!IMU_ReadMulti(LSM6DSR_REG_OUTZ_L_G, buf, 2U)) {
    return 0;
  }

  return (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
}

static void IMU_CsLow(void) {
  DL_GPIO_clearPins(GPIO_CTRL_SPI_CS_GPIO_PORT, GPIO_CTRL_SPI_CS_GPIO_PIN);
}

static void IMU_CsHigh(void) {
  DL_GPIO_setPins(GPIO_CTRL_SPI_CS_GPIO_PORT, GPIO_CTRL_SPI_CS_GPIO_PIN);
}

static uint8_t IMU_SpiTransfer(uint8_t data) {
  DL_SPI_transmitDataBlocking8(SPI_0_INST, data);
  return DL_SPI_receiveDataBlocking8(SPI_0_INST);
}

static void PathNav_Init(void) {
  PosPid_Init(&pos_dist_pid, 1.5f, 0.0f, 0.5f, 0.10f, 0.25f, 0.005f);
  PosPid_Init(&pos_yaw_pid, 2.0f, 0.0f, 0.8f, 0.25f, 0.50f, 0.02f);
  PathNav_Clear();
}

static void PathNav_Start(void) {
  path_origin_x_m = odom_x_m;
  path_origin_y_m = odom_y_m;
  path_origin_yaw_rad = odom_yaw_rad;
  PathNav_Generate();

  if (path_point_count == 0U) {
    path_nav_active = false;
    path_nav_finished = true;
    return;
  }

  path_current_index = 0U;
  path_nav_active = true;
  path_nav_finished = false;
  PosPid_Reset(&pos_dist_pid);
  PosPid_Reset(&pos_yaw_pid);
}

static void PathNav_Update(float dt) {
  if (!path_nav_active || (path_point_count == 0U)) {
    return;
  }

  PathPoint target = path_points[path_current_index];
  float dx_m = target.x_m - odom_x_m;
  float dy_m = target.y_m - odom_y_m;
  float dist_m = sqrtf((dx_m * dx_m) + (dy_m * dy_m));

  while ((dist_m < PATH_LOOKAHEAD_M) &&
         (path_current_index < (path_point_count - 1U))) {
    path_current_index++;
    target = path_points[path_current_index];
    dx_m = target.x_m - odom_x_m;
    dy_m = target.y_m - odom_y_m;
    dist_m = sqrtf((dx_m * dx_m) + (dy_m * dy_m));
  }

  bool final_point = (path_current_index >= (path_point_count - 1U));
  float desired_yaw_rad = (dist_m > 0.01f) ? atan2f(dy_m, dx_m)
                                           : target.yaw_rad;
  if (final_point && (dist_m < PATH_FINISH_POS_M)) {
    desired_yaw_rad = target.yaw_rad;
  }

  float yaw_error_abs = PathNav_YawErrorAbs(desired_yaw_rad, odom_yaw_rad);
  float vx_mps = 0.0f;
  float vyaw_radps = PosPid_Calculate(&pos_yaw_pid, desired_yaw_rad,
                                      odom_yaw_rad, dt, true, PI);

  if (final_point && (dist_m < PATH_FINISH_POS_M) &&
      (yaw_error_abs < PATH_FINISH_YAW_RAD)) {
    path_nav_active = false;
    path_nav_finished = true;
    PID_Motor_SetTwist(0.0f, 0.0f);
    PosPid_Reset(&pos_dist_pid);
    PosPid_Reset(&pos_yaw_pid);
    Led_StartFlash(3U);
    return;
  }

  if ((!final_point || (dist_m >= PATH_FINISH_POS_M)) &&
      (yaw_error_abs < PATH_ALIGN_YAW_RAD)) {
    vx_mps = PosPid_Calculate(&pos_dist_pid, dist_m, 0.0f, dt, false, PI);
  }

  PID_Motor_SetTwist(vx_mps, vyaw_radps);
}

static void PathNav_Generate(void) {
  PathNav_Clear();

  const float start_x = 0.0f;
  const float start_y = 0.0f;
  const float b_x = start_x + PATH_AB_LEN_M;
  const float b_y = start_y;
  const float c_x = b_x;
  const float c_y = b_y - (2.0f * PATH_BC_RADIUS_M);
  const float d_x = c_x - PATH_CD_LEN_M;
  const float d_y = c_y;

  PathNav_AddLocalPoint(start_x, start_y, 0.0f);
  PathNav_AppendLocalLine(start_x, start_y, b_x, b_y, PATH_STEP_M);
  PathNav_AppendLocalArc(b_x, b_y - PATH_BC_RADIUS_M, PATH_BC_RADIUS_M,
                         PI / 2.0f, -PI / 2.0f, PATH_STEP_M);
  PathNav_AppendLocalLine(c_x, c_y, d_x, d_y, PATH_STEP_M);
  PathNav_AppendLocalArc(d_x, d_y + PATH_DA_RADIUS_M, PATH_DA_RADIUS_M,
                         -PI / 2.0f, -3.0f * PI / 2.0f, PATH_STEP_M);

  float end_a_x = d_x + PATH_DA_RADIUS_M * cosf(-3.0f * PI / 2.0f);
  float end_a_y = (d_y + PATH_DA_RADIUS_M) +
                  PATH_DA_RADIUS_M * sinf(-3.0f * PI / 2.0f);
  PathNav_AppendLocalLine(end_a_x, end_a_y,
                          end_a_x + PATH_EXTRA_FORWARD_M, end_a_y,
                          PATH_STEP_M);
}

static void PathNav_Clear(void) {
  path_point_count = 0U;
  path_current_index = 0U;
}

static void PathNav_AddLocalPoint(float local_x_m, float local_y_m,
                                  float local_yaw_rad) {
  if (path_point_count >= PATH_MAX_POINTS) {
    return;
  }

  float cos_yaw = cosf(path_origin_yaw_rad);
  float sin_yaw = sinf(path_origin_yaw_rad);
  PathPoint *point = &path_points[path_point_count];

  point->x_m = path_origin_x_m + (local_x_m * cos_yaw) -
               (local_y_m * sin_yaw);
  point->y_m = path_origin_y_m + (local_x_m * sin_yaw) +
               (local_y_m * cos_yaw);
  point->yaw_rad = Odom_NormalizeYaw(path_origin_yaw_rad + local_yaw_rad);
  path_point_count++;
}

static void PathNav_AppendLocalLine(float sx, float sy, float ex, float ey,
                                    float step) {
  float dx = ex - sx;
  float dy = ey - sy;
  float dist = sqrtf((dx * dx) + (dy * dy));
  int steps = (int)ceilf(dist / step);

  if (steps <= 0) {
    return;
  }

  float yaw = atan2f(dy, dx);
  for (int i = 1; i <= steps; i++) {
    float t = (float)i / (float)steps;
    PathNav_AddLocalPoint(sx + (dx * t), sy + (dy * t), yaw);
  }
}

static void PathNav_AppendLocalArc(float cx, float cy, float radius,
                                   float start_angle, float end_angle,
                                   float step) {
  float diff = end_angle - start_angle;
  float arc_len = fabsf(diff) * radius;
  int steps = (int)ceilf(arc_len / step);

  if (steps <= 0) {
    return;
  }

  for (int i = 1; i <= steps; i++) {
    float t = (float)i / (float)steps;
    float angle = start_angle + (t * diff);
    float px = cx + (radius * cosf(angle));
    float py = cy + (radius * sinf(angle));
    float yaw = angle - (PI / 2.0f);

    PathNav_AddLocalPoint(px, py, Odom_NormalizeYaw(yaw));
  }
}

static float PathNav_YawErrorAbs(float target_yaw_rad, float real_yaw_rad) {
  float error = Odom_NormalizeYaw(target_yaw_rad - real_yaw_rad);
  return fabsf(error);
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

  if ((g_key_count >= PATH_START_KEY_COUNT) && !report_100_sent) {
    UART1_SendString("100\r\n");
    Led_StartFlash(2U);
    report_100_sent = true;
  }

  if ((g_switch_state == SWITCH_STATE_2) && (g_key_count >= 10) &&
      !report_200_sent) {
    UART1_SendString("200\r\n");
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

static void Encoder_StartReverseGpioInputs(void) {
  DL_GPIO_setLowerPinsPolarity(
      GPIO_ENCODERS_PORT, DL_GPIO_PIN_4_EDGE_RISE | DL_GPIO_PIN_13_EDGE_RISE |
                              DL_GPIO_PIN_14_EDGE_RISE);
  DL_GPIO_setUpperPinsPolarity(GPIO_ENCODERS_PORT, DL_GPIO_PIN_25_EDGE_RISE);

  DL_GPIO_clearInterruptStatus(GPIO_ENCODERS_PORT, ENCODER_REVERSE_GPIO_PINS);
  DL_GPIO_enableInterrupt(GPIO_ENCODERS_PORT, ENCODER_REVERSE_GPIO_PINS);

  NVIC_SetPriority(GPIOA_INT_IRQn, 0);
  NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
  NVIC_EnableIRQ(GPIOA_INT_IRQn);
}
