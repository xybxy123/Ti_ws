#include "pid_motor.h"
#include "motor.h"

#define PID_MOTOR_KP (200.0f)
#define PID_MOTOR_KI (7000.0f)
#define PID_MOTOR_KD (0.3f)
#define PID_MOTOR_OUTPUT_LIMIT (20.0f)
#define PID_MOTOR_DELTA_LIMIT (0.2f)

PID_Struct pid_LF;
PID_Struct pid_LR;
PID_Struct pid_RF;
PID_Struct pid_RR;

extern volatile float vel_LF_mps;
extern volatile float vel_LR_mps;
extern volatile float vel_RF_mps;
extern volatile float vel_RR_mps;

static float PID_Abs(float value);
static float PID_Clamp(float value, float limit);
static int8_t PID_OutputToMotorSpeed(float output);
static void PID_MotorInitOne(PID_Struct *pid);

void PID_Init(PID_Struct *pid, float p, float i, float d, float out_limit,
              float delta_limit) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->target = 0.0f;
  pid->output_limit = PID_Abs(out_limit);
  pid->delta_limit = PID_Abs(delta_limit);
  PID_Reset(pid);
}

void PID_Reset(PID_Struct *pid) {
  pid->input = 0.0f;
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->last_last_error = 0.0f;
  pid->output = 0.0f;
  pid->delta_output = 0.0f;
}

void PID_SetTarget(PID_Struct *pid, float target) { pid->target = target; }

float PID_Calculate(PID_Struct *pid, float dt) {
  if (dt <= 0.0f) {
    return pid->output;
  }

  float error = pid->target - pid->input;
  float delta_error = error - pid->last_error;
  float delta_delta_error =
      error - (2.0f * pid->last_error) + pid->last_last_error;

  pid->delta_output = (pid->kp * delta_error) + (pid->ki * error * dt) +
                      (pid->kd * delta_delta_error / dt);
  pid->delta_output = PID_Clamp(pid->delta_output, pid->delta_limit);

  pid->output = PID_Clamp(pid->output + pid->delta_output, pid->output_limit);
  pid->last_last_error = pid->last_error;
  pid->last_error = error;
  pid->error = error;

  return pid->output;
}

void PID_Motor_Init(void) {
  PID_MotorInitOne(&pid_LF);
  PID_MotorInitOne(&pid_LR);
  PID_MotorInitOne(&pid_RF);
  PID_MotorInitOne(&pid_RR);
}

void PID_Motor_SetTargetSpeed(float speed_LF, float speed_LR, float speed_RF,
                              float speed_RR) {
  PID_SetTarget(&pid_LF, speed_LF);
  PID_SetTarget(&pid_LR, speed_LR);
  PID_SetTarget(&pid_RF, speed_RF);
  PID_SetTarget(&pid_RR, speed_RR);
}

// void PID_Motor_Update(float dt) {
//   pid_LF.input = vel_LF_mps;
//   pid_LR.input = vel_LR_mps;
//   pid_RF.input = vel_RF_mps;
//   pid_RR.input = vel_RR_mps;

//   Motor_SetSpeed(PID_OutputToMotorSpeed(PID_Calculate(&pid_LF, dt)),
//                  PID_OutputToMotorSpeed(PID_Calculate(&pid_LR, dt)),
//                  PID_OutputToMotorSpeed(PID_Calculate(&pid_RF, dt)),
//                  PID_OutputToMotorSpeed(PID_Calculate(&pid_RR, dt)));
// }

void PID_Motor_Update(float dt) {
  // 因為沒有反向，編碼器讀回來的速度永遠是正數
  pid_LF.input = vel_LF_mps;
  pid_LR.input = vel_LR_mps;
  pid_RF.input = vel_RF_mps;
  pid_RR.input = vel_RR_mps;

  float out_LF, out_LR, out_RF, out_RR;

  // ======== 左前輪 ========
  if (pid_LF.target <= 0.02f) {
    out_LF = 0.0f; // 速度小於 0.02，強制輸出 0 (對應 50% 佔空比停止)
    PID_Reset(&pid_LF); // 清除歷史誤差，防止積分累積導致重新啟動時暴衝
  } else {
    out_LF = PID_Calculate(&pid_LF, dt);
    if (out_LF < 0.0f)
      out_LF = 0.0f; // 限制只輸出正轉，過濾掉任何後退指令
  }

  // ======== 左後輪 ========
  if (pid_LR.target <= 0.02f) {
    out_LR = 0.0f;
    PID_Reset(&pid_LR);
  } else {
    out_LR = PID_Calculate(&pid_LR, dt);
    if (out_LR < 0.0f)
      out_LR = 0.0f;
  }

  // ======== 右前輪 ========
  if (pid_RF.target <= 0.02f) {
    out_RF = 0.0f;
    PID_Reset(&pid_RF);
  } else {
    out_RF = PID_Calculate(&pid_RF, dt);
    if (out_RF < 0.0f)
      out_RF = 0.0f;
  }

  // ======== 右後輪 ========
  if (pid_RR.target <= 0.02f) {
    out_RR = 0.0f;
    PID_Reset(&pid_RR);
  } else {
    out_RR = PID_Calculate(&pid_RR, dt);
    if (out_RR < 0.0f)
      out_RR = 0.0f;
  }

  // 下發給馬達
  // 這裡的 out 為 0 時，會進入 motor.c 被轉換成 PWM 500 (50% 佔空比)
  Motor_SetSpeed(PID_OutputToMotorSpeed(out_LF), PID_OutputToMotorSpeed(out_LR),
                 PID_OutputToMotorSpeed(out_RF),
                 PID_OutputToMotorSpeed(out_RR));
}

void PID_Motor_Stop(void) {
  PID_Motor_SetTargetSpeed(0.0f, 0.0f, 0.0f, 0.0f);
  PID_Reset(&pid_LF);
  PID_Reset(&pid_LR);
  PID_Reset(&pid_RF);
  PID_Reset(&pid_RR);
  Motor_StopAll();
}

static float PID_Abs(float value) { return (value < 0.0f) ? -value : value; }

static float PID_Clamp(float value, float limit) {
  if (limit <= 0.0f) {
    return value;
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static int8_t PID_OutputToMotorSpeed(float output) {
  output = PID_Clamp(output, 100.0f);
  return (int8_t)output;
}

static void PID_MotorInitOne(PID_Struct *pid) {
  PID_Init(pid, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
           PID_MOTOR_OUTPUT_LIMIT, PID_MOTOR_DELTA_LIMIT);
}

// ==============================================================================
// 差速运动学解算接口 (Differential Drive Kinematics)
// ==============================================================================

void PID_Motor_SetTwist(float vx, float vyaw) {
  // 当机器人向左转 (vyaw > 0) 时，右侧轮子速度需要大于左侧轮子
  // 角速度带来的轮边线速度偏移量为: V_delta = vyaw * (Track_Width / 2)
  float v_delta = vyaw * (ROBOT_TRACK_WIDTH_M / 2.0f);

  // 计算左右两侧的期望线速度
  float v_left = vx - v_delta;
  float v_right = vx + v_delta;

  // 将计算结果下发给四个电机
  // (由于是平行差速，左前和左后速度相同，右前和右后速度相同)
  PID_Motor_SetTargetSpeed(v_left, v_left, v_right, v_right);
}