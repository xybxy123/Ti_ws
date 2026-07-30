#include "pid_motor.h"
#include "motor.h"

float PID_MOTOR_KP = 0.0f;
float PID_MOTOR_KI = 2000.0f;
float PID_MOTOR_KD = 0.0f;
float PID_MOTOR_OUTPUT_LIMIT = 200.0f;
float PID_MOTOR_DELTA_LIMIT = 1.0f;

PID_Struct pid_LF;
PID_Struct pid_LR;

PID_Struct pid_RF;
PID_Struct pid_RR;

float p = 2.0f;
float i = 1000.0f;
float d = 0.0f;
float out_limit = 200.0f;
float delta_limit = 1.0f;

extern volatile float vel_LF_mps;
extern volatile float vel_LR_mps;
extern volatile float vel_RF_mps;
extern volatile float vel_RR_mps;
extern volatile float reverse_vel_LF_mps;
extern volatile float reverse_vel_LR_mps;
extern volatile float reverse_vel_RF_mps;
extern volatile float reverse_vel_RR_mps;

static float PID_Abs(float value);
static float PID_Clamp(float value, float limit);
static int8_t PID_OutputToMotorSpeed(float output);
static float PID_MotorUpdateOne(PID_Struct *pid, float forward_input,
                                float reverse_input, float dt);
static void PID_MotorInitOne(PID_Struct *pid);
static void PID_MotorInitTwo(PID_Struct *pid);

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
  PID_MotorInitTwo(&pid_LF);
  PID_MotorInitTwo(&pid_LR);
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

void PID_Motor_Update(float dt) {
  float out_LF =
      PID_MotorUpdateOne(&pid_LF, vel_LF_mps, reverse_vel_LF_mps, dt);
  float out_LR =
      PID_MotorUpdateOne(&pid_LR, vel_LR_mps, reverse_vel_LR_mps, dt);
  float out_RF =
      PID_MotorUpdateOne(&pid_RF, vel_RF_mps, reverse_vel_RF_mps, dt);
  float out_RR =
      PID_MotorUpdateOne(&pid_RR, vel_RR_mps, reverse_vel_RR_mps, dt);

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

static float PID_MotorUpdateOne(PID_Struct *pid, float forward_input,
                                float reverse_input, float dt) {
  if (PID_Abs(pid->target) <= 0.02f) {
    PID_Reset(pid);
    return 0.0f;
  }

  if (pid->target < 0.0f) {
    pid->input = -reverse_input;
  } else {
    pid->input = forward_input;
  }

  float output = PID_Calculate(pid, dt);
  if ((pid->target > 0.0f) && (output < 0.0f)) {
    output = 0.0f;
  } else if ((pid->target < 0.0f) && (output > 0.0f)) {
    output = 0.0f;
  }

  return output;
}

static void PID_MotorInitOne(PID_Struct *pid) {
  PID_Init(pid, PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
           PID_MOTOR_OUTPUT_LIMIT, PID_MOTOR_DELTA_LIMIT);
}

static void PID_MotorInitTwo(PID_Struct *pid) {
  PID_Init(pid, p, i, d, out_limit, delta_limit);
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