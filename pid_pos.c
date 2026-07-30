#include "pis_pos.h"
#include <math.h>

static float PosPid_Abs(float value) { return (value < 0.0f) ? -value : value; }

static float PosPid_Clamp(float value, float limit) {
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

void PosPid_Init(PosPid *pid, float kp, float kd, float accel, float delta,
                 float max_out, float deadzone) {
  if (pid == 0) {
    return;
  }

  pid->kp = kp;
  pid->kd = kd;
  pid->two_accel = PosPid_Abs(2.0f * accel);
  pid->delta = PosPid_Abs(delta);
  pid->max_out = PosPid_Abs(max_out);
  pid->deadzone = PosPid_Abs(deadzone);
  pid->last_error = 0.0f;

  if ((kp != 0.0f) && (accel != 0.0f)) {
    pid->x0 = powf(accel / (kp * sqrtf(2.0f * accel)), 2.0f);
  } else {
    pid->x0 = 0.0f;
  }

  pid->y0 = sqrtf(2.0f * accel * pid->x0);
  pid->x0 = delta - pid->x0;
  pid->y0 = (delta * kp) - pid->y0;
}

void PosPid_Reset(PosPid *pid) {
  if (pid == 0) {
    return;
  }

  pid->last_error = 0.0f;
}

float PosPid_Calculate(PosPid *pid, float target, float real, float dt,
                       bool normalization, float unit) {
  if (pid == 0) {
    return 0.0f;
  }
  if (dt < 0.0001f) {
    dt = 0.0001f;
  }

  unit = PosPid_Abs(unit);
  float error = target - real;

  if (normalization && (unit > 0.0f)) {
    float period = 2.0f * unit;
    if (error > unit) {
      error -= period;
    } else if (error < -unit) {
      error += period;
    }
  }

  if ((pid->deadzone != 0.0f) && (PosPid_Abs(error) < pid->deadzone)) {
    pid->last_error = error;
    return 0.0f;
  }

  float abs_error = PosPid_Abs(error);
  float sgn_error = (error >= 0.0f) ? 1.0f : -1.0f;
  float output = 0.0f;

  if (abs_error <= pid->delta) {
    output = error * pid->kp;
  } else {
    output = sgn_error * (sqrtf(pid->two_accel * PosPid_Abs(abs_error - pid->x0)) + pid->y0);
  }

  if (pid->kd != 0.0f) {
    output += ((error - pid->last_error) / dt) * pid->kd;
  }

  pid->last_error = error;
  return PosPid_Clamp(output, pid->max_out);
}
