#ifndef PID_MOTOR_H_
#define PID_MOTOR_H_

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float input;
  float target;
  float error;
  float last_error;
  float last_last_error;
  float output;
  float output_limit;
  float delta_output;
  float delta_limit;
} PID_Struct;

extern PID_Struct pid_LF;
extern PID_Struct pid_LR;
extern PID_Struct pid_RF;
extern PID_Struct pid_RR;

void PID_Init(PID_Struct *pid, float p, float i, float d, float out_limit,
              float delta_limit);
void PID_Reset(PID_Struct *pid);
void PID_SetTarget(PID_Struct *pid, float target);
float PID_Calculate(PID_Struct *pid, float dt);

void PID_Motor_Init(void);
void PID_Motor_SetTargetSpeed(float speed_LF, float speed_LR, float speed_RF,
                              float speed_RR);
void PID_Motor_Update(float dt);
void PID_Motor_Stop(void);

#endif /* PID_MOTOR_H_ */
