#ifndef PID_MOTOR_H_
#define PID_MOTOR_H_

#include <stdint.h>

// ==================== 差速底盘运动学参数 ====================
// 单位统一定义为 米 (m)
#define ROBOT_TRACK_WIDTH_M (0.256f) // 左右轮距 256mm
// ========================================================

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

/*
 * @brief  平行差速底盘 速度控制 (ROS cmd_vel 接口)
 * @param  vx:   X轴线速度 (m/s)，正代表前进
 * @param  vyaw: Z轴角速度 (rad/s)，正代表向左转 (逆时针)
 */
void PID_Motor_SetTwist(float vx, float vyaw);

#endif /* PID_MOTOR_H_ */
