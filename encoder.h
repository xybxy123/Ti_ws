#ifndef ENCODER_H_
#define ENCODER_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

// 机械参数
#define PI 3.1415926f
#define ENCODER_PPR 500U
#define REDUCTION_RATIO 34U
#define WHEEL_DIAM_M 0.065f
#define ENC_TO_MPS_K (PI * WHEEL_DIAM_M / (ENCODER_PPR * REDUCTION_RATIO))

/* 定义编码器数据结构体 */
typedef struct {
  volatile uint32_t count_LF; // 左前轮脉冲总数 (M2 -> PA21)
  volatile uint32_t count_LR; // 左后轮脉冲总数 (M3 -> PA26)
  volatile uint32_t count_RF; // 右前轮脉冲总数 (M1 -> PA12)
  volatile uint32_t count_RR; // 右后轮脉冲总数 (M4 -> PA28)

  float vel_LF; // 脉冲/秒
  float vel_LR;
  float vel_RF;
  float vel_RR;
} Encoder;

extern Encoder encoder_;

void Encoder_Init(Encoder *enc);
void Encoder_UpdateVelocity(Encoder *enc, float dt);

float Encoder_GetLeftFrontVel(Encoder *enc);
float Encoder_GetLeftRearVel(Encoder *enc);
float Encoder_GetRightFrontVel(Encoder *enc);
float Encoder_GetRightRearVel(Encoder *enc);

#endif /* ENCODER_H_ */
