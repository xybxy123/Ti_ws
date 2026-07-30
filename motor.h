#ifndef MOTOR_H_
#define MOTOR_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 速度範圍: -100 到 100
 *  100 : 正向全速
 *    0 : 停止
 * -100 : 反向全速
 */

/**
 * @brief 單獨控制各個車輪
 */
void Motor_SetLeftFront(int8_t speed);  // 左前輪 (M2 -> PA9)
void Motor_SetLeftRear(int8_t speed);   // 左後輪 (M3 -> PB17)
void Motor_SetRightFront(int8_t speed); // 右前輪 (M1 -> PA8)
void Motor_SetRightRear(int8_t speed);  // 右後輪 (M4 -> PB2)

/**
 * @brief 同時設定四個車輪的轉速
 *
 * @param speed_LF 左前輪速度
 * @param speed_LR 左後輪速度
 * @param speed_RF 右前輪速度
 * @param speed_RR 右後輪速度
 */
void Motor_SetSpeed(int8_t speed_LF, int8_t speed_LR, int8_t speed_RF,
                    int8_t speed_RR);

/**
 * @brief 停止所有馬達
 */
void Motor_StopAll(void);

#endif /* MOTOR_H_ */