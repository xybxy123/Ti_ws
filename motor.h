#ifndef MOTOR_H_
#define MOTOR_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/**
 * @brief 設定四個馬達的轉速
 * 
 * @param speed_m1 馬達 1 速度 (PA8), 範圍: -100 到 100
 * @param speed_m2 馬達 2 速度 (PA9), 範圍: -100 到 100
 * @param speed_m3 馬達 3 速度 (PB17), 範圍: -100 到 100
 * @param speed_m4 馬達 4 速度 (PB2), 範圍: -100 到 100
 * 
 * 說明： 100為正向全速，-100為反向全速，0為停止。
 */
void Motor_SetSpeed(int8_t speed_m1, int8_t speed_m2, int8_t speed_m3, int8_t speed_m4);

/**
 * @brief 停止所有馬達
 */
void Motor_StopAll(void);

#endif /* MOTOR_H_ */
