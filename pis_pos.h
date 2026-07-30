#ifndef PIS_POS_H_
#define PIS_POS_H_

#include <stdbool.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef struct {
  float kp;
  float kd;
  float two_accel;
  float delta;
  float max_out;
  float deadzone;
  float x0;
  float y0;
  float last_error;
} PosPid;

void PosPid_Init(PosPid *pid, float kp, float kd, float accel, float delta,
                 float max_out, float deadzone);
void PosPid_Reset(PosPid *pid);
float PosPid_Calculate(PosPid *pid, float target, float real, float dt,
                       bool normalization, float unit);

#endif /* PIS_POS_H_ */
