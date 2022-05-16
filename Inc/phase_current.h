
#ifndef PHASE_CURRENT_H_
#define PHASE_CURRENT_H_

#include "config.h"
#include "main.h"


void sample_phase_currents(int32_t* i32_ph1_current, int32_t* i32_ph2_current);
void dynamic_phase_current_sampling();

void transform_currents(int32_t i32_ph1_current, int32_t i32_ph2_current, q31_t q31_theta, MotorState_t* MS);

#endif // PHASE_CURRENT_H_
