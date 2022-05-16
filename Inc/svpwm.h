

#ifndef SVPWM_H_
#define SVPWM_H_

#include "config.h"
#include "main.h"

void compute_svpwm_switchtime(MotorState_t* MS, q31_t q31_theta);


#endif // SVPWM_H_