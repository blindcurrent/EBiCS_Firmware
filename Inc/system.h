
#ifndef _SYSTEM__H_
#define _SYSTEM__H_

#include "config.h"
#include "main.h"


uint32_t process_errors(MotorState_t *pMS);
void trigger_motor_error(motor_error_state_t err);
void enable_pwm(void);
void enable_pwm_with_dynamic_switchtime(void);

void disable_pwm(void);

uint32_t get_motor_error_state(void);
uint8_t is_pwm_enabled(void);

void init_watchdog(void);










#endif // _SYSTEM__H_