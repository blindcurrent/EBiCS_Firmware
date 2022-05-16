
#ifndef HALL_H_
#define HALL_H_

#include "config.h"
#include "main.h"

void init_hall_data(HallData_t* pHallData);

void process_hall_event(HallData_t* pHallData, uint16_t ui16_tim2_recent, uint8_t ui8_hall_state);
void estimate_rotor_angle(HallData_t* pHallData, uint16_t ui16_tim2_recent);

void autodetect(MotorState_t* pMS);











#endif // HALL_H_