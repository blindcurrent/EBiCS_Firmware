
#ifndef WHEEL_SPEED_H_
#define WHEEL_SPEED_H_

#include "config.h"
#include "main.h"

void init_wheel_speed_data(WheelSpeedData_t *pWheelSpeedData);

void process_wheel_speed_data(const HallData_t *pHallData, WheelSpeedData_t *pWheelSpeedData);

#endif // WHEEL_SPEED_H_
