
#ifndef ADC_MODULE_H_
#define ADC_MODULE_H_

#include "config.h"
#include "main.h"



void init_adc_data(ADC_Data_t* pADC_Data);

void calibrate_adc_offset(ADC_Data_t *pADC_Data);

void process_battery_data(BatteryData_t *pBatteryData, int32_t i32_i_d_mA, int32_t i32_i_q_mA, int32_t u_d, int32_t u_q);
void compute_remaining_distance(BatteryData_t *pBatteryData, uint32_t ui32_speed_kmh_x10);

void process_chip_temperature(TemperatureData_t* pChipTemperatureData);

void process_motor_temperature(TemperatureData_t* pMotorTemperatureData);


#endif //ADC_MODULE_H_