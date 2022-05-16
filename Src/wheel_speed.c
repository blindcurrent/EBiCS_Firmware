
#include "wheel_speed.h"

uint8_t ui8_internal_SPEED_control_flag = 0;
static const uint8_t ui8_speed_shift = 1;

void init_wheel_speed_data(WheelSpeedData_t *pWheelSpeedData)
{
    pWheelSpeedData->uint32_SPEED_kmh_x10 = 0;
    pWheelSpeedData->ui16_wheel_time_ms = 0x0DAC;
}



uint32_t internal_tics_to_speedx100(uint32_t tics)
{
    // returns the speed in kmh x 100
    return WHEEL_CIRCUMFERENCE * 50 * 3600 / (6 * TRANSMISSION_RATIO * tics);
}

//int32_t speed_to_tics(uint8_t speed)
//{
//    return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * TRANSMISSION_RATIO * speed * 10);
//}

//int8_t tics_to_speed(uint32_t tics)
//{
//    return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * TRANSMISSION_RATIO * tics * 10);
//}


void process_wheel_speed_data(const HallData_t *pHallData, WheelSpeedData_t *pWheelSpeedData)
{
    static uint32_t uint32_SPEEDx100_kmh_cumulated = 0;


    if (ui8_internal_SPEED_control_flag)
    {

        if(pHallData->uint32_tics_filtered < HALL_TIMEOUT)
        {
            uint32_SPEEDx100_kmh_cumulated -= uint32_SPEEDx100_kmh_cumulated >> ui8_speed_shift;
            uint32_SPEEDx100_kmh_cumulated += internal_tics_to_speedx100(pHallData->uint32_tics_filtered);
            pWheelSpeedData->uint32_SPEED_kmh_x10 = (uint32_SPEEDx100_kmh_cumulated >> ui8_speed_shift) / 10;
        
            // period [s] = tics x 6 x TRANSMISSION_RATIO / frequency    (frequency = 500kHz)
            //pWheelSpeedData->ui16_wheel_time_ms = (pHallData->uint32_tics_filtered * 6 * TRANSMISSION_RATIO) / (500);
            uint32_t temp = pHallData->uint32_tics_filtered;
            temp = temp * 6 * TRANSMISSION_RATIO / 500;
            pWheelSpeedData->ui16_wheel_time_ms = (uint16_t) temp;
        }
        ui8_internal_SPEED_control_flag = 0;
    }
    else if(pHallData->uint32_tics_filtered >= HALL_TIMEOUT)
    {
        uint32_SPEEDx100_kmh_cumulated = 0;
        pWheelSpeedData->uint32_SPEED_kmh_x10 = 0;
        pWheelSpeedData->ui16_wheel_time_ms = 0x0DAC;
    }

}
