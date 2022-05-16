#include "pedal_sensor.h"

static const uint8_t uint8_pas_shift = 2;
static const uint8_t uint8_torque_shift = 4;
static uint32_t uint32_PAS_cumulated = PAS_TIMEOUT << uint8_pas_shift;
static uint32_t uint32_torque_adc_cumulated = 0;

void init_pedal_data(PedalData_t* PedalData)
{
    PedalData->ui8_PAS_flag = 0;
    PedalData->uint32_PAS_counter = PAS_TIMEOUT;
    PedalData->uint32_PAS = PAS_TIMEOUT;
    PedalData->uint32_PAS_omega_x10 = 0;
    PedalData->uint32_PAS_rpm = 0;
    PedalData->uint8_pedaling = 0;
    //
    PedalData->uint16_torque_adc = 0;
    PedalData->uint32_torque_Nm_x10 = 0;
}

void process_pedal_data(PedalData_t* PedalData)
{
    if (PedalData->ui8_PAS_flag)
    {
        if (PedalData->uint32_PAS_counter > 100)  // debounce
        {
            uint32_PAS_cumulated -= uint32_PAS_cumulated >> uint8_pas_shift;
            uint32_PAS_cumulated += PedalData->uint32_PAS_counter;
            PedalData->uint32_PAS = uint32_PAS_cumulated >> uint8_pas_shift;
                
            // omega = 2pi / T = 2pi / (32 * tics / 8000) = 500pi / tics = 1571 / tics
            PedalData->uint32_PAS_omega_x10 = (1571*10) / PedalData->uint32_PAS;
            // rpm = 60 / T = 60 / (32 * tics / 8000) = 15000 / tics
            PedalData->uint32_PAS_rpm = 15000 / PedalData->uint32_PAS;

            PedalData->uint32_PAS_counter = 0;
            PedalData->ui8_PAS_flag = 0;


            uint32_t ui32_reg_adc_value_shifted = PedalData->uint16_torque_adc << uint8_torque_shift;

            if (ui32_reg_adc_value_shifted > uint32_torque_adc_cumulated)
            {
                // accept rising values unfiltered
                uint32_torque_adc_cumulated = ui32_reg_adc_value_shifted;
            }
            else
            {
                // filter falling values
                uint32_torque_adc_cumulated -= uint32_torque_adc_cumulated >> uint8_torque_shift;
                uint32_torque_adc_cumulated += PedalData->uint16_torque_adc;
            }

            PedalData->uint32_torque_Nm_x10 = 10 * (uint32_torque_adc_cumulated >> uint8_torque_shift) / CAL_TORQUE;
        }
    }

    if ((PedalData->uint32_PAS_counter >= PAS_TIMEOUT) || 
            (PedalData->uint32_PAS_counter >= 4 * PedalData->uint32_PAS / 3))
    {
        PedalData->uint32_PAS = PAS_TIMEOUT;
        uint32_PAS_cumulated = PAS_TIMEOUT << uint8_pas_shift;
        PedalData->uint32_PAS_omega_x10 = 0;
        PedalData->uint32_PAS_rpm = 0;
        uint32_torque_adc_cumulated = 0;
        PedalData->uint32_torque_Nm_x10 = 0;
        //
        PedalData->uint8_pedaling = 0;
    }
    else
    {
        PedalData->uint8_pedaling = 1;
    }

}
