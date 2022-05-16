/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_KM_N5236 (1<<6)
#define DISPLAY_TYPE_EBiCS (1<<5)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_618U (1<<3)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<4)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE_BAFANG (1<<2)							// For 'Blaupunkt' Display of Prophete Entdecker
#define DISPLAY_TYPE_KUNTENG (1<<1)							// For ASCII-Output in Debug mode
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);


//#define DISPLAY_TYPE DISPLAY_TYPE_DEBUG
//#define FAST_LOOP_LOG
#define DISPLAY_TYPE KM_N5236

#define ACTIVATE_WATCHDOG
//#define READ_SPEC_ANGLE_FROM_EEPROM

// ------------------------------- PAS MODULE
#define PAS_TIMEOUT 4000
#define CAL_TORQUE 13 // 15
#define TORQUE_ADC_OFFSET 550 //30   // 40

// ------------------------------- HALL MODULE
#define HALL_TIMEOUT 64000                 // 0.44 kmh
#define SIXSTEPTHRESHOLD_UP 11171          // 2.5 kmh
#define SIXSTEPTHRESHOLD_DOWN 13964        // 2 kmh
#define P_FACTOR_PLL  9 //10 //9  
#define I_FACTOR_PLL  7 //8 //7 

// ------------------------------- WHEEL_SPEED_MODULE
#define WHEEL_CIRCUMFERENCE 2234
#define POLE_PARIS 24
#define GEAR_RATIO 1            // direct drive
#define TRANSMISSION_RATIO 24  // POLE_PAIRS x GEAR_RATIO

// ------------------------------- ADC MODULE
#define CAL_BAT_V 25   // adcData[0] * CAL_BAT_V = voltage in mV
#define BATTERY_UNDER_VOLTAGE 382 //395         // in V x 10
#define BATTERY_CRITICAL_VOLTAGE  410 //405 //415    // in V x 10

#define CRITICAL_CHIP_TEMPERATURE 75    // in degrees
#define CHIP_OVER_TEMPERATURE 85        // in degrees

#define CHIP_TEMP_ADC_INDEX 5 // ADC_CHANNEL_TEMPSENSOR
//#define MOTOR_TEMP_SENSOR_INSTALLED 
#define MOTOR_TEMP_ADC_INDEX 1 // motor temp on sp

// ------------------------------- PHASE CURRENT SAMPLING

#define TRIGGER_OFFSET_ADC 100 // 50
#define TRIGGER_DEFAULT 2047   // change this if you change _T
#define PWM_DEAD_TIME 32

// ------------------------------- MOTOR
#define PSI_F_SHIFT10 28

// ------------------------------- SVPWM
#define _T_SHIFT 11 
#define _T 2048     // 2028
#define _U_MAX	2000 // 




#define CAL_I 50 //46 //60 //55      //38       
#define MOTOR_KV 26000


// ------------------------------- MOTOR CONTROL
//#define OPEN_LOOP_CONTROL

// ------------------------------- LIMITS
#define BOOST_TIME                          64
#define SPEEDLIMIT_KMH_X10                  470
#define MOTOR_POWER_MAX_mW                  900000
#define MOTOR_POWER_BOOST_DELTA_mW          0
#define MOTOR_POWER_MAX_REDUCED_mW          250000
#define BATTERY_CURRENT_MAX_mA              22000
#define BATTERY_CURRENT_BOOST_DELTA_mA      2000
#define BATTERY_CURRENT_MAX_REDUCED_mA      7000
#define PHASE_CURRENT_MAX_mA                60000
#define PHASE_CURRENT_BOOST_DELTA_mA        0
#define PHASE_CURRENT_MAX_REDUCED_mA        30000
#define REGEN_PHASE_CURRENT_MAX_mA          (-40000)



#define SPEEDLIMIT_WALK_ASSIST_KMH_X10 40
#define SPEC_ANGLE -167026406L //BionX IGH3 -143165476
#define TQ_ADC_INDEX 6  // AD1: 6, SP: 1


#define AUTODETECT 0


#endif /* CONFIG_H_ */
