
#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "config.h"
#include "main.h"


void debug_comm(MotorState_t *pMS);

// -------------------------------------------------------- fast log

// how to use it
// (1) fill i16_fast_log_sample_buffer and call fast_log_data()
// (2) send 'F' to the debug UART interface

#if defined(FAST_LOOP_LOG)
#define FAST_LOG_N_DATA_STREAMS 2
#define FAST_LOG_N_SAMPLES 100
#define FAST_LOG_SKIP 3
extern uint8_t ui8_fast_log_state;
extern int16_t i16_fast_log_sample_buffer[FAST_LOG_N_DATA_STREAMS];
void fast_log_data();
#endif
// --------------------------------------------------------



#endif // _DEBUG_H_