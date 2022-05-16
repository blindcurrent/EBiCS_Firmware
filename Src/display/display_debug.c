#include "display_base.h"
#include "display_debug.h"
#include "main.h"
#include "config.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <stdarg.h>


#if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG

char print_buffer[DISPLAY_SIZE_TX_BUFFER];

extern UART_HandleTypeDef huart1;

extern uint8_t ui8_buffer_index1; // circular buffer read position
extern uint8_t ui8_buffer_index2; // circular buffer write position
extern uint8_t ui8_bytes_received;
extern uint8_t RxBuff[DISPLAY_SIZE_RX_BUFFER];
extern uint8_t TxBuff[DISPLAY_SIZE_TX_BUFFER];

extern uint8_t ui8_e_print_log_info_flag;

static uint8_t do_log = 1;

extern uint8_t ui8_fast_log_state;

static void transmit_buffer(uint8_t* data, uint16_t size)
{
    if(size > DISPLAY_SIZE_TX_BUFFER)
    {
        return;
    }
    
    if(ui8_g_UART_TxCplt_flag)
    {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*) data, size);
        ui8_g_UART_TxCplt_flag = 0;
    }
}

/**
   * @brief Formatted printing via dma uart -> non blocking.
   * Output can be suppressed with the do_log switch.
   */
void debug_printf(const char *format, ...)
{
    if(do_log == 0)
    {
        return;
    }

    va_list arg;
    va_start(arg, format);
    int c = vsnprintf(print_buffer, DISPLAY_SIZE_TX_BUFFER, format, arg);
    va_end(arg);
    if(c > DISPLAY_SIZE_TX_BUFFER)
        c = DISPLAY_SIZE_TX_BUFFER;
    transmit_buffer((uint8_t *)print_buffer, c);
}

/**
   * @brief Formatted printing via dma uart -> non blocking.
   */
void info_printf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    int c = vsnprintf(print_buffer, DISPLAY_SIZE_TX_BUFFER,format, arg);
    va_end(arg);                
    if(c > DISPLAY_SIZE_TX_BUFFER)
        c = DISPLAY_SIZE_TX_BUFFER;
    transmit_buffer((uint8_t *)print_buffer, c);
}

void Display_Service(MotorState_t *pMS)
{
    if(ui8_bytes_received >= DISPLAY_SIZE_TX_BUFFER - 1)
    {
        return;
    }

    // echo received message

    //for (uint8_t i = 0; i < bytes_received; ++i)
    //{
    //    TxBuff[i] = RxBuff[RX_BYTE(i)];
    //}
    //HAL_UART_Transmit_DMA(&huart1, TxBuff, bytes_received);

    
    for (uint8_t i = 0; i < ui8_bytes_received; ++i)
    {
        TxBuff[i] = RxBuff[RX_BYTE(i)];
    }
    TxBuff[ui8_bytes_received] = '\0';


    if(strncmp((char *) TxBuff, "go", 2) == 0)
    {
        pMS->ui8_go = !(pMS->ui8_go);
        if(pMS->ui8_go)
        {
            strcpy((char *) TxBuff, "go!\n");
            transmit_buffer(TxBuff, 4);
        }
        else
        {
            strcpy((char *) TxBuff, "stop\n");
            transmit_buffer(TxBuff, 5);
        }
    }
    else if(strncmp((char *) TxBuff, "a ", 2) == 0)
    {
        // parse value
        int8_t temp = atoi((char *) &(TxBuff[2]));
        if( (temp >= -3) && (temp <= 5) )
        {
            pMS->i8_assist_level = temp;
        }
        else
        {
            pMS->i8_assist_level = 0;
        }
        sprintf((char *) TxBuff, "i8_assist_level set to = %d\n", pMS->i8_assist_level);
        transmit_buffer(TxBuff, strlen((char *) TxBuff));
    }
    else if(strncmp((char *) TxBuff, "v ", 2) == 0)
    {
        // parse value
        pMS->ui16_dbg_value = atoi((char *) &(TxBuff[2]));
        if(pMS->ui16_dbg_value > 400)
        {
            pMS->ui16_dbg_value = 400;
        }
        sprintf((char *) TxBuff, "ui16_value = %u\n", pMS->ui16_dbg_value);
        transmit_buffer(TxBuff, strlen((char *) TxBuff));
    }
    else if(strncmp((char *) TxBuff, "x ", 2) == 0)
    {
        // parse value
        pMS->ui16_dbg_value2 = atoi((char *) &(TxBuff[2]));
        if(pMS->ui16_dbg_value2 > 20)
        {
            pMS->ui16_dbg_value2 = 20;
        }
        //sprintf((char *) TxBuff, "ui16_value2 = %u\n", pMS->ui16_dbg_value2);
        //transmit_buffer(TxBuff, strlen((char *) TxBuff));
    }
    else if(strncmp((char *) TxBuff, "l ", 2) == 0)
    {
        // parse value
        pMS->ui8_dbg_log_value = atoi((char *) &(TxBuff[2]));
        if(pMS->ui8_dbg_log_value > 10)
        {
            pMS->ui8_dbg_log_value = 0;
        }
        sprintf((char *) TxBuff, "dbg log = %u\n", pMS->ui8_dbg_log_value);
        transmit_buffer(TxBuff, strlen((char *) TxBuff));
        ui8_e_print_log_info_flag = 1;
    }
    else if(strncmp((char *) TxBuff, "F", 1) == 0)
    {
        if(ui8_fast_log_state == 0)
        {
            debug_printf("start sampling\n");
            ui8_fast_log_state = 1;
        }
    }
    else if(strncmp((char *) TxBuff, "log", 3) == 0)
    {
        pMS->ui8_log = !(pMS->ui8_log);
        do_log = pMS->ui8_log;
        ui8_g_UART_TxCplt_flag = 1;
        if(pMS->ui8_log)
        {
            strcpy((char *) TxBuff, "log on\n");
            transmit_buffer(TxBuff, 7);
        }
        else
        {
            strcpy((char *) TxBuff, "log off\n");
            transmit_buffer(TxBuff, 8);
        }
    }
    else if(strncmp((char *) TxBuff, "hello", 5) == 0)
    {
        strcpy((char *) TxBuff, "hello\n");
        ui8_g_UART_TxCplt_flag = 1;
        transmit_buffer(TxBuff, 6);
    }
    else if(strncmp((char *) TxBuff, "light", 5) == 0)
    {
        // toggle light
        pMS->ui8_lights = !(pMS->ui8_lights);
        if(pMS->ui8_lights)
        {
            strcpy((char *) TxBuff, "light on\n");
            transmit_buffer(TxBuff, 9);
        }
        else
        {
            strcpy((char *) TxBuff, "light off\n");
            transmit_buffer(TxBuff, 10);
        }
    }
    else
    {
        // echo the message
        for (uint8_t i = 0; i < ui8_bytes_received; ++i)
        {
            TxBuff[i] = RxBuff[RX_BYTE(i)];
        }
        transmit_buffer(TxBuff, ui8_bytes_received);
    }


    // set buffer read index to buffer write index
    ui8_buffer_index1 = ui8_buffer_index2;
}


#endif
