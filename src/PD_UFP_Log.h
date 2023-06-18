/**
 * PD_UFP_Log.h
 *
 *      Author: Ryan Ma
 *      Edited: Kai Liebich
 *
 *      PD_UFP_Log_c, extended from PD_UFP_c to provide logging function.
 *      Asynchronous, minimal impact on PD timing.
 * 
 */

#ifndef PD_UFP_LOG_H
#define PD_UFP_LOG_H

#include "PD_UFP.h"

struct status_log_t {
    uint16_t time;
    uint16_t msg_header;
    uint8_t obj_count;
    uint8_t status;
};

enum pd_log_level_t {
    PD_LOG_LEVEL_INFO,
    PD_LOG_LEVEL_VERBOSE
};

class PD_UFP_Log_c : public PD_UFP_c
{
    public:
        PD_UFP_Log_c(TwoWire &twoWire, pd_log_level_t log_level = PD_LOG_LEVEL_INFO);
        // Task
        //void print_status(Serial_ & serial);
        void print_status(HardwareSerial & serial);
        // Get
        int status_log_readline(char * buffer, int maxlen);

    protected:
        int status_log_readline_msg(char * buffer, int maxlen, status_log_t * log);
        int status_log_readline_src_cap(char * buffer, int maxlen);
        // Status log functions
        uint8_t status_log_obj_add(uint16_t header, uint32_t * obj);
        virtual void status_log_event(uint8_t status, uint32_t * obj);
        // status log event queue
        status_log_t status_log[16];    // array size must be power of 2 and <=256
        uint8_t status_log_read;
        uint8_t status_log_write;
        // status log object queue
        uint32_t status_log_obj[16];    // array size must be power of 2 and <=256
        uint8_t status_log_obj_read;
        uint8_t status_log_obj_write;
        // state variables
        pd_log_level_t status_log_level;
        uint8_t status_log_counter;        
        char status_log_time[8];
};

#endif