
/**
 * PD_UFP.h
 *
 *      Author: Ryan Ma
 *      Edited: Kai Liebich
 *
 * Minimalist USB PD Ardunio Library for PD Micro board
 * Only support UFP(device) sink only functionality
 * Requires FUSB302_UFP.h, PD_UFP_Protocol.h and Standard Arduino Library
 *
 * Support PD3.0 PPS
 * 
 */

#ifndef PD_UFP_H
#define PD_UFP_H

#include <Arduino.h>
#include <Wire.h>

#include "FUSB302_UFP.h"
#include "PD_UFP_Protocol.h"

/**
 * @brief Main class for user interaction to control a PD compliant USB-C power supply
 * 
 */
class PD_UFP_c
{
    public:
        /**
         * @brief Construct a new pd ufp c object
         * 
         * @param twoWire TwoWire (I2c) that the FUSB302 is connected to
         */
        PD_UFP_c(TwoWire &twoWire);

        /**
         * @brief Initializes the class for operation in normal, non PPS mode
         * 
         * @param int_pin       Pin number where the INT signal of the FUSB302 is connected to
         * @param power_option  Target power option (default = 5V)
         */
        void init(uint8_t int_pin, PD_power_option_t power_option = PD_POWER_OPTION_MAX_5V);

        /**
         * @brief Initializes the class for operation in PPS mode
         * 
         * @param int_pin       Pin number where the INT signal of the FUSB302 is connected to
         * @param PPS_voltage   Target PPS voltage in mV
         * @param PPS_current   Target PPS current in mA
         * @param power_option  Target power option (default = 5V)
         */
        void init_PPS(uint8_t int_pin, uint16_t PPS_voltage, uint8_t PPS_current, PD_power_option_t power_option = PD_POWER_OPTION_MAX_5V);

        /**
         * @brief Handles any upcoming PD related communication
         */
        void handle(void);

        /**
         * @brief Is the power supply is currently transitioning from one state to another?
         * 
         * @return True or false
         */
        bool is_ps_transition(void) { return send_request || wait_ps_rdy; }

        /**
         * @brief Returns the currently set voltage
         * 
         * @return Voltage in mV 
         */
        uint16_t get_voltage(void) { return ready_voltage; }
        /**
         * @brief Returns the currently set current
         * 
         * @return Current in mA 
         */
        uint16_t get_current(void) { return ready_current; }

        /**
         * @brief Returns the currently maximum available power
         * 
         * @return Power in mW 
         */
        uint32_t get_power(void) { return (uint32_t)ready_voltage * (uint32_t)ready_current; }
        
        /**
         * @brief Returns the current state of the power supply
         * 
         * @return Power supply state 
         */
        status_power_t get_ps_status(void) { return status_power; }

        /**
         * @brief Updates the target PPS values for voltage and current
         * 
         * @param PPS_voltage   Voltage in mV
         * @param PPS_current   Current in mA
         * @return              True if successful, else false
         */
        bool set_PPS(uint16_t PPS_voltage, uint8_t PPS_current);
        
        /**
         * @brief Updates the target power option
         * 
         * @param power_option Target power option
         */
        void set_power_option(PD_power_option_t power_option);

    protected:
        void handle_protocol_event(event_t events);
        void handle_FUSB302_event(FUSB302_event_t events);
        bool timer(void);
        void set_default_power(void);
        // Device
        FUSB302_dev_c FUSB302;
        PD_UFP_Protocol_c protocol;
        uint8_t int_pin;
        // Power ready power
        uint16_t ready_voltage; //mV
        uint16_t ready_current; //mA
        // PPS setup
        uint16_t PPS_voltage_next;
        uint8_t PPS_current_next;
        // Status
        virtual void status_power_ready(status_power_t status, uint16_t voltage, uint16_t current);
        uint8_t status_initialized;
        uint8_t status_src_cap_received;
        status_power_t status_power;
        // Timer and counter for PD Policy
        uint16_t time_polling;
        uint16_t time_wait_src_cap;
        uint16_t time_wait_ps_rdy;
        uint16_t time_PPS_request;
        uint8_t get_src_cap_retry_count;
        uint8_t wait_src_cap;
        uint8_t wait_ps_rdy;
        uint8_t send_request;
        // Status logging
        virtual void status_log_event(uint8_t status, uint32_t * obj = 0) {}
};


///////////////////////////////////////////////////////////////////////////////////////////////////
// Optional: PD_UFP_Log_c, extended from PD_UFP_c to provide logging function.
//           Asynchronous, minimal impact on PD timing.
///////////////////////////////////////////////////////////////////////////////////////////////////
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

