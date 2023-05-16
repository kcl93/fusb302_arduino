
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
 
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "PD_UFP.h"

#define t_PD_POLLING            100
#define t_TypeCSinkWaitCap      350
#define t_RequestToPSReady      580     // combine t_SenderResponse and t_PSTransition
#define t_PPSRequest            5000    // must less than 10000 (10s)

#define PIN_FUSB302_INT         12

enum {
    STATUS_LOG_MSG_TX,
    STATUS_LOG_MSG_RX,
    STATUS_LOG_DEV,
    STATUS_LOG_CC,
    STATUS_LOG_SRC_CAP,
    STATUS_LOG_POWER_READY,
    STATUS_LOG_POWER_PPS_STARTUP,
    STATUS_LOG_POWER_REJECT,
    STATUS_LOG_LOAD_SW_ON,
    STATUS_LOG_LOAD_SW_OFF,
};


///////////////////////////////////////////////////////////////////////////////////////////////////
// PD_UFP_c
///////////////////////////////////////////////////////////////////////////////////////////////////
PD_UFP_c::PD_UFP_c(TwoWire &twoWire):
    FUSB302(twoWire),
    ready_voltage(0),
    ready_current(0),
    PPS_voltage_next(0),
    PPS_current_next(0),
    status_initialized(0),
    status_src_cap_received(0),
    status_power(STATUS_POWER_NA),
    time_polling(0),
    time_wait_src_cap(0),
    time_wait_ps_rdy(0),
    time_PPS_request(0),
    get_src_cap_retry_count(0),
    wait_src_cap(0),
    wait_ps_rdy(0),
    send_request(0)
{

}

void PD_UFP_c::init(uint8_t int_pin, PD_power_option_t power_option)
{
    init_PPS(int_pin, 0, 0, power_option);
}

void PD_UFP_c::init_PPS(uint8_t int_pin, uint16_t PPS_voltage, uint8_t PPS_current, PD_power_option_t power_option)
{
    this->int_pin = int_pin;
    // Initialize FUSB302
    pinMode(int_pin, INPUT_PULLUP); // Set FUSB302 int pin input ant pull up
    if ((this->FUSB302.init() == FUSB302_SUCCESS) && (this->FUSB302.get_ID(0, 0) == FUSB302_SUCCESS))
    {
        this->status_initialized = 1;
    }

    // Two stage startup for PPS Voltge < 5V
    if ((PPS_voltage > 0) && (PPS_voltage < 5000))
    {
        this->PPS_voltage_next = PPS_voltage;
        this->PPS_current_next = PPS_current;
        PPS_voltage = 5000;
    }

    // Initialize PD protocol engine
    this->protocol.init();
    this->protocol.set_power_option(power_option);
    this->protocol.set_PPS(PPS_voltage, PPS_current, false);

    this->status_log_event(STATUS_LOG_DEV);
}

void PD_UFP_c::handle(void)
{
    if (timer() || (digitalRead(int_pin) == 0))
    {
        FUSB302_event_t FUSB302_events = 0;
        for (uint8_t i = 0; (i < 3) && (this->FUSB302.alert(&FUSB302_events) != FUSB302_SUCCESS); i++) {}
        if (FUSB302_events)
        {
            handle_FUSB302_event(FUSB302_events);
        }
    }
}

bool PD_UFP_c::set_PPS(uint16_t PPS_voltage, uint8_t PPS_current)
{
    if ((this->status_power == STATUS_POWER_PPS) && (this->protocol.set_PPS(PPS_voltage, PPS_current, true)))
    {
        this->send_request = 1;
        return true;
    }
    return false;
}

void PD_UFP_c::set_power_option(PD_power_option_t power_option)
{
    if (this->protocol.set_power_option(power_option))
    {
        this->send_request = 1;
    }
}

void PD_UFP_c::handle_protocol_event(event_t events)
{    
    if (events & PD_PROTOCOL_EVENT_SRC_CAP)
    {
        this->wait_src_cap = 0;
        this->get_src_cap_retry_count = 0;
        this->wait_ps_rdy = 1;
        this->time_wait_ps_rdy = millis();
        this->status_log_event(STATUS_LOG_SRC_CAP);
    }
    if (events & PD_PROTOCOL_EVENT_REJECT)
    {
        if (this->wait_ps_rdy)
        {
            this->wait_ps_rdy = 0;
            this->status_log_event(STATUS_LOG_POWER_REJECT);
        }
    }    
    if (events & PD_PROTOCOL_EVENT_PS_RDY)
    {
        PD_power_info_t p;
        uint8_t selected_power = this->protocol.get_selected_power();
        this->protocol.get_power_info(selected_power, &p);
        this->wait_ps_rdy = 0;
        if (p.type == PD_PDO_TYPE_AUGMENTED_PDO)
        {
            // PPS mode
            this->FUSB302.set_vbus_sense(0);
            if (this->PPS_voltage_next)
            {
                // Two stage startup for PPS voltage < 5V
                this->protocol.set_PPS(this->PPS_voltage_next, this->PPS_current_next, false);
                this->PPS_voltage_next = 0;
                this->send_request = 1;
                this->status_log_event(STATUS_LOG_POWER_PPS_STARTUP);
            }
            else
            {
                time_PPS_request = millis();
                this->status_power_ready(STATUS_POWER_PPS, 
                this->protocol.get_PPS_voltage(), this->protocol.get_PPS_current());
                this->status_log_event(STATUS_LOG_POWER_READY);
            }
        }
        else
        {
            this->FUSB302.set_vbus_sense(1);
            this->status_power_ready(STATUS_POWER_TYP, p.max_v, p.max_i);
            this->status_log_event(STATUS_LOG_POWER_READY);
        }
    }
}

void PD_UFP_c::handle_FUSB302_event(FUSB302_event_t events)
{
    if (events & FUSB302_EVENT_DETACHED)
    {
        this->protocol.reset();
        return;
    }
    if (events & FUSB302_EVENT_ATTACHED)
    {
        uint8_t cc1 = 0, cc2 = 0, cc = 0;
        this->FUSB302.get_cc(&cc1, &cc2);
        this->protocol.reset();
        if (cc1 && cc2 == 0)
        {
            cc = cc1;
        }
        else if (cc2 && cc1 == 0)
        {
            cc = cc2;
        }
        /* TODO: handle no cc detected error */
        if (cc > 1)
        {
            this->wait_src_cap = 1;
        }
        else
        {
            this->set_default_power();
        }
        this->status_log_event(STATUS_LOG_CC);
    }
    if (events & FUSB302_EVENT_RX_SOP)
    {
        event_t protocol_event = 0;
        uint16_t header;
        uint32_t obj[7];
        this->FUSB302.get_message(&header, obj);
        this->protocol.handle_msg(header, obj, &protocol_event);
        this->status_log_event(STATUS_LOG_MSG_RX, obj);
        if (protocol_event)
        {
            this->handle_protocol_event(protocol_event);
        }
    }
    if (events & FUSB302_EVENT_GOOD_CRC_SENT)
    {
        uint16_t header;
        uint32_t obj[7];
        delay(2);  /* Delay respond in case there are retry messages */
        if (this->protocol.respond(&header, obj)) {
            this->status_log_event(STATUS_LOG_MSG_TX, obj);
            this->FUSB302.tx_sop(header, obj);
        }
    }
}

bool PD_UFP_c::timer(void)
{
    uint16_t t = millis();
    if (this->wait_src_cap && ((t - this->time_wait_src_cap) > t_TypeCSinkWaitCap))
    {
        this->time_wait_src_cap = t;
        if (this->get_src_cap_retry_count < 3) {
            uint16_t header;
            this->get_src_cap_retry_count += 1;
            /* Try to request soruce capabilities message (will not cause power cycle VBUS) */
            this->protocol.create_get_src_cap(&header);
            this->status_log_event(STATUS_LOG_MSG_TX);
            this->FUSB302.tx_sop(header, 0);
        }
        else
        {
            this->get_src_cap_retry_count = 0;
            /* Hard reset will cause the source power cycle VBUS. */
            this->FUSB302.tx_hard_reset();
            this->protocol.reset();
        }
    }
    if (this->wait_ps_rdy)
    {
        if ((t - this->time_wait_ps_rdy) > t_RequestToPSReady)
        {
           this-> wait_ps_rdy = 0;
            this->set_default_power();
        }
    }
    else if (this->send_request || ((this->status_power == STATUS_POWER_PPS) && ((t - this->time_PPS_request) > t_PPSRequest)))
    {
        this->wait_ps_rdy = 1;
        this->send_request = 0;
        this->time_PPS_request = t;
        uint16_t header;
        uint32_t obj[7];
        /* Send request if option updated or regularly in PPS mode to keep power alive */
        this->protocol.create_request(&header, obj);
        this->status_log_event(STATUS_LOG_MSG_TX, obj);
        this->time_wait_ps_rdy = millis();
        this->FUSB302.tx_sop(header, obj);
    }
    if ((t - this->time_polling) > t_PD_POLLING)
    {
        this->time_polling = t;
        return true;
    }
    return false;
}

void PD_UFP_c::set_default_power(void)
{
    this->status_power_ready(STATUS_POWER_TYP, 5000, 1000);
    this->status_log_event(STATUS_LOG_POWER_READY);
}

void PD_UFP_c::status_power_ready(status_power_t status, uint16_t voltage, uint16_t current)
{
    this->ready_voltage = voltage;
    this->ready_current = current;
    this->status_power = status;
}
