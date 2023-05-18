
/**
 * PD_UFP_Protocol.h
 *
 *      Author: Ryan Ma
 *      Edited: Kai Liebich
 *
 * Minimalist USB PD implement with only UFP(device) sink only functionality
 * Requires PD PHY to do automatic GoodCRC response on valid SOP messages.
 * Requires only stdint.h, stdbool.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * Support PD3.0 PPS
 * Do not support extended message. Not necessary for PD trigger and PPS.
 * 
 * Reference: USB_PD_R2_0 V1.3 - 20170112
 *            USB_PD_R3_0 V2.0 20190829 + ECNs 2020-12-10
 *            - Chapter 6. Protocol Layer
 *
 */

#ifndef PD_UFP_PROTOCOL_H
#define PD_UFP_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define PD_PROTOCOL_MAX_NUM_OF_PDO      7

#define PD_PROTOCOL_EVENT_SRC_CAP       (1 << 0)
#define PD_PROTOCOL_EVENT_PS_RDY        (1 << 1)
#define PD_PROTOCOL_EVENT_ACCEPT        (1 << 2)
#define PD_PROTOCOL_EVENT_REJECT        (1 << 3)
#define PD_PROTOCOL_EVENT_PPS_STATUS    (1 << 4)

typedef uint8_t event_t;

/**
 * @brief PD power options for normal and PPS mode
 * 
 */
typedef enum : uint8_t
{
    PD_POWER_OPTION_MAX_5V      = 0,
    PD_POWER_OPTION_MAX_9V      = 1,
    PD_POWER_OPTION_MAX_12V     = 2,
    PD_POWER_OPTION_MAX_15V     = 3,
    PD_POWER_OPTION_MAX_20V     = 4,
    PD_POWER_OPTION_MAX_VOLTAGE = 5,
    PD_POWER_OPTION_MAX_CURRENT = 6,
    PD_POWER_OPTION_MAX_POWER   = 7,
} PD_power_option_t;

typedef enum : uint8_t
{   /* Power data object type */
    PD_PDO_TYPE_FIXED_SUPPLY    = 0,
    PD_PDO_TYPE_BATTERY         = 1,
    PD_PDO_TYPE_VARIABLE_SUPPLY = 2,
    PD_PDO_TYPE_AUGMENTED_PDO   = 3     /* USB PD 3.0 */
} PD_power_data_obj_type_t;

typedef enum : uint8_t
{
    PPS_PTF_NOT_SUPPORT         = 0,
    PPS_PTF_NORMAL              = 1,
    PPS_PTF_WARNING             = 2,
    PPS_PTF_OVER_TEMPERATURE    = 3
} PPS_PTF_t;

typedef enum : uint8_t
{
    PPS_OMF_VOLTAGE_MODE        = 0,
    PPS_OMF_CURRENT_LIMIT_MODE  = 1
} PPS_OMF_t;

typedef struct
{
    uint16_t output_voltage;    /* Voltage in 20mV units, 0xFFFF if not supported */
    uint8_t output_current;     /* Current in 50mV units, 0xFF if not supported */
    PPS_PTF_t flag_PTF;
    PPS_OMF_t flag_OMF;
} PPS_status_t;

typedef struct
{
    const char * name;
    uint8_t id;
    uint8_t spec_rev;
    uint8_t num_of_obj;
    uint8_t extended;
} PD_msg_info_t;

typedef struct
{
    PD_power_data_obj_type_t type;
    uint16_t min_v;     /* Voltage in 50mV units */
    uint16_t max_v;     /* Voltage in 50mV units */
    uint16_t max_i;     /* Current in 10mA units */
    uint16_t max_p;     /* Power in 250mW units */
} PD_power_info_t;

enum : uint8_t
{
    STATUS_POWER_NA = 0,
    STATUS_POWER_TYP,
    STATUS_POWER_PPS
};
typedef uint8_t status_power_t;

typedef struct PD_msg_header_info PD_msg_header_info_t;
typedef struct PD_power_option_setting PD_power_option_setting_t;
typedef struct PD_msg_state PD_msg_state_t;


///////////////////////////////////////////////////////////////////////////////////////////////////
// PD_UFP_Protocol_c
///////////////////////////////////////////////////////////////////////////////////////////////////
class PD_UFP_Protocol_c
{
    public:
        /**
         * @brief Constructor
         * 
         */
        PD_UFP_Protocol_c(void) { init(); }
        
        /* Message handler */
        void handle_msg(uint16_t header, uint32_t *obj, event_t *events);
        bool respond(uint16_t *h, uint32_t *obj);

        /* PD Message creation */
        void create_get_src_cap(uint16_t *header);
        void create_get_PPS_status(uint16_t *header);
        void create_request(uint16_t *header, uint32_t *obj);

        /* Get functions */
        uint8_t  get_selected_power() { return this->power_data_obj_selected; }

        /**
         * @brief Get the PPS voltage
         * 
         * @return Voltage in mV 
         */
        uint16_t get_PPS_voltage() { return this->PPS_voltage * 20; }

        /**
         * @brief Get the PPS current
         * 
         * @return Current in mA 
         */
        uint16_t  get_PPS_current() { return (uint16_t)this->PPS_current * 50; }

        uint16_t get_tx_msg_header() { return this->tx_msg_header; }
        uint16_t get_rx_msg_header() { return this->rx_msg_header; }

        bool get_msg_info(uint16_t header, PD_msg_info_t * msg_info);

        bool get_power_info(uint8_t index, PD_power_info_t *power_info);
        bool get_PPS_status(PPS_status_t * PPS_status);

        /* Set Fixed and Variable power option */
        bool set_power_option(PD_power_option_t option);
        bool select_power(uint8_t index);

        /**
         * @brief Set PPS voltage and current
         * 
         * @param PPS_voltage   Voltage in mV
         * @param PPS_current   Current in mA
         * @param strict        If true, nothing is being changed in case of not qualified settings,
         *                      else falls back to the regular power option
         * 
         * @return true         If PPS setting is qualified or strict is false
         * @return false        If PPS setting is not qualified and strict is true
         */
        bool set_PPS(uint16_t PPS_voltage, uint8_t PPS_current, bool strict);  

        void reset(void);
        void init(void);
        
    protected:
        const PD_msg_state_t *msg_state;
        uint16_t tx_msg_header;
        uint16_t rx_msg_header;
        uint8_t message_id;

        uint16_t PPS_voltage; // in 20mV steps
        uint16_t PPS_current; // in 50mA steps
        uint8_t PPSSDB[4];  /* PPS Status Data Block */

        PD_power_option_t power_option;
        uint32_t power_data_obj[PD_PROTOCOL_MAX_NUM_OF_PDO];
        uint8_t power_data_obj_count;
        uint8_t power_data_obj_selected;

        static const PD_msg_state_t data_msg_list[];
        static const PD_msg_state_t ctrl_msg_list[];
        static const PD_msg_state_t ext_msg_list[];
        static const PD_power_option_setting_t power_option_setting[8];

        void parse_header(PD_msg_header_info_t *info, uint16_t header);

        uint16_t generate_header(uint8_t type, uint8_t obj_count);
        uint16_t generate_header_ext(uint8_t type, uint8_t data_size, uint32_t * obj);

        uint8_t evaluate_src_cap(uint16_t PPS_voltage, uint8_t PPS_current);
        
        static void handler_good_crc   (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_goto_min   (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_accept     (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_reject     (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_ps_rdy     (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_source_cap (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_BIST       (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_alert      (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_vender_def (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);
        static void handler_PPS_Status (PD_UFP_Protocol_c *p, uint16_t header, uint32_t * obj, event_t * events);

        static bool responder_get_sink_cap  (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
        static bool responder_reject        (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
        static bool responder_soft_reset    (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
        static bool responder_source_cap    (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
        static bool responder_vender_def    (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
        static bool responder_sink_cap_ext  (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
        static bool responder_not_support   (PD_UFP_Protocol_c *p, uint16_t * header, uint32_t * obj);
};


#endif
