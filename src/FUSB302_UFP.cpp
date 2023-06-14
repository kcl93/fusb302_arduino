
/**
 * FUSB302_UFP.cpp
 *
 *      Author: Ryan Ma
 *      Edited: Kai Liebich
 *
 * Minimalist USB PD implement with only UFP(device) functionality
 * Requires only stdint.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * FUSB302 can support PD3.0 with limitations and workarounds
 * - Do not have enough FIFO for unchunked message, use chunked message instead
 * - VBUS sense low threshold at 4V, disable vbus_sense if request PPS below 4V
 * 
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "FUSB302_UFP.h"

/* Switches0 : 02h */
#define PU_EN2          (0x01 << 7)
#define PU_EN1          (0x01 << 6)
#define VCONN_CC2       (0x01 << 5)
#define VCONN_CC1       (0x01 << 4)
#define MEAS_CC2        (0x01 << 3)
#define MEAS_CC1        (0x01 << 2)
#define PDWN2           (0x01 << 1)
#define PDWN1           (0x01 << 0)

/* Switches1 : 03h */
#define POWERROLE       (0x01 << 7)
#define SPECREV1        (0x01 << 6)
#define SPECREV0        (0x01 << 5)
#define DATAROLE        (0x01 << 4)
#define AUTO_CRC        (0x01 << 2)
#define TXCC2           (0x01 << 1)
#define TXCC1           (0x01 << 0)

/* Measure : 04h */
#define MEAS_VBUS       (0x01 << 6)

/* Control0 : 06h */
#define TX_FLUSH        (0x01 << 6)
#define INT_MASK        (0x01 << 5)
#define HOST_CUR_MASK   (0x03 << 2)
#define HOST_CUR_3A0    (0x03 << 2)
#define HOST_CUR_1A5    (0x02 << 2)
#define HOST_CUR_USB    (0x01 << 2)
#define AUTO_PRE        (0x01 << 1)
#define TX_START        (0x01 << 0)

/* Control1 : 07h */
#define ENSOP2DB        (0x01 << 6)
#define ENSOP1DB        (0x01 << 5)
#define BIST_MODE2      (0x01 << 4)
#define RX_FLUSH        (0x01 << 2)
#define ENSOP2          (0x01 << 1)
#define ENSOP1          (0x01 << 0)

/* Control2 : 08h */
#define WAKE_EN         (0x01 << 3)
#define MODE_MASK       (0x03 << 1)
#define MODE_DFP        (0x03 << 1)
#define MODE_UFP        (0x02 << 1)
#define MODE_DRP        (0x01 << 1)
#define TOGGLE          (0x01 << 0)

/* Control3 : 09h */
#define SEND_HARDRESET  (0x01 << 6)
#define BIST_TMODE      (0x01 << 5)     /* on FUSB302B only */
#define AUTO_HARDRESET  (0x01 << 4)
#define AUTO_SOFTRESET  (0x01 << 3)
#define N_RETRIES_MASK  (0x03 << 1)
#define N_RETRIES(n)    ((n)  << 1)
#define AUTO_RETRY      (0x01 << 0)

/* Mask : 0Ah */
#define M_VBUSOK        (0x01 << 7)
#define M_ACTIVITY      (0x01 << 6)
#define M_COMP_CHNG     (0x01 << 5)
#define M_CRC_CHK       (0x01 << 4)
#define M_ALERT         (0x01 << 3)
#define M_WAKE          (0x01 << 2)
#define M_COLLISION     (0x01 << 1)
#define M_BC_LVL        (0x01 << 0)

/* Power : 0Bh */
#define PWR_INT_OSC     (0x01 << 3) 	/* Enable internal oscillator */
#define PWR_MEASURE     (0x01 << 2) 	/* Measure block powered */
#define PWR_RECEIVER    (0x01 << 1) 	/* Receiver powered and current reference for Measure block */
#define PWR_BANDGAP     (0x01 << 0) 	/* Bandgap and wake circuitry */

/* Reset : 0Ch */
#define PD_RESET        (0x01 << 1)
#define SW_RES          (0x01 << 0)

/* Maska : 0Eh */
#define M_OCP_TEMP      (0x01 << 7)
#define M_TOGDONE       (0x01 << 6)
#define M_SOFTFAIL      (0x01 << 5)
#define M_RETRYFAIL     (0x01 << 4)
#define M_HARDSENT      (0x01 << 3)
#define M_TXSENT        (0x01 << 2)
#define M_SOFTRST       (0x01 << 1)
#define M_HARDRST       (0x01 << 0)

/* Maskb : 0Fh */
#define M_GCRCSENT      (0x01 << 0)

/* Status0a : 3Ch */
#define SOFTFAIL        (0x01 << 5)
#define RETRYFAIL       (0x01 << 4)
#define POWER3_2        (0x01 << 2)
#define SOFTRST         (0x01 << 1)
#define HARDRST         (0x01 << 0)

/* Status1a : 3Dh */
#define TOGSS_MASK      (0x07 << 3)
#define TOGSS_RUNNING   (0x00 << 3)
#define TOGSS_SRC1      (0x01 << 3)
#define TOGSS_SRC2      (0x02 << 3)
#define TOGSS_SNK1      (0x05 << 3)
#define TOGSS_SNK2      (0x06 << 3)
#define TOGSS_AUDIOA    (0x07 << 3)
#define RXSOP2DB        (0x01 << 2)
#define RXSOP1DB        (0x01 << 1)
#define RXSOP           (0x01 << 0)

/* Interrupta : 3Eh */
#define I_OCP_TEMP      (0x01 << 7)
#define I_TOGDONE       (0x01 << 6)
#define I_SOFTFAIL      (0x01 << 5)
#define I_RETRYFAIL     (0x01 << 4)
#define I_HARDSENT      (0x01 << 3)
#define I_TXSENT        (0x01 << 2)
#define I_SOFTRST       (0x01 << 1)
#define I_HARDRST       (0x01 << 0)

/* Interruptb : 3Fh */
#define I_GCRCSENT      (0x01 << 0)

/* Status0 : 40h */
#define VBUSOK          (0x01 << 7)
#define ACTIVITY        (0x01 << 6)
#define COMP            (0x01 << 5)
#define CRC_CHK         (0x01 << 4)
#define ALERT           (0x01 << 3)
#define WAKE            (0x01 << 2)
#define BC_LVL_MASK     (0x03 << 0)
#define BC_LVL_LT200    (0x00 << 0)
#define BC_LVL_200_660  (0x01 << 0)
#define BC_LVL_660_1230 (0x02 << 0)
#define BC_LVL_GT1230   (0x03 << 0)

/* Status1 : 41h */
#define RXSOP2          (0x01 << 7)
#define RXSOP1          (0x01 << 6)
#define RX_EMPTY        (0x01 << 5)
#define RX_FULL         (0x01 << 4)
#define TX_EMPTY        (0x01 << 3)
#define TX_FULL         (0x01 << 2)
#define OVRTEMP         (0x01 << 1)
#define OCP             (0x01 << 0)

/* Interrupt : 42h */
#define I_VBUSOK        (0x01 << 7)
#define I_ACTIVITY      (0x01 << 6)
#define I_COMP_CHNG     (0x01 << 5)
#define I_CRC_CHK       (0x01 << 4)
#define I_ALERT         (0x01 << 3)
#define I_WAKE          (0x01 << 2)
#define I_COLLISION     (0x01 << 1)
#define I_BC_LVL        (0x01 << 0)

#define ADDRESS_DEVICE_ID   0x01
#define ADDRESS_SWITCHES0   0x02
#define ADDRESS_SWITCHES1   0x03
#define ADDRESS_MEASURE     0x04
#define ADDRESS_SLICE       0x05
#define ADDRESS_CONTROL0    0x06
#define ADDRESS_CONTROL1    0x07
#define ADDRESS_CONTROL2    0x08
#define ADDRESS_CONTROL3    0x09
#define ADDRESS_MASK        0x0A
#define ADDRESS_POWER       0x0B
#define ADDRESS_RESET       0x0C
#define ADDRESS_MASKA       0x0E
#define ADDRESS_MASKB       0x0F
#define ADDRESS_STATUS0A    0x3C
#define ADDRESS_STATUS1A    0x3D
#define ADDRESS_INTERRUPTA  0x3E
#define ADDRESS_INTERRUPTB  0x3F
#define ADDRESS_STATUS0     0x40
#define ADDRESS_STATUS1     0x41
#define ADDRESS_INTERRUPT   0x42
#define ADDRESS_FIFOS       0x43

#define REG_DEVICE_ID       this->reg_control[ADDRESS_DEVICE_ID  - ADDRESS_DEVICE_ID]
#define REG_SWITCHES0       this->reg_control[ADDRESS_SWITCHES0  - ADDRESS_DEVICE_ID]
#define REG_SWITCHES1       this->reg_control[ADDRESS_SWITCHES1  - ADDRESS_DEVICE_ID]
#define REG_MEASURE         this->reg_control[ADDRESS_MEASURE    - ADDRESS_DEVICE_ID]
#define REG_SLICE           this->reg_control[ADDRESS_SLICE      - ADDRESS_DEVICE_ID]
#define REG_CONTROL0        this->reg_control[ADDRESS_CONTROL0   - ADDRESS_DEVICE_ID]
#define REG_CONTROL1        this->reg_control[ADDRESS_CONTROL1   - ADDRESS_DEVICE_ID]
#define REG_CONTROL2        this->reg_control[ADDRESS_CONTROL2   - ADDRESS_DEVICE_ID]
#define REG_CONTROL3        this->reg_control[ADDRESS_CONTROL3   - ADDRESS_DEVICE_ID]
#define REG_MASK            this->reg_control[ADDRESS_MASK       - ADDRESS_DEVICE_ID]
#define REG_POWER           this->reg_control[ADDRESS_POWER      - ADDRESS_DEVICE_ID]
#define REG_RESET           this->reg_control[ADDRESS_RESET      - ADDRESS_DEVICE_ID]
#define REG_MASKA           this->reg_control[ADDRESS_MASKA      - ADDRESS_DEVICE_ID]
#define REG_MASKB           this->reg_control[ADDRESS_MASKB      - ADDRESS_DEVICE_ID]
#define REG_STATUS0A        this->reg_status[ADDRESS_STATUS0A    - ADDRESS_STATUS0A]
#define REG_STATUS1A        this->reg_status[ADDRESS_STATUS1A    - ADDRESS_STATUS0A]
#define REG_INTERRUPTA      this->reg_status[ADDRESS_INTERRUPTA  - ADDRESS_STATUS0A]
#define REG_INTERRUPTB      this->reg_status[ADDRESS_INTERRUPTB  - ADDRESS_STATUS0A]
#define REG_STATUS0         this->reg_status[ADDRESS_STATUS0     - ADDRESS_STATUS0A]
#define REG_STATUS1         this->reg_status[ADDRESS_STATUS1     - ADDRESS_STATUS0A]
#define REG_INTERRUPT       this->reg_status[ADDRESS_INTERRUPT   - ADDRESS_STATUS0A]

enum FUSB302_transmit_data_tokens_t {
    TX_TOKEN_TXON    = 0xA1,
    TX_TOKEN_SOP1    = 0x12,
    TX_TOKEN_SOP2    = 0x13,
    TX_TOKEN_SOP3    = 0x1B,
    TX_TOKEN_RESET1  = 0x15,
    TX_TOKEN_RESET2  = 0x16,
    TX_TOKEN_PACKSYM = 0x80,
    TX_TOKEN_JAM_CRC = 0xFF,
    TX_TOKEN_EOP     = 0x14,
    TX_TOKEN_TXOFF   = 0xFE,
};

enum FUSB302_state_t {
    FUSB302_STATE_UNATTACHED = 0,
    FUSB302_STATE_ATTACHED
};

#define FUSB302_ERR_MSG(s)  s

#define REG_READ(addr, data, count) do { \
    if (reg_read(addr, data, count) != FUSB302_SUCCESS) { return FUSB302_ERR_READ_DEVICE; } \
} while(0)

#define REG_WRITE(addr, data, count) do { \
    if (reg_write(addr, data, count) != FUSB302_SUCCESS) { return FUSB302_ERR_WRITE_DEVICE; } \
} while(0)

FUSB302_dev_c::FUSB302_dev_c(TwoWire &twoWire):
    i2c_address(0x22),
    twoWire(twoWire)
{

}

FUSB302_ret_t FUSB302_dev_c::i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    this->twoWire.beginTransmission(dev_addr);
    this->twoWire.write(reg_addr);
    this->twoWire.endTransmission();
    if (count > 0)
    {
        this->twoWire.clearWireTimeoutFlag();
        this->twoWire.requestFrom(dev_addr, count);
        while ((this->twoWire.getWireTimeoutFlag() == false) && (this->twoWire.available() == true) && (count > 0))
        {
            *data++ = this->twoWire.read();
            count--;
        }
    }
    return count == 0 ? FUSB302_SUCCESS : FUSB302_ERR_READ_DEVICE;
}

FUSB302_ret_t FUSB302_dev_c::i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    this->twoWire.beginTransmission(dev_addr);
    this->twoWire.write(reg_addr);
    while (count > 0)
    {
        this->twoWire.write(*data++);
        count--;
    }
    uint8_t ret = this->twoWire.endTransmission();
    return (ret == 0) ? FUSB302_SUCCESS : FUSB302_ERR_WRITE_DEVICE;
}

FUSB302_ret_t FUSB302_dev_c::reg_read(uint8_t address, uint8_t *data, uint8_t count)
{
    FUSB302_ret_t ret = this->i2c_read(this->i2c_address, address, data, count);
    if (ret != FUSB302_SUCCESS)
    {
        this->err_msg = FUSB302_ERR_MSG("Fail to read register");
    }
    return ret;
}

FUSB302_ret_t FUSB302_dev_c::reg_write(uint8_t address, uint8_t *data, uint8_t count)
{
    FUSB302_ret_t ret = this->i2c_write(this->i2c_address, address, data, count);
    if (ret != FUSB302_SUCCESS)
    {
        this->err_msg = FUSB302_ERR_MSG("Fail to write register");
    }
    return ret;
}

FUSB302_ret_t FUSB302_dev_c::read_cc_lvl(uint8_t * cc_value)
{
    /*  00: < 200 mV          : vRa
        01: >200 mV, <660 mV  : vRd-USB
        10: >660 mV, <1.23 V  : vRd-1.5
        11: >1.23 V           : vRd-3.0  */
    uint8_t cc, cc_verify;
    REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
    cc = REG_STATUS0 & BC_LVL_MASK;
    for (uint8_t i = 0; i < 5; i++) {
        REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
        cc_verify = REG_STATUS0 & BC_LVL_MASK;
        if (cc != cc_verify) {
            return FUSB302_BUSY;
        }
    }
    *cc_value = cc;
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::read_incoming_packet(FUSB302_event_t * events)
{
    uint8_t len, b[3];
    REG_READ(ADDRESS_FIFOS, b, 3);
    this->rx_header = ((uint16_t)b[2] << 8) | b[1];
    len = (this->rx_header >> 12) & 0x7;
    REG_READ(ADDRESS_FIFOS, this->rx_buffer, len * 4 + 4);  /* add 4 to len to read CRC out */

    if (events)
    {
        *events |= FUSB302_EVENT_RX_SOP;
    }
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::state_unattached(FUSB302_event_t * events)
{
    REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
    if (REG_STATUS0 & VBUSOK)
    {
        /* enable internal oscillator */
        REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE | PWR_INT_OSC;
        REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);
        delay(1);

        /* read cc1 */
        REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC1;
        REG_SWITCHES1 = SPECREV0;
        REG_MEASURE = 49;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);
        delay(1);
        while (this->read_cc_lvl(&this->cc1) != FUSB302_SUCCESS) {
            delay(1);
        }

        /* read cc2 */
        REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC2;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 1);
        delay(1);
        while (this->read_cc_lvl(&this->cc2) != FUSB302_SUCCESS) {
            delay(1);
        }

        /* clear interrupt */
        REG_READ(ADDRESS_INTERRUPTA, &REG_INTERRUPTA, 2);
        this->interrupta = 0;
        this->interruptb = 0;        

        /* enable tx on cc pin */
        if (this->cc1 > 0) {
            REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC1;
            REG_SWITCHES1 = SPECREV0 | AUTO_CRC | TXCC1;
            //REG_SWITCHES1 = SPECREV0 | TXCC1;
        } else if (this->cc2 > 0) {
            REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC2;
            REG_SWITCHES1 = SPECREV0 | AUTO_CRC | TXCC2;
            //REG_SWITCHES1 = SPECREV0 | TXCC2;
        } else {
            REG_SWITCHES0 = PDWN1 | PDWN2;
            REG_SWITCHES1 = SPECREV0;
        }
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 2);

        /* update state */
        this->state = FUSB302_STATE_ATTACHED;
        if (events) {
            *events |= FUSB302_EVENT_ATTACHED;
        }
    }
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::state_attached(FUSB302_event_t * events)
{
    REG_READ(ADDRESS_STATUS0A, &REG_STATUS0A, 7);
    this->interrupta |= REG_INTERRUPTA;
    this->interruptb |= REG_INTERRUPTB;    
    if (this->vbus_sense && ((REG_STATUS0 & VBUSOK) == 0))
    {
        /* reset cc pins to pull down */
        REG_SWITCHES0 = PDWN1 | PDWN2;
        REG_SWITCHES1 = SPECREV0;
        REG_MEASURE = 49;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);

        /* turn off internal oscillator */
        REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE;
        REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);

        /* update state */
        this->state = FUSB302_STATE_UNATTACHED;
        if (events)
        {
            *events |= FUSB302_EVENT_DETACHED;
        }
        return FUSB302_SUCCESS;
    }
    if (REG_STATUS0A & HARDRST)
    {
        uint8_t reg_control = PD_RESET;
        REG_WRITE(ADDRESS_RESET, &reg_control, 1);
        return FUSB302_SUCCESS;
    }
    if (this->interruptb & I_GCRCSENT)
    {
        this->interruptb &= ~I_GCRCSENT;
        if (events)
        {
            *events |= FUSB302_EVENT_GOOD_CRC_SENT;
        }
    }
    if ((REG_STATUS1 & RX_EMPTY) == 0)
    {
        if (this->read_incoming_packet(events) != FUSB302_SUCCESS)
        {
            uint8_t rx_flush = REG_CONTROL1 | RX_FLUSH;
            this->reg_write(ADDRESS_CONTROL1, &rx_flush, 1);
        }
    }
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::init()
{
    if (this->i2c_address == 0)
    {
        this->err_msg = FUSB302_ERR_MSG("Invalid i2c address");
        return FUSB302_ERR_PARAM;
    }

    if (this->reg_read(ADDRESS_DEVICE_ID, &this->reg_control[1], 1) != FUSB302_SUCCESS)
    {
        this->err_msg = FUSB302_ERR_MSG("Device not found");
        return FUSB302_ERR_READ_DEVICE;
    }

	if ((this->reg_control[1] & 0x80) == 0)
    {
        this->err_msg = FUSB302_ERR_MSG("Invalid device version");
        return FUSB302_ERR_DEVICE_ID;
    }

    this->state = FUSB302_STATE_UNATTACHED;
    this->rx_header = 0;
    memset(this->rx_buffer, 0, sizeof(this->rx_buffer));

    /* restore default settings */
    REG_RESET = SW_RES;
    REG_WRITE(ADDRESS_RESET, &REG_RESET, 1);
    
    /* fetch all R/W registers */
    REG_READ(ADDRESS_DEVICE_ID, &REG_DEVICE_ID, 15);

    /* configure switchs and comparators */
    REG_SWITCHES0 = PDWN1 | PDWN2;
    REG_SWITCHES1 = SPECREV0;
    REG_MEASURE = 49;
	REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);

    /* configure auto retries */
    REG_CONTROL3 &= ~N_RETRIES_MASK;
    REG_CONTROL3 |= N_RETRIES(3) | AUTO_RETRY;
    REG_WRITE(ADDRESS_CONTROL3, &REG_CONTROL3, 1);

    /* configure interrupt mask */
    REG_MASK = 0xFF;
    REG_MASK &= ~(M_VBUSOK | M_ACTIVITY | M_COLLISION | M_ALERT | M_CRC_CHK);
    REG_WRITE(ADDRESS_MASK, &REG_MASK, 1);
    
    /* configure interrupt maska/maskb */
    REG_MASKA = 0xFF;
    REG_MASKA &= ~(M_RETRYFAIL | M_HARDSENT | M_TXSENT | M_HARDRST);
    REG_WRITE(ADDRESS_MASKA, &REG_MASKA, 1);
    REG_MASKB = 0xFF;
    REG_MASKB &= ~(M_GCRCSENT);
    REG_WRITE(ADDRESS_MASKB, &REG_MASKB, 1);
    
    /* enable interrupt */
    REG_CONTROL0 &= ~INT_MASK;
    REG_WRITE(ADDRESS_CONTROL0, &REG_CONTROL0, 1);

    /* Power on, enable VUSB detection */
    REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE;
    REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);
    
    this->vbus_sense = 1;
    this->err_msg = FUSB302_ERR_MSG("");
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::pd_reset()
{
    uint8_t reg = PD_RESET;
    REG_WRITE(ADDRESS_RESET, &reg, 1);
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::pdwn_cc(uint8_t enable)
{
    REG_SWITCHES0 = enable ? (PDWN1 | PDWN2) : 0;
	REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 1);
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::set_vbus_sense(uint8_t enable)
{
    if (this->vbus_sense != enable)
    {
        if (enable)
        {
            REG_MASK &= ~M_VBUSOK;  /* enable VBUSOK interrupt */
        }
        else
        { 
            REG_MASK |= M_VBUSOK;   /* disable VBUSOK interrupt */
        }
        REG_WRITE(ADDRESS_MASK, &REG_MASK, 1);
        this->vbus_sense = enable;
    }
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::get_ID(uint8_t * version_ID, uint8_t * revision_ID)
{
    if (REG_DEVICE_ID & 0x80)
    {
        if (version_ID)
        {
            *version_ID = (REG_DEVICE_ID >> 4) & 0x7;
        }
        if (revision_ID)
        {
            *revision_ID = (REG_DEVICE_ID >> 0) & 0xF;
        }
        return FUSB302_SUCCESS;
    }
    return FUSB302_ERR_PARAM;
}

FUSB302_ret_t FUSB302_dev_c::get_cc(uint8_t *cc1, uint8_t *cc2)
{
    if (cc1)
    {
        *cc1 = this->cc1;
    }
    if (cc2)
    {
        *cc2 = this->cc2;
    }
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::get_vbus_level(uint8_t *vbus)
{
    uint8_t reg_control;
    REG_READ(ADDRESS_STATUS0, &reg_control, 1);
    *vbus = reg_control & VBUSOK ? 1 : 0;
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::get_message(uint16_t * header, uint32_t * data)
{
    if (header)
    {
        *header = this->rx_header;
    }
    if (data)
    {
        uint8_t len = (this->rx_header >> 12) & 0x7;
        memcpy(data, this->rx_buffer, len * 4);
    }
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::tx_sop(uint16_t header, const uint32_t *data)
{
    uint8_t buf[40];
    uint8_t * pbuf = buf;
    uint8_t obj_count = ((header >> 12) & 7);
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP2;
    *pbuf++ = (uint8_t)TX_TOKEN_PACKSYM | ((obj_count << 2) + 2);
    *pbuf++ = header & 0xFF; header >>= 8;
    *pbuf++ = header & 0xFF;
    for (uint8_t i = 0; i < obj_count; i++)
    {
        uint32_t d = *data++;
        *pbuf++ = d & 0xFF; d >>= 8;
        *pbuf++ = d & 0xFF; d >>= 8;
        *pbuf++ = d & 0xFF; d >>= 8;
        *pbuf++ = d & 0xFF;
    }
    *pbuf++ = (uint8_t)TX_TOKEN_JAM_CRC;
    *pbuf++ = (uint8_t)TX_TOKEN_EOP;
    *pbuf++ = (uint8_t)TX_TOKEN_TXOFF;
    *pbuf++ = (uint8_t)TX_TOKEN_TXON;
    REG_WRITE(ADDRESS_FIFOS, buf, pbuf - buf);
    delay(1);
	return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::tx_hard_reset()
{
    uint8_t reg_control = REG_CONTROL3;
    reg_control |= SEND_HARDRESET;
    REG_WRITE(ADDRESS_CONTROL3, &reg_control, 1);
    delay(5);
    reg_control = PD_RESET;
    REG_WRITE(ADDRESS_RESET, &reg_control, 1);
    return FUSB302_SUCCESS;
}

FUSB302_ret_t FUSB302_dev_c::alert(FUSB302_event_t * events)
{
    switch (this->state)
    {
        case 0:
            this->state_unattached(events);
            break;

        case 1:
            this->state_attached(events);
            break;
    }
    this->state = FUSB302_STATE_UNATTACHED;
    return FUSB302_SUCCESS;
}
