/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1250 radios.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


#ifndef _LORAGW_SX1260_H
#define _LORAGW_SX1260_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

#include "config.h"     /* library configuration options (dynamically generated) */
#include "radio.h"
#include "sx126x.h"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

#define SX1260_FREQ_TO_REG(f) (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum {
    RADIO_GET_STATUS                        = 0xC0,
    RADIO_WRITE_REGISTER                    = 0x0D,
    RADIO_READ_REGISTER                     = 0x1D,
    RADIO_WRITE_BUFFER                      = 0x0E,
    RADIO_READ_BUFFER                       = 0x1E,
    RADIO_SET_SLEEP                         = 0x84,
    RADIO_SET_STANDBY                       = 0x80,
    RADIO_SET_FS                            = 0xC1,
    RADIO_SET_TX                            = 0x83,
    RADIO_SET_RX                            = 0x82,
    RADIO_SET_RXDUTYCYCLE                   = 0x94,
    RADIO_SET_CAD                           = 0xC5,
    RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
    RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    RADIO_SET_PACKETTYPE                    = 0x8A,
    RADIO_GET_PACKETTYPE                    = 0x11,
    RADIO_SET_RFFREQUENCY                   = 0x86,
    RADIO_SET_TXPARAMS                      = 0x8E,
    RADIO_SET_PACONFIG                      = 0x95,
    RADIO_SET_CADPARAMS                     = 0x88,
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    RADIO_SET_MODULATIONPARAMS              = 0x8B,
    RADIO_SET_PACKETPARAMS                  = 0x8C,
    RADIO_GET_RXBUFFERSTATUS                = 0x13,
    RADIO_GET_PACKETSTATUS                  = 0x14,
    RADIO_GET_RSSIINST                      = 0x15,
    RADIO_GET_STATS                         = 0x10,
    RADIO_RESET_STATS                       = 0x00,
    RADIO_CFG_DIOIRQ                        = 0x08,
    RADIO_GET_IRQSTATUS                     = 0x12,
    RADIO_CLR_IRQSTATUS                     = 0x02,
    RADIO_CALIBRATE                         = 0x89,
    RADIO_CALIBRATEIMAGE                    = 0x98,
    RADIO_SET_REGULATORMODE                 = 0x96,
    RADIO_GET_ERROR                         = 0x17,
    RADIO_CLR_ERROR                         = 0x07,
    RADIO_SET_TCXOMODE                      = 0x97,
    RADIO_SET_TXFALLBACKMODE                = 0x93,
    RADIO_SET_RFSWITCHMODE                  = 0x9D,
    RADIO_SET_STOPRXTIMERONPREAMBLE         = 0x9F,
    RADIO_SET_LORASYMBTIMEOUT               = 0xA0,
} RadioCommands_t;


typedef enum {
    STDBY_RC                = 0x00,
    STDBY_XOSC              = 0x01
} sx1260_standby_modes_t;

typedef enum {
    PACKET_TYPE_GFSK        = 0x00,
    PACKET_TYPE_LORA        = 0x01
} sx1260_packet_type_t;

typedef enum {
    SET_RAMP_10U            = 0x00,
    SET_RAMP_20U            = 0x01,
    SET_RAMP_40U            = 0x02,
    SET_RAMP_80U            = 0x03,
    SET_RAMP_200U           = 0x04,
    SET_RAMP_800U           = 0x05,
    SET_RAMP_1700U          = 0x06,
    SET_RAMP_3400U          = 0x07
} sx1260_ramp_time_t;

/*!
 * \brief Structure describing the radio status
 */
typedef union RadioStatus_u
{
    uint8_t Value;
    struct
    {   //bit order is lsb -> msb
        uint8_t           : 1;  //!< Reserved
        uint8_t CmdStatus : 3;  //!< Command status
        uint8_t ChipMode  : 3;  //!< Chip mode
        uint8_t           : 1;  //!< Reserved
    }Fields;
}RadioStatus_t;

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int SX126xWriteCommand(RadioCommands_t op_code, uint8_t *data, uint16_t size);
int SX126xReadCommand( RadioCommands_t op_code, uint8_t *data, uint16_t size);

/*!
 * \brief Returns the instantaneous RSSI value for the last packet received
 *
 * \retval      rssiInst      Instantaneous RSSI
 */
int8_t SX126xGetRssiInst( void );


#endif

/* --- EOF ------------------------------------------------------------------ */
