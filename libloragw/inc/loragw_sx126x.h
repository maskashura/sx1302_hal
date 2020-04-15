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
