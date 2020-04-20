/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for the sx1250 module

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* Fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>     /* sigaction */
#include <unistd.h>     /* getopt, access */


#include "loragw_spi.h"
#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_sx126x.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           16

#define LINUXDEV_PATH_DEFAULT "/dev/spidev0.0"

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);
static void usage(void);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    uint8_t test_buff[BUFF_SIZE];
    uint8_t read_buff[BUFF_SIZE];
    uint32_t test_val, read_val;
    int cycle_number = 0;
    int i, x;

    /* SPI interfaces */
    const char spidev_path_default[] = LINUXDEV_PATH_DEFAULT;
    const char * spidev_path = spidev_path_default;

    printf("LoRa SX126X test!\n");

    /* Board reset */
    //if (system("./reset_lgw.sh start") != 0) {
    //    printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
    //    exit(EXIT_FAILURE);
    //}

    x = lgw_connect(spidev_path);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the concentrator using SPI %s\n", spidev_path);
        return EXIT_FAILURE;
    }

    /* Reset radios */
    if(access("/sys/class/gpio/gpio22",0) < 0){
        printf("NRESET (GPIO22) is not exist!\n");  
        system("echo 22 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio22/direction");
    }
    //NRESET->1
    system("echo 1 > /sys/class/gpio/gpio22/value");
    //SX126xWriteCommand(RADIO_SET_STANDBY, 0x00, 1);
    SX126xSetStandby( STDBY_RC );
    WaitOnBusy();
    printf("WaitOnBusy\n");
    //syncword
    uint8_t buff;
    SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
    SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
    printf("SX1261: syncword: 0x%02X\n",SX126xReadRegister(0x0740));
    printf("SX1261: syncword: 0x%02X\n",SX126xReadRegister(0x0741));
    //
    printf("SX1261: TXMod: 0x%02X\n",SX126xReadRegister(0x0889));
    printf("SX1261: RXgain: 0x%02X\n",SX126xReadRegister(0x08ac));
    SX126xWriteRegister( REG_RX_GAIN, 0X96 );
    printf("SX1261: RXgain: 0x%02X\n",SX126xReadRegister(0x08ac));
    //
    printf("SX1261: FSK Syncword: 0x%02X\n",SX126xReadRegister(0x06c0));
    SX126xWriteRegister( 0x06c0, 0X55 );
    printf("SX1261: FSK Syncword: 0x%02X\n",SX126xReadRegister(0x06c0));

    /* Read status from SX1261 */
    //RadioStatus_t status_1261=SX126xGetStatus();
    uint8_t buf;
    uint8_t stat = 0;
    stat = SX126xReadCommand( RADIO_GET_STATUS, &buf, 0 );
    printf("SX1261: SX1261_status: 0x%02X\n",stat);
    
    int8_t rssi = 0;    
    SX126xReadCommand( RADIO_GET_RSSIINST, &buf, 1 );
    rssi = -buf >> 1;
    printf("SX1261: RSSI: %d\n", rssi);
    //
    //RadioIsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
    if(RadioIsChannelFree(MODEM_FSK,926600000,0,0)){
        printf("Channel is free\n");
    } else {
        printf("Channel is not free\n");
    }
    lgw_disconnect();
    //NRESET->0
    system("echo 0 > /sys/class/gpio/gpio22/value");
    printf("End of test for loragw_spi_sx1260.c\n");

    /* Board reset */
//    if (system("./reset_lgw.sh stop") != 0) {
//        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
//        exit(EXIT_FAILURE);
//    }

    return 0;
}



/* --- EOF ------------------------------------------------------------------ */
