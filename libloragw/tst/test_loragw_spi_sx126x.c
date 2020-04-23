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
static RadioEvents_t RadioEvents;

#define RF_FREQUENCY                                926600000 // Hz

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false     

#define FSK_DATARATE                                50000     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false


/*!
 * Radio hardware and global parameters
 */
SX126x_t SX126x;

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);
static void usage(void);
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
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
    //init sx1261
    uint8_t buf;
    uint8_t stat = 0;
    stat = SX126xReadCommand( RADIO_GET_STATUS, &buf, 0 );
    printf("SX1261: SX1261_status: 0x%02X\n",stat);
    SX126xSetStandby( STDBY_RC );

    SX126xSetDio2AsRfSwitchCtrl( true );
    RadioSetModem( MODEM_LORA );
    //LoRa Syncword
    SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
    SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

    SX126xSetRegulatorMode( USE_DCDC );
    SX126xSetStandby( STDBY_RC );
    //
    SX126xClearIrqStatus( IRQ_RADIO_ALL );
    RadioSetModem( MODEM_LORA );
    SX126xSetModulationParams( &SX126x.ModulationParams );
    int set_freq=868000000;
    RadioSetChannel(set_freq);
    printf("SX1261: Frequency=%d\n",set_freq); 
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    SX126xSetSyncWord( ( uint8_t[] ){ 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0 } );
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX126xSetDioIrqParams( 0x0242, 0x0242, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX126xWriteRegister( REG_RX_GAIN, 0X96 );
    SX126xSetRx( 0xFFFFFF ); // Rx Continuous
    while (1){
    int rssi = SX126xGetRssiInst( );
    //rssi = SX126xGetRssiInst();
    printf("SX1261: RSSI=%d\n",rssi);  
    sleep(2);      
    }





    WaitOnBusy();
   
    //NRESET->0
    system("echo 0 > /sys/class/gpio/gpio22/value");
    printf("End of test for loragw_spi_sx1260.c\n");
    lgw_disconnect();


    return 0;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    //static uint8_t ledState = 1;
    // Toggle LED 1
    //ledState ^= 1;
    //GpioWrite( &Led1, ledState );
}

/* --- EOF ------------------------------------------------------------------ */
