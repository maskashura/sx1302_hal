/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX120 radios.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_spi.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx126x.h" //Public functions prototypes
#include "radio.h"
#include "sx126x.h" //Register of 1261


/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_RAD == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif

#define SX1260_FREQ_TO_REG(f)       (uint32_t)((uint64_t)f * (1 << 25) / 32000000U)

/*
 * Private global variables
 */


/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
}RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;

/*!
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool ImageCalibrated = false;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define WAIT_BUSY_SX1260_MS  1

/* -------------------------------------------------------------------------- */
/* --- INTERNAL SHARED VARIABLES -------------------------------------------- */

extern void *lgw_spi_target; /*! generic pointer to the SPI device */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int SX126xWriteCommand( RadioCommands_t op_code, uint8_t *data, uint16_t size) {
    int spi_device;
    int cmd_size = 1; /* op_code */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a, i;

    /* wait BUSY */
    wait_ms(WAIT_BUSY_SX1260_MS);

    /* check input variables */
    CHECK_NULL(lgw_spi_target);

    spi_device = *(int *)lgw_spi_target; /* must check that spi_target is not null beforehand */

    if(access("/sys/class/gpio/gpio7",0) < 0){
        printf("NSS (GPIO7) is not exist!\n");  
        system("echo 7 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio7/direction");
    }
    //NSS->1
    system("echo 1 > /sys/class/gpio/gpio7/value");
    out_buf[0] = (uint8_t)op_code; //0x0d write command

    for(i = 0; i < (int)size; i++) {
        out_buf[cmd_size + i] = data[i];
    }
    command_size = cmd_size + size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);
    //NSS->1
    system("echo 1 > /sys/class/gpio/gpio7/value");
    //WaitOnBusy
    /*
    if( op_code != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
    */
    /* determine return code */
    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI write success\n");
        return LGW_SPI_SUCCESS;
    }
}


int SX126xReadCommand( RadioCommands_t op_code, uint8_t *data, uint16_t size) {
    int spi_device;
    int cmd_size = 1; /* header + op_code + NOP */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    uint8_t in_buf[ARRAY_SIZE(out_buf)];
    struct spi_ioc_transfer k;
    int a, i;

    /* wait BUSY */
    wait_ms(WAIT_BUSY_SX1260_MS);

    /* check input variables */
    CHECK_NULL(lgw_spi_target);
    CHECK_NULL(data);

    spi_device = *(int *)lgw_spi_target; /* must check that spi_target is not null beforehand */

    if(access("/sys/class/gpio/gpio7",0) < 0){
        printf("NSS (GPIO7) is not exist!\n");   
        system("echo 7 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio7/direction");
    }
    //NSS->0
    system("echo 0 > /sys/class/gpio/gpio7/value");
    out_buf[0] = (uint8_t)op_code; //0x1d read command
    data[0] = 0x00;
    for(i = 0; i < (int)size; i++) {
        out_buf[cmd_size + i] = data[i];
    }
    command_size = cmd_size + size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.rx_buf = (unsigned long) in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);
    //NSS->1
    system("echo 1 > /sys/class/gpio/gpio7/value");
    //WaitOnBusy
    /*
    if( op_code != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
    */
    /* determine return code */
    /* determine return code */
    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        //*data = in_buf[command_size - 1];
        memcpy(data, in_buf + cmd_size, size);      //read data from input buffer
        return LGW_SPI_SUCCESS;
    }
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    int spi_device;
    int cmd_size = 3; /* op_code + address */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    struct spi_ioc_transfer k;
    int a, i;

    /* wait BUSY */
    wait_ms(WAIT_BUSY_SX1260_MS);

    /* check input variables */
    CHECK_NULL(lgw_spi_target);

    spi_device = *(int *)lgw_spi_target; /* must check that spi_target is not null beforehand */

    if(access("/sys/class/gpio/gpio7",0) < 0){
        printf("NSS (GPIO7) is not exist!\n");  
        system("echo 7 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio7/direction");
    }
    //NSS->1
    system("echo 1 > /sys/class/gpio/gpio7/value");
    out_buf[0] = RADIO_WRITE_REGISTER; //write command
    out_buf[1] =( address & 0xFF00 ) >> 8;
    out_buf[2] =address & 0x00FF;
    
    for(i = 0; i < (int)size; i++) {
        out_buf[cmd_size + i] = buffer[i];
    }
    command_size = cmd_size + size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.len = command_size;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);
    //NSS->1
    system("echo 1 > /sys/class/gpio/gpio7/value");
    //WaitOnBusy
    /*
    if( op_code != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
    */
    /* determine return code */
    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI write success\n");
        return LGW_SPI_SUCCESS;
    }
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    int spi_device;
    int cmd_size = 1; /* header + op_code + NOP */
    uint8_t out_buf[cmd_size + size];
    uint8_t command_size;
    uint8_t in_buf[ARRAY_SIZE(out_buf)];
    struct spi_ioc_transfer k;
    int a, i;

    /* wait BUSY */
    wait_ms(WAIT_BUSY_SX1260_MS);

    /* check input variables */
    CHECK_NULL(lgw_spi_target);
    CHECK_NULL(buffer);

    spi_device = *(int *)lgw_spi_target; /* must check that spi_target is not null beforehand */

    if(access("/sys/class/gpio/gpio7",0) < 0){
        printf("NSS (GPIO7) is not exist!\n");   
        system("echo 7 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio7/direction");
    }
    //NSS->0
    system("echo 0 > /sys/class/gpio/gpio7/value");
    out_buf[0] = RADIO_READ_REGISTER; //read command
    buffer[0] = 0x00;
    for(i = 0; i < (int)size; i++) {
        out_buf[cmd_size + i] = buffer[i];
    }
    command_size = cmd_size + size;

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buf;
    k.rx_buf = (unsigned long) in_buf;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);
    //NSS->1
    system("echo 1 > /sys/class/gpio/gpio7/value");
    //WaitOnBusy
    /*
    if( op_code != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
    */
    /* determine return code */
    /* determine return code */
    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        //*data = in_buf[command_size - 1];
        memcpy(buffer, in_buf + cmd_size, size);
        return LGW_SPI_SUCCESS;
    }
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int8_t SX126xGetRssiInst( void )
{
    uint8_t buf[1];
    int8_t rssi = 0;

    SX126xReadCommand( RADIO_GET_RSSIINST, buf, 1 );
    rssi = -buf[0] >> 1;
    return rssi;
}


RadioStatus_t SX126xGetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status = { .Value = 0 };

    stat = SX126xReadCommand( RADIO_GET_STATUS, NULL, 0 );
    status.Fields.CmdStatus = ( stat & ( 0x07 << 1 ) ) >> 1;
    status.Fields.ChipMode = ( stat & ( 0x07 << 4 ) ) >> 4;
    return status;
}
/*
void RadioInit( RadioEvents_t *events )
{
    RadioEvents = events;

    SX126xInit( RadioOnDioIrq );
    SX126xSetStandby( STDBY_RC );
    SX126xSetRegulatorMode( USE_DCDC );

    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    // Initialize driver timeout timers
    TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
    TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

    IrqFired = false;
}
*/
bool RadioIsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    RadioSleep( );

    RadioSetModem( modem );

    RadioSetChannel( freq );

    RadioRx( 0 );

    //DelayMs( 1 );
    usleep(100);
    carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        //rssi = RadioRssi( modem );
        rssi = SX126xGetRssiInst();
        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    RadioSleep( );
    return status;
}

void RadioSleep( void )
{
    SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;
    SX126xSetSleep( params );

    //DelayMs( 2 );
    usleep(200);
}

void SX126xSetSleep( SleepParams_t sleepConfig )
{
    //SX126xAntSwOff( ); //De-initializes the RF Switch I/Os pins interface (decrease the power consumption in MCU low power modes)

    uint8_t value = ( ( ( uint8_t )sleepConfig.Fields.WarmStart << 2 ) |
                      ( ( uint8_t )sleepConfig.Fields.Reset << 1 ) |
                      ( ( uint8_t )sleepConfig.Fields.WakeUpRTC ) );
    SX126xWriteCommand( RADIO_SET_SLEEP, &value, 1 );
    //SX126xSetOperatingMode( MODE_SLEEP );         //TX,RX LED signal?
}


void RadioSetModem( RadioModems_t modem )
{
    switch( modem )
    {
    default:
    case MODEM_FSK:
        SX126xSetPacketType( PACKET_TYPE_GFSK );
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        SX126xSetPacketType( PACKET_TYPE_LORA );
        // Public/Private network register is reset when switching modems
        if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous )
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork( RadioPublicNetwork.Current );
        }
        break;
    }
}

void RadioSetChannel( uint32_t freq )
{
    //SX126xSetRfFrequency( freq );
}

void SX126xSetRfFrequency( uint32_t frequency )
{
    uint8_t buf[4];
    uint32_t freq = 0;
    
    if( ImageCalibrated == false )
    {
        SX126xCalibrateImage( frequency );
        ImageCalibrated = true;
    }
    
    freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
    SX126xWriteCommand( RADIO_SET_RFFREQUENCY, buf, 4 );
}

void SX126xSetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    PacketType = packetType;
    SX126xWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}

void RadioSetPublicNetwork( bool enable )
{
    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    RadioSetModem( MODEM_LORA );
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
        SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
        SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );
    }
}


void SX126xCalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    SX126xWriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}
void RadioRx( uint32_t timeout )
{
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( RxContinuous == true )
    {
        SX126xSetRx( 0xFFFFFF ); // Rx Continuous
    }
    else
    {
        SX126xSetRx( RxTimeout << 6 );
    }
}