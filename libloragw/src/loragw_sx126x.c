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
 * Tx and Rx timers
 */
//TimerEvent_t TxTimeoutTimer;
//TimerEvent_t RxTimeoutTimer;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;
uint8_t MaxPayloadLength = 0xFF;
int lbt_target=20;

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

bool RxContinuous = false;

/*
 * Local types definition
 */

 /*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};

/*!
 * Radio hardware and global parameters
 */
SX126x_t SX126x;
/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
}RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

bool IrqFired = false;

/*!
 * \brief Stores the current packet header type set in the radio
 */
static volatile RadioLoRaPacketLengthsMode_t LoRaHeaderType;

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

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

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq( void* context );

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
    //NSS->0
    system("echo 0 > /sys/class/gpio/gpio7/value");
    out_buf[0] = (uint8_t)op_code; 

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
    WaitOnBusy();
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
    int cmd_size = 2; /* op_code + (NOP) */
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
    out_buf[0] = (uint8_t)op_code; 
    out_buf[1] = 0x00;
    for(i = 0; i < (int)size; i++) {
            data[i] = 0x00;
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
    WaitOnBusy();

    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        //*data = in_buf[command_size - 1];
        //printf("readcmd. status=0x%02X\n",in_buf[cmd_size - 1]);
        //for (int cnt=0;cnt<strlen(in_buf);cnt++) {
        //    printf("in_buf=0x%02X\n",in_buf[cnt]);
        //}
        memcpy(data, in_buf + cmd_size, size);      //read data from input buffer
        //return LGW_SPI_SUCCESS;
        return in_buf[cmd_size-1];  //return status
    }
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    int spi_device;
    int cmd_size = 3; /* op_code + address+ address */
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
    //NSS->0
    system("echo 0 > /sys/class/gpio/gpio7/value");
    out_buf[0] = RADIO_WRITE_REGISTER; //write command (0X0D)
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
    WaitOnBusy();

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
    int cmd_size = 4; /* op_code + address+address + NOP */
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
    out_buf[0] = RADIO_READ_REGISTER; //read command (0X1D)
    out_buf[1] =( address & 0xFF00 ) >> 8;
    out_buf[2] =address & 0x00FF;
    for(i = 0; i < (int)size; i++) {
        buffer[i] = 0x00;
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
    WaitOnBusy();
    /*
    if( op_code != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
    */
    /* determine return code */

    if (a != (int)k.len) {
        DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        
        for (int cnt=0;cnt<strlen(in_buf);cnt++) {
            printf("in_buf=0x%02X\n",in_buf[cnt]);
        }
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

int16_t RadioRssi( RadioModems_t modem )
{
    return SX126xGetRssiInst( );
}

int8_t SX126xGetRssiInst( void )
{
    uint8_t buf;
    int8_t rssi = 0;

    SX126xReadCommand( RADIO_GET_RSSIINST, &buf, 1 );
    rssi = -buf >> 1; //-(RSSIINST/2)
    return rssi;
}


RadioStatus_t SX126xGetStatus( void )
{
    uint8_t stat = 0;
    uint8_t buf;
    RadioStatus_t status = { .Value = 0 };
    //stat = SX126xReadCommand( RADIO_GET_STATUS, NULL, 0 );
    stat = SX126xReadCommand( RADIO_GET_STATUS, &buf, 1 );
    status.Fields.CmdStatus = ( stat & ( 0x07 << 1 ) ) >> 1;
    status.Fields.ChipMode = ( stat & ( 0x07 << 4 ) ) >> 4;
    return status;
}

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
/*    
    TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
    TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

    IrqFired = false;
*/    
}

void RadioOnDioIrq( void* context )
{
    IrqFired = true;
}

void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}

void SX126xInit( DioIrqHandler dioIrq )
{
    SX126xReset( );

    SX126xIoIrqInit( dioIrq );

    SX126xWakeup( );
    SX126xSetStandby( STDBY_RC );

    // Initialize TCXO control
    SX126xIoTcxoInit( );

    SX126xSetDio2AsRfSwitchCtrl( true );
    //SX126xSetOperatingMode( MODE_STDBY_RC );
}
void SX126xReset( void )
{
    //GPIO init (export)
}

void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
    SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    //GpioSetInterrupt( &SX126x.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, dioIrq );
}

void SX126xWakeup( void )
{
    //CRITICAL_SECTION_BEGIN( );

    SX126xGetStatus;

    //CRITICAL_SECTION_END( );
}

bool RadioIsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    RadioSleep( );

    RadioSetModem( modem );

    RadioSetChannel( freq );

    //RadioRx( 0 );

    //DelayMs( 1 );
    usleep(100);
    //carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
   // while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    //{
        rssi = RadioRssi( modem );
        //rssi = SX126xGetRssiInst();
        printf("SX1261: RSSI=%d\n",rssi);
        if( rssi > rssiThresh )
        {
            status = false;
            //break;
        }
    //}
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
    SX126xSetRfFrequency( freq );
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
/*
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }
*/
    if( RxContinuous == true )
    {
        SX126xSetRx( 0xFFFFFF ); // Rx Continuous
    }
    else
    {
        SX126xSetRx( RxTimeout << 6 );
    }
}

void SX126xSetTx( uint32_t timeout )
{
    uint8_t buf[3];

    //SX126xSetOperatingMode( MODE_TX );

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_TX, buf, 3 );
}

void SX126xSetRx( uint32_t timeout )
{
    uint8_t buf[3];

    //SX126xSetOperatingMode( MODE_RX );

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
}
void WaitOnBusy(){

    if(access("/sys/class/gpio/gpio4",0) < 0){
        printf("Busy (GPIO4) is not exist!\n");   
        system("echo 4 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio4/direction");
    }
    while( GPIORead(4) == 1 );
}

int GPIORead(int pin) {
    #define VALUE_MAX 30
	char path[VALUE_MAX];
	char value_str[3];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for reading!\n");
		return(-1);
	}

	if (-1 == read(fd, value_str, 3)) {
		fprintf(stderr, "Failed to read value!\n");
		return(-1);
	}

	close(fd);

	return(atoi(value_str));
}

void SX126xSetStandby( RadioStandbyModes_t standbyConfig )
{
    SX126xWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    /*
    if( standbyConfig == STDBY_RC )
    {
        SX126xSetOperatingMode( MODE_STDBY_RC );
    }
    else
    {
        SX126xSetOperatingMode( MODE_STDBY_XOSC );
    }*/
}

void RadioStandby( void )
{
    SX126xSetStandby( STDBY_RC );
}

void SX126xIoTcxoInit( void )
{
    CalibrationParams_t calibParam;

    //SX126xSetDio3AsTcxoCtrl( TCXO_CTRL_1_7V, SX126xGetBoardTcxoWakeupTime( ) << 6 ); // convert from ms to SX126x time base
    SX126xSetDio3AsTcxoCtrl( TCXO_CTRL_1_7V, 0 << 6 ); // convert from ms to SX126x time base
    calibParam.Value = 0x7F;
    SX126xCalibrate( calibParam );
}
/*
uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}
*/
void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF ); // Delay(23:0) *15.625 µs
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}

void SX126xCalibrate( CalibrationParams_t calibParam )
{
    uint8_t value = ( ( ( uint8_t )calibParam.Fields.ImgEnable << 6 ) |
                      ( ( uint8_t )calibParam.Fields.ADCBulkPEnable << 5 ) |
                      ( ( uint8_t )calibParam.Fields.ADCBulkNEnable << 4 ) |
                      ( ( uint8_t )calibParam.Fields.ADCPulseEnable << 3 ) |
                      ( ( uint8_t )calibParam.Fields.PLLEnable << 2 ) |
                      ( ( uint8_t )calibParam.Fields.RC13MEnable << 1 ) |
                      ( ( uint8_t )calibParam.Fields.RC64KEnable ) );

    SX126xWriteCommand( RADIO_CALIBRATE, &value, 1 );
}


void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{

    RxContinuous = rxContinuous;
    if( rxContinuous == true )
    {
        symbTimeout = 0;
    }
    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
        case MODEM_FSK:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;

            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;
            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth );

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetWhiteningSeed( 0x01FF );

            RxTimeout = ( uint32_t )( symbTimeout * ( ( 1.0 / ( double )datarate ) * 8.0 ) * 1000 );
            break;

        case MODEM_LORA:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126xSetLoRaSymbNumTimeout( symbTimeout );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate = ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;

            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );

            // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
            if( SX126x.PacketParams.Params.LoRa.InvertIQ == LORA_IQ_INVERTED )
            {
                // RegIqPolaritySetup = @address 0x0736
                SX126xWriteRegister( 0x0736, SX126xReadRegister( 0x0736 ) & ~( 1 << 2 ) );
            }
            else
            {
                // RegIqPolaritySetup @address 0x0736
                SX126xWriteRegister( 0x0736, SX126xReadRegister( 0x0736 ) | ( 1 << 2 ) );
            }
            // WORKAROUND END

            // Timeout Max, Timeout handled directly in SetRx function
            RxTimeout = 0xFFFF;

            break;
    }
}


void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    if( modem == MODEM_LORA )
    {
        SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
        SX126xSetPacketParams( &SX126x.PacketParams );
    }
    else
    {
        if( SX126x.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH )
        {
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength = max;
            SX126xSetPacketParams( &SX126x.PacketParams );
        }
    }
}

void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}

void SX126xSetLoRaSymbNumTimeout( uint8_t SymbNum )
{
    SX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1 );
}


void SX126xSetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t n;
    uint32_t tempVal = 0;
    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        SX126xSetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        n = 8;
        tempVal = ( uint32_t )( 32 * ( ( double )XTAL_FREQ / ( double )modulationParams->Params.Gfsk.BitRate ) );
        buf[0] = ( tempVal >> 16 ) & 0xFF;
        buf[1] = ( tempVal >> 8 ) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
        buf[4] = modulationParams->Params.Gfsk.Bandwidth;
        tempVal = ( uint32_t )( ( double )modulationParams->Params.Gfsk.Fdev / ( double )FREQ_STEP );
        buf[5] = ( tempVal >> 16 ) & 0xFF;
        buf[6] = ( tempVal >> 8 ) & 0xFF;
        buf[7] = ( tempVal& 0xFF );
        SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
        break;
    case PACKET_TYPE_LORA:
        n = 4;
        buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
        buf[1] = modulationParams->Params.LoRa.Bandwidth;
        buf[2] = modulationParams->Params.LoRa.CodingRate;
        buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

        SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );

        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
}


void SX126xSetPacketParams( PacketParams_t *packetParams )
{
    uint8_t n;
    uint8_t crcVal = 0;
    uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        SX126xSetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM )
        {
            SX126xSetCrcSeed( CRC_IBM_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_IBM );
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT )
        {
            SX126xSetCrcSeed( CRC_CCITT_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = packetParams->Params.Gfsk.CrcLength;
        }
        n = 9;
        buf[0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.Gfsk.PreambleLength;
        buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
        buf[3] = ( packetParams->Params.Gfsk.SyncWordLength /*<< 3*/ ); // convert from byte to bit
        buf[4] = packetParams->Params.Gfsk.AddrComp;
        buf[5] = packetParams->Params.Gfsk.HeaderType;
        buf[6] = packetParams->Params.Gfsk.PayloadLength;
        buf[7] = crcVal;
        buf[8] = packetParams->Params.Gfsk.DcFree;
        break;
    case PACKET_TYPE_LORA:
        n = 6;
        buf[0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.LoRa.PreambleLength;
        buf[2] = LoRaHeaderType = packetParams->Params.LoRa.HeaderType;
        buf[3] = packetParams->Params.LoRa.PayloadLength;
        buf[4] = packetParams->Params.LoRa.CrcMode;
        buf[5] = packetParams->Params.LoRa.InvertIQ;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
    SX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
}

static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    if( bandwidth == 0 )
    {
        return( 0x1F );
    }

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i+1].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX126xSetCrcSeed( uint16_t seed )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( seed & 0xFF );

    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            SX126xWriteRegisters( REG_LR_CRCSEEDBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}

uint8_t SX126xSetSyncWord( uint8_t *syncWord )
{
    SX126xWriteRegisters( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
    return 0;
}

void SX126xSetWhiteningSeed( uint16_t seed )
{
    uint8_t regValue = 0;
    
    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            regValue = SX126xReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
            regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
            SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
            SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, ( uint8_t )seed );
            break;

        default:
            break;
    }
}

void SX126xSetCrcPolynomial( uint16_t polynomial )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( polynomial & 0xFF );

    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            SX126xWriteRegisters( REG_LR_CRCPOLYBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}

RadioPacketTypes_t SX126xGetPacketType( void )
{
    return PacketType;
}
void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX126xWriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}

void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    //if( SX126xGetDeviceId( ) == SX1261 ) 
        if( power == 15 )
        {
            SX126xSetPaConfig( 0x06, 0x00, 0x01, 0x01 );
        }
        else
        {
            SX126xSetPaConfig( 0x04, 0x00, 0x01, 0x01 );
        }
        if( power >= 14 )
        {
            power = 14;
        }
        else if( power < -17 )
        {
            power = -17;
        }
        SX126xWriteRegister( REG_OCP, 0x18 ); // current max is 80 mA for the whole device

    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SX126xWriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}

void SX126xClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}


void lgw_sx126x_init(void) {
    
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
    int set_freq=920000000;
    RadioSetChannel(set_freq);
    printf("SX1261: Frequency=%d\n",set_freq); 
    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    SX126xSetSyncWord( ( uint8_t[] ){ 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0 } );
    //SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX126xSetDioIrqParams( 0x0242, 0x0242, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX126xWriteRegister( REG_RX_GAIN, 0X96 );
    SX126xSetRx( 0xFFFFFF ); // Rx Continuous
    printf("SX126X INIT. Done\n");
    //return 0;
}
bool lgw_sx126x_check_ch_free(uint32_t set_freq , int8_t rssiThres) {
    int rssi;

    RadioSetChannel(set_freq);
    //printf("SX1261: Frequency=%d\n",set_freq); 
    rssi = SX126xGetRssiInst( );    
    printf("SX1261: RSSI=%d\n",rssi); 
    if ((rssiThres <= 20) || ((rssi*-1) >= rssiThres) ){
        return true;
    } else {
        return false;
    }
        
    
}