/*
 * SX1280.h
 *
 *  Created on: Jul 31, 2022
 *      Author: Sarah Goossens
 */

#ifndef SX1280_SX1280_H_
#define SX1280_SX1280_H_

#include "main.h"
#include <math.h>

void SX1280WaitOnBusy(void);
void SX1280HalReset(void);

//copied from sx1280.h of SX1280_DemoApp_V1_0 in System workbench for STM32:
/*!
 * \brief Structure describing the radio status
 */
typedef union
{
    struct // Structure of the radio status
    {
        uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
        uint8_t DmaBusy   : 1;  //!< Flag for DMA busy
        uint8_t CmdStatus : 3;  //!< Command status
        uint8_t ChipMode  : 3;  //!< Chip mode
    }Fields;
    uint8_t Value; // Serialized radio status
}RadioStatus_t;

//Represents all possible opcode understood by the radio
typedef enum RadioCommands_u
{
    RADIO_GET_STATUS                        = 0xC0,
    RADIO_WRITE_REGISTER                    = 0x18,
    RADIO_READ_REGISTER                     = 0x19,
    RADIO_WRITE_BUFFER                      = 0x1A,
    RADIO_READ_BUFFER                       = 0x1B,
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
    RADIO_GET_PACKETTYPE                    = 0x03,
    RADIO_SET_RFFREQUENCY                   = 0x86,
    RADIO_SET_TXPARAMS                      = 0x8E,
    RADIO_SET_CADPARAMS                     = 0x88,
    RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    RADIO_SET_MODULATIONPARAMS              = 0x8B,
    RADIO_SET_PACKETPARAMS                  = 0x8C,
    RADIO_GET_RXBUFFERSTATUS                = 0x17,
    RADIO_GET_PACKETSTATUS                  = 0x1D,
    RADIO_GET_RSSIINST                      = 0x1F,
    RADIO_SET_DIOIRQPARAMS                  = 0x8D,
    RADIO_GET_IRQSTATUS                     = 0x15,
    RADIO_CLR_IRQSTATUS                     = 0x97,
    RADIO_CALIBRATE                         = 0x89,
    RADIO_SET_REGULATORMODE                 = 0x96,
    RADIO_SET_SAVECONTEXT                   = 0xD5,
    RADIO_SET_AUTOTX                        = 0x98,
    RADIO_SET_AUTORX                        = 0x9E,
    RADIO_SET_LONGPREAMBLE                  = 0x9B,
    RADIO_SET_UARTSPEED                     = 0x9D,
    RADIO_SET_RANGING_ROLE                  = 0xA3,
    RADIO_GET_SILICON_VERSION               = 0x14,
}RadioCommands_t;

/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    LORA_BW_0200                            = 0x34,
    LORA_BW_0400                            = 0x26,
    LORA_BW_0800                            = 0x18,
    LORA_BW_1600                            = 0x0A,
}RadioLoRaBandwidths_t;


void SX1280HalReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size);
void SX1280HalWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size);
void SX1280HalWriteReadCommand(RadioCommands_t command, uint8_t *txbuffer, uint16_t size, uint8_t *rxbuffer);
RadioStatus_t SX1280GetStatus(void);
void SetRfFrequency(uint32_t frequency);
void WriteTXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size );
void SX1280SetSleep(void);
void GoToStandby(uint8_t mode);
void SetTypicalRegisterSettings(uint8_t packetType, uint8_t payloadLength, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t bleChannelNr);
void SetAllModulationParameters(uint8_t CurrentMode, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t spreading_factor);
void WriteRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size );
void ReadRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size );
void SetTx(void);
void ClearIrqStatus( uint16_t irq );
void EepromWrite( uint32_t addr, uint32_t data);
void SetRx(void);
void ReadRXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size );
double calc_y1(double x, int32_t rssi, int8_t chipwhip);//0-chip  1-whip
int32_t complement2( const uint32_t num, const uint8_t bitCnt );
double SX1280ComputeRangingCorrectionPolynome(const uint8_t sf, const RadioLoRaBandwidths_t bw, const double median);
int32_t SX1280GetLoRaBandwidth( );
uint8_t SX1280GetRangingPowerDeltaThresholdIndicator( void );
double SX1280GetRangingCorrectionPerSfBwGain( const uint8_t sf, const RadioLoRaBandwidths_t bw, const int8_t gain);
void SX1280Init(void); // added 20240413 to have a common location to start up the radio for the first time

//copied from rsf1280act.h of SX1280_NULEO in MBED studio:

#define FIRMWARE_VERSION    ( ( char* )"Firmware Version: v1.2" )//Added Star network & Learn of multiple IDs 2-2-2018

#define EEPROM_B0              0
#define EEPROM_B1              1
#define EEPROM_B2              2
#define EEPROM_B3              3
#define EEPROM_B4              4
#define EEPROM_B5              5

#define EEPROM_C0              0
#define EEPROM_C1              2
#define EEPROM_C2              4


#define XTAL_FREQ              52000000
#define FREQ_STEP              ( ( double )( XTAL_FREQ / pow( 2.0, 18.0 ) ) )
#define WaitOnBusy( )          while( bsy == 1 ){ }
#define STDBY_RC               0x00 //set standby command parameter
#define STDBY_XOSC             0x01 //set standby command parameter

#define MY_DATA_EEPROM_BASE       ( ( uint32_t )0x08080000U ) // STM32L073RZ MCU
#define EEPROM_MYID            0 //base address - 2 bytes long
#define EEPROM_YOURID          4 //base address - 2 bytes long
#define EEPROM_MODMODE         8 //base address - 1 byte long


#define RX_TIMEOUT_TICK_SIZE   RADIO_TICK_SIZE_1000_US
#define IRQ_RADIO_ALL          0xFFFF

#define RANGINGSLAVE           0
#define RANGINGMASTER          1

#define RAWRESULT              0
#define AVERAGERESULT          1
#define DEBIASEDRESULT         2
#define FILTEREDRESULT         3

#define MYRXTIMEOUT_LONG       4000000

#define GFSK                     0
#define LORA                     1
#define RANGING                  2
#define FLRC                     3

#define DEFAULTSPREADINGFACTOR   12
#define RANGINGSPREADINGFACTOR6  6
#define RANGINGSPREADINGFACTOR10 10
#define PTTSPREADINGFACTOR       6
#define PTTPACKETSIZE            255

#define STARTDELAY             6500

#define TICKERSF6                0.006
#define TICKERSF10               0.050

#define RSSITHRESHOLD            -80

#define REG_LR_SYNCWORDBASEADDRESS1                 0x09CE
#define REG_LR_SYNCWORDBASEADDRESS2                 0x09D3
#define REG_LR_SYNCWORDBASEADDRESS3                 0x09D8
#define REG_LR_CRCSEEDBASEADDR                      0x09C8
#define REG_LR_CRCPOLYBASEADDR                      0x09C6

#define PLAY                   0
#define RECORD                 1
#define RANGINGCHCHANGE        2

#define LOW                    2402000000
#define MID                    2440000000
#define HIGH                   2480000000

#define DEFAULTMODGFSKP1       6 //Bandwidth (MHz DSB)         P86
#define DEFAULTMODGFSKP2       4  //Modindex
#define DEFAULTMODGFSKP3       2  //BT                          P87

#define DEFAULTMODLORAP1       DEFAULTSPREADINGFACTOR //Spreading factor            P111
#define DEFAULTMODLORAP2       1  //Bandwidth [kHz]             P112
#define DEFAULTMODLORAP3       1  //Coding rate

#define DEFAULTMODFLRCP1       3 //Bitrate (Mb/s)  Bandwidth (MHz DSB)  P32  P102
#define DEFAULTMODFLRCP2       1 //Coding rate                          P103
#define DEFAULTMODFLRCP3       2 //BT   Bandwidth-Time bit period product




#define DEFAULTPACKETGFSKP1    8 //Preamble length in bits       P87
#define DEFAULTPACKETGFSKP2    5 //Sync Word size in bytes       P88
#define DEFAULTPACKETGFSKP3    2 //Sync Word combination to use
#define DEFAULTPACKETGFSKP4    5 //Packet Length mode
#define DEFAULTPACKETGFSKP5    12 //Payload length in bytes
#define DEFAULTPACKETGFSKP6    4 //CRC type
#define DEFAULTPACKETGFSKP7    1 //Whitening mode

#define DEFAULTPACKETLORAP1    0 //Preamble length in symbols    P113
#define DEFAULTPACKETLORAP2    1 //Header mode
#define DEFAULTPACKETLORAP3    6 //PayloadLength
#define DEFAULTPACKETLORAP4    1 //CRC mode
#define DEFAULTPACKETLORAP5    1 //LoRa IQ swap
#define DEFAULTPACKETLORAP6    0 //N/A
#define DEFAULTPACKETLORAP7    0 //N/A

#define DEFAULTPACKETFLRCP1    8 //Preamble length in bits       P103       was 6
#define DEFAULTPACKETFLRCP2    3 //Sync Word size in bytes       P104
#define DEFAULTPACKETFLRCP3    2 //Sync Word combination to use
#define DEFAULTPACKETFLRCP4    2 //Packet Length mode            P105
#define DEFAULTPACKETFLRCP5    6 //Payload length in bytes - must be 6-127...
#define DEFAULTPACKETFLRCP6    4 //CRC type
#define DEFAULTPACKETFLRCP7    2 //In FLRC packet type, it is not possible to enable whitening. You must always set the value of packetParam7 to disabled.

// modParam1 options for Lora:
#define LORA_SF5        0x50 // Lora Spreading Factor  5
#define LORA_SF6        0x60 // Lora Spreading Factor  6
#define LORA_SF7        0x70 // Lora Spreading Factor  7
#define LORA_SF8        0x80 // Lora Spreading Factor  8
#define LORA_SF9        0x90 // Lora Spreading Factor  9
#define LORA_SF10       0xA0 // Lora Spreading Factor 10
#define LORA_SF11       0xB0 // Lora Spreading Factor 11
#define LORA_SF12       0xC0 // Lora Spreading Factor 12

// modParam2 options for Lora:
#define LORA_BW_0200     0x34 // Bandwidth  203.125 kHz
#define LORA_BW_0400     0x26 // Bandwidth  406.25  kHz
#define LORA_BW_0800     0x18 // Bandwidth  812.5   kHz
#define LORA_BW_1600     0x0A // Bandwidth 1625.0   kHz

// modParam3 options for Lora:
#define LORA_CR_4_5      0x01 // Lora Coding Rate (not long interleaving) 4/5
#define LORA_CR_4_6      0x02 // Lora Coding Rate (not long interleaving) 4/6
#define LORA_CR_4_7      0x03 // Lora Coding Rate (not long interleaving) 4/7
#define LORA_CR_4_8      0x04 // Lora Coding Rate (not long interleaving) 4/8
#define LORA_CR_LI_4_5   0x05 // Lora Coding Rate (long interleaving) 4/5
#define LORA_CR_LI_4_6   0x06 // Lora Coding Rate (long interleaving) 4/6
#define LORA_CR_LI_4_8   0x07 // Lora Coding Rate (long interleaving) 4/8

// Packet parameter 1 options for Lora - Preamble Length:
// packetParam1 defines the preamble length number expressed in LoRa symbols. Recommended value is 12 symbols.
// P113           Preamble length in symbols
//                preamble length =LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
// mant = 3;
// exp = 2;
// buf[0] = mant | (exp<<4);
// orig		buf[0] = 0x23;   //Recommended value is 12 symbols 0x0C

// Packet parameter 2 options for Lora - Header Type:
#define LORA_PACKET_VARIABLE_LENGTH  0x00 // The packet is on variable size, header included
#define LORA_PACKET_FIXED_LENGTH     0x80 // The packet is known on both sides, no header included in the packet
#define LORA_PACKET_EXPLICIT         LORA_PACKET_VARIABLE_LENGTH
#define LORA_PACKET_IMPLICIT         LORA_PACKET_FIXED_LENGTH

// Packet parameter 3 options for Lora - Payload Length (1 to 255 bytes):

// Packet parameter 4 options for Lora - CRC:
#define LORA_CRC_OFF     0x00 // CRC not used
#define LORA_CRC_ON      0x20 // CRC activated

// Packet parameter 5 options for Lora - InvertIQ/chirp invert:
#define LORA_IQ_INVERTED 0x00
#define LORA_IQ_NORMAL   0x40

// Packet parameter 6 options for Lora - not used
// Packet parameter 7 options for Lora - not used

typedef enum
{
    RADIO_TICK_SIZE_0015_US                 = 0x00,
    RADIO_TICK_SIZE_0062_US                 = 0x01,
    RADIO_TICK_SIZE_1000_US                 = 0x02,
    RADIO_TICK_SIZE_4000_US                 = 0x03,
}RadioTickSizes_t;

typedef struct TickTime_s
{
    RadioTickSizes_t Step;                                  //!< The step of ticktime
    /*!
     * \brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t NbSteps;
}TickTime_t;

//DLP-RFS1280ACT packet types
#define     LOCATOR_PING_PACKET            0x57
#define     LEARN_BROADCAST_PACKET         0x58
#define     STAY_QUIET_PACKET              0x59
#define     LISTEN_PACKET                  0x5A
#define     SIMPLE_PING_PACKET             0x21
#define     PTT_AUDIO_PACKET               0x22
#define     RANGING_PACKET                 0x23



/*!
 * \brief The address of the register holding LORA packet parameters
 */
#define REG_LR_PACKETPARAMS                     0x903

/*!
 * \brief The address of the register holding payload length
 *
 * \remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
 */
#define REG_LR_PAYLOADLENGTH                    0x901
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB    0x954
#define REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK   0x000FFFFF

#define CHANNELS 40
#define READINGSPERCHANNEL 10


///*
// * Frequency look up table :
// * To avoid Wifi channels, 40 Bluetooth channels are defined below (they already
// * avoid Wifi common channels) : from 2402 MHz to 2480 MHz, step 2 MHz.
// * User can define channel count for Ranging run, and it is optimized to have
// * several frequencies in the largest band as possible. Also the 40 frequencies
// * are generated by random sorting to preferate the 10 first in the largest band
// * as possible (10 is the shortest channel count the user can choose).
// */
//const uint32_t Channelf[] =
//{
// 2450000000,
// 2476000000,
// 2436000000,
// 2430000000,
// 2468000000,
// 2458000000,
// 2416000000,
// 2424000000,
// 2478000000,
// 2456000000,
// 2448000000,
// 2462000000,
// 2472000000,
// 2432000000,
// 2446000000,
// 2422000000,
// 2442000000,
// 2460000000,
// 2474000000,
// 2414000000,
// 2464000000,
// 2454000000,
// 2444000000,
// 2404000000,
// 2434000000,
// 2410000000,
// 2408000000,
// 2440000000,
// 2452000000,
// 2480000000,
// 2426000000,
// 2428000000,
// 2466000000,
// 2418000000,
// 2412000000,
// 2406000000,
// 2470000000,
// 2438000000,
// 2420000000,
// 2402000000,
//};

#endif /* SX1280_SX1280_H_ */
