/*
 * app_network_connect.c
 *
 *  Created on: Apr 15, 2023
 *      Author: Sarah Goossens

 * at startup:
 *
 * --> HOST: after synchronization with RTC and GNSS is done, continuously repeat to broadcast my ID, xEpoch and pairing command, then listen for short period
 * --> NODE: after synchronization with RTC, continuously repeat to listen for pairing command from a HOST, if received, make yourself known
 *
 * message build-up for the broadcast pairing message from a host:
 *
 *  txBuf[ 0] = STARTDELIMITER;           // "S" Start Delimiter
 *  txBuf[ 1] = moduleTable[0].ID[0];     // MyID LSB
 *  txBuf[ 2] = moduleTable[0].ID[1];
 *  txBuf[ 3] = moduleTable[0].ID[2];
 *  txBuf[ 4] = moduleTable[0].ID[3];
 *  txBuf[ 5] = moduleTable[0].ID[4];
 *  txBuf[ 6] = moduleTable[0].ID[5];
 *  txBuf[ 7] = moduleTable[0].ID[6];
 *  txBuf[ 8] = moduleTable[0].ID[7];     // MyID MSB
 *  txBuf[ 9] = (uint8_t)(xEpoch & 0xFF); // My Epoch LSB
 *  txBuf[10] = (uint8_t)(xEpoch & 0xFF); // My Epoch 2nd Byte
 *  txBuf[11] = (uint8_t)(xEpoch & 0xFF); // My Epoch 3rd Byte
 *  txBuf[12] = (uint8_t)(xEpoch & 0xFF); // My Epoch 4th Byte
 *  txBuf[13] = (uint8_t)(xEpoch & 0xFF); // My Epoch 5th Byte
 *  txBuf[14] = (uint8_t)(xEpoch & 0xFF); // My Epoch 6th Byte
 *  txBuf[15] = (uint8_t)(xEpoch & 0xFF); // My Epoch 7th Byte
 *  txBuf[16] = (uint8_t)(xEpoch & 0xFF); // My Epoch MSB
 *  txBuf[17] = HOSTISTHEREANYONE;        // "P" pairing command, "Hello, is there anyone?"
 *  txBuf[18] = (uint8_t)CheckSumCalc;    // check sum
 *  txBuf[19] = ENDDELIMITER;             // "E" Message End delimiter
 *
 *  Example:
 *
 *  5381F3A30A073C1C00B60596322700000050CF45
 *   |+-------+------++-------+------+ | | +-> "E" Message End delimiter
 *   |        |               |        | +---> Check Sum
 *   |        |               |        +-----> "P" pairing command, "Hello, is there anyone?"
 *   |        |               +--------------> My epoch
 *   |        +------------------------------> MyID
 *   +---------------------------------------> "S" Start Delimiter
 *
 * message build-up of the answering pairing message to a host
 *
 *  txBuf[ 0] = STARTDELIMITER;                    // "S" Start Delimiter
 *  txBuf[ 1] = moduleTable[0].ID[0];              // MyID LSB
 *  txBuf[ 2] = moduleTable[0].ID[1];
 *  txBuf[ 3] = moduleTable[0].ID[2];
 *  txBuf[ 4] = moduleTable[0].ID[3];
 *  txBuf[ 5] = moduleTable[0].ID[4];
 *  txBuf[ 6] = moduleTable[0].ID[5];
 *  txBuf[ 7] = moduleTable[0].ID[6];
 *  txBuf[ 8] = moduleTable[0].ID[7];              // MyID MSB
 *  txBuf[ 9] = moduleTable[1].ID[0];              // Host ID LSB
 *  txBuf[10] = moduleTable[1].ID[1];
 *  txBuf[11] = moduleTable[1].ID[2];
 *  txBuf[12] = moduleTable[1].ID[3];
 *  txBuf[13] = moduleTable[1].ID[4];
 *  txBuf[14] = moduleTable[1].ID[5];
 *  txBuf[15] = moduleTable[1].ID[6];
 *  txBuf[16] = moduleTable[1].ID[7];              // Host ID MSB
 *  txBuf[17] = (((int32_t)hostEfeHz)>>24) & 0xFF; // received ESTIMATED_FREQUENCY_ERROR MSB
 *  txBuf[18] = (((int32_t)hostEfeHz)>>16) & 0xFF;
 *  txBuf[19] = (((int32_t)hostEfeHz)>> 8) & 0xFF;
 *  txBuf[20] =  ((int32_t)hostEfeHz)      & 0xFF; // received ESTIMATED_FREQUENCY_ERROR LSB
 *  txBuf[21] = hostRssi;                          // Communication RSSI of packet received by the node
 *  txBuf[22] = NODEIAMHERE;                       // "Y" pairing ANSWER command, "Yes, I'm here!"
 *  txBuf[23] = (uint8_t)CheckSumCalc;             // Check Sum
 *  txBuf[24] = ENDDELIMITER;                      // "E" Message End delimiter
 *
 *  Example:
 *
 *  517 [app_init] Sensor Unique ID: 81F3A30A073C1C00
 *                    81F3A30A073C1C00
 *
 *                    81F3A30A073C1C00
 *  5380F3A30A07391D0081F3A30A073C1C000000000000590F45
 *   |+-------+------++-------+------++--+---+ | | | +-> "E" Message End delimiter
 *   |        |               |          |     | | +---> Check Sum
 *   |        |               |          |     | +-----> "Y" pairing ANSWER command, "Yes, I'm here!"
 *   |        |               |          |     +-------> Communication RSSI of packet received by the node
 *   |        |               |          +-------------> received ESTIMATED_FREQUENCY_ERROR
 *   |        |               +------------------------> Host ID
 *   |        +----------------------------------------> MyID
 *   +-------------------------------------------------> "S" Start Delimiter
 *
 *
 */

#include "main.h"
#include "FreeRTOS.h"
#if !STM32WBAUSED
  #include "cmsis_os.h"
#endif

#include "app_network_connect.h"
#include "usart.h"              // to declare huart2
#include <string.h>
#include "../Drivers/SX1280/SX1280.h"
#include "../Drivers/PCF2131/PCF2131.h"
#include "time.h"
#include "app_rtc.h"
#include "app_hal_pps.h"
#include "spi.h"
#include "app_rtc_sync.h"
#include <inttypes.h>
#include "app_supercap.h"

#define PRINTF_APP_NETWORK_CONNECT 1

// For the Antenna RSSI test following settings are being set:
// Modulation Parameter 2 options for Lora: Spreading factor:
//   SF5  LORA_SF5  0x50
//   SF6  LORA_SF6  0x60
//   SF7  LORA_SF7  0x70
//   SF8  LORA_SF8  0x80
//   SF9  LORA_SF9  0x90
//   SF10 LORA_SF10 0xA0
//   SF11 LORA_SF11 0xB0
//   SF12 LORA_SF12 0xC0
// Modulation Parameter 2 options for Lora: Bandwidth:
//    203.125kHz LORA_BW_0200 0x34
//    406.250kHz LORA_BW_0400 0x26
//    812.500kHz LORA_BW_0800 0x18
//   1625.000kHz LORA_BW_1600 0x0A
// Modulation Parameter 3 options for Lora: Coding Rate:
//                     4/5 LORA_CR_4_5      0x01
//                     4/6 LORA_CR_4_6      0x02
//                     4/7 LORA_CR_4_7      0x03
//                     4/8 LORA_CR_4_8      0x04
//   long interleaving 4/5 LORA_CR_LI_4_5   0x05
//   long interleaving 4/6 LORA_CR_LI_4_6   0x06
//   long interleaving 4/7 LORA_CR_LI_4_7   0x07
// Channel: the hopping sequence is using the BLE channel plan (see AN1200-50_SX1280_Ranging_Protocol_V1_1.pdf: 7. The Channel Plan)
//   BLE     Hopping  Frequency
// Channel  Sequence    [MHz]
//  Number
//   22        CH1       2450
//   37        CH2       2402
//   35        CH3       2476
//   15        CH4       2436
//   12        CH5       2430
//   31        CH6       2468
//   26        CH7       2458
//    6        CH8       2416
//   10        CH9       2424
//   36       CH10       2478
//   25       CH11       2456
//   21       CH12       2448
//   28       CH13       2462
//   33       CH14       2472
//   13       CH15       2432
//   20       CH16       2446
//    9       CH17       2422
//   18       CH18       2442
//   27       CH19       2460
//   34       CH20       2474
//    5       CH21       2414
//   29       CH22       2464
//   24       CH23       2454
//   19       CH24       2444
//    0       CH25       2404
//   14       CH26       2434
//    3       CH27       2410
//    2       CH28       2408
//   17       CH29       2440
//   23       CH30       2452
//   39       CH31       2480
//   38       CH32       2426
//   11       CH33       2428
//   30       CH34       2466
//    7       CH35       2418
//    4       CH36       2412
//    1       CH37       2406
//   32       CH38       2470
//   17       CH39       2438
//    8       CH40       2420


// Channel: list according to BLE channel number. The hopping sequence which will be used is the BLE channel number 0...39
//   BLE     Hopping  Frequency
// Channel  Sequence    [MHz]
//  Number
//    0       CH25       2404
//    1       CH37       2406
//    2       CH28       2408
//    3       CH27       2410
//    4       CH36       2412
//    5       CH21       2414
//    6        CH8       2416
//    7       CH35       2418
//    8       CH40       2420
//    9       CH17       2422
//   10        CH9       2424
//   11       CH33       2428
//   12        CH5       2430
//   13       CH15       2432
//   14       CH26       2434
//   15        CH4       2436
//   17       CH39       2438
//   17       CH29       2440
//   18       CH18       2442
//   19       CH24       2444
//   20       CH16       2446
//   21       CH12       2448
//   22        CH1       2450
//   23       CH30       2452
//   24       CH23       2454
//   25       CH11       2456
//   26        CH7       2458
//   27       CH19       2460
//   28       CH13       2462
//   29       CH22       2464
//   30       CH34       2466
//   31        CH6       2468
//   32       CH38       2470
//   33       CH14       2472
//   34       CH20       2474
//   35        CH3       2476
//   36       CH10       2478
//   37        CH2       2402    BLE advertising channel
//   38       CH32       2426    BLE advertising channel
//   39       CH31       2480    BLE advertising channel

extern module            moduleTable[MAXNRMODULES];
extern char              uart_buf[200];
extern char              byteString[3];
extern char              receivedSensorID[8]; // Received SensorID, 3 bytes
extern uint64_t          timeStampInt;        // RTOS tick at the time of the external interrupt (in this case NINTA or RTC_PPS)
extern SemaphoreHandle_t timeStampIntMutex;
extern time_t            rtc_epoch;
extern time_t            received_epoch;      // received epoch from external module: if host, this will be used for time synchronization purposes
                                              //                                      if node, to check if node needs to be synchronized
extern uint64_t          receivedxEpoch;
extern SemaphoreHandle_t recEpochMutex;
extern uint8_t           changeClock;
extern uint64_t          startOfTransmit;
extern uint64_t          timeStamp8;
extern uint64_t          timeStamp1;
extern uint64_t          timeStamp5;
extern uint64_t          timeStamp4;
extern uint64_t          xEpochLnt1;
extern uint64_t          xEpochLnt8;
extern uint64_t          xEpochLht5;
extern uint64_t          xEpochLht6;
extern uint64_t          rtosTimeStampBCastStrt;
extern uint64_t          rtosTimeStampBCastDone;
extern uint64_t          rtosTimeStampRTCPPS;
extern uint64_t          rtosTimeStampGNSSPPS;
extern uint32_t          gnss_pps;
extern float             compensateTicks;
extern uint32_t          pairingOngoing;
extern uint64_t          rtosTimeStampStartRTC;
extern uint32_t          superCapAvailable;
extern uint32_t          rtosDriftToGNSS;
extern uint32_t          totalRtosDriftToGNSS;

extern uint64_t          previousrtosTimeStampGNSSPPS;

static uint8_t txBuf[41];
static uint8_t rxBuf[41];
static uint8_t packetStatusBuf[7];
static uint8_t rawEfeBuf[4];
uint8_t        moduleExists;      // for a Host: this is 0 at the start of receiving a new message from a node. When this is 1 it means that the Node already exists in the moduletable.
struct tm      rtc_tmStructure;
struct tm     *rec_tmStructure;     // pointer to received time/date from received epoch
uint64_t       xEpoch        = 0;   // transmitted or received epoch, includes subsec (to 1/100th of s).

int32_t        xEpochOffset  = 0;   // xEpoch date/time offset between host and node
uint32_t       CheckSumCalc  = 0;   //
PCF2131_time_t rtc_time;
uint32_t       targetID;            // Your ID, to identify the sensor ID (Host or Node) in case of one to one communication. Is 0 in case of broadcast.
char           receivedTargetID[8]; // Received targetID, this should be identical to moduleTable[0].ID (MyID)
uint8_t        recSubSeconds = 0;
uint32_t       hostEfe       = 0;   // received ESTIMATED_FREQUENCY_ERROR from host (raw format)
float          hostEfeHz     = 0.0; // received ESTIMATED_FREQUENCY_ERROR from host in Hz
float          hostRssi      = 0.0; // Communication RSSI of packet received by the node
float          hostSnr       = 0.0; // received SNR of last packet received by the node
uint32_t       nodeEfe       = 0;   // received ESTIMATED_FREQUENCY_ERROR from node (raw format)
float          nodeEfeHz     = 0.0; // received ESTIMATED_FREQUENCY_ERROR from node in Hz
float          nodeRssi      = 0.0; // Communication RSSI of packet received by the host
float          nodeSnr       = 0.0; // received SNR of last packet received by the host
uint64_t       rtosTimeStampBCastData = 0;


#if STM32WBAUSED
  TaskHandle_t netwConThreadHandler;
#else
  osThreadId netwConThreadHandler;
#endif



/* Private function prototypes -----------------------------------------------*/
uint8_t IdenticalIDs(char *ID1, char *ID2);
#if SENSOR_HOST
  void BroadCastMyID(void);
  void ConfirmPairing(uint32_t ModuleNr);
  void ConfirmDataReceived(uint8_t ModuleNr);
#else
  void AnswerPairing(void);
  void DataToHost(void);
#endif

void NetConThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartNetConThread, "NetConThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityRealtime, &netwConThreadHandler) != pdPASS)
  {
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_network_connect] [NetConThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
#else
  osThreadDef(NetConThread, StartNetConThread, osPriorityHigh, 0, 128);
  netwConThreadHandler = osThreadCreate(osThread(NetConThread), NULL);
#endif
}

void StartNetConThread(const void * params)
{
#if PRINTF_APP_NETWORK_CONNECT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_network_connect] [NetConThread] Started.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  uint32_t   whileIterations   = 0;
  uint32_t   notificationValue = 0;                   // Used to identify where the notification is coming from.
  uint32_t   delayRtcStartup   = 0;
  uint8_t    printExtraInfo    = 0;
  uint8_t    spreadingFactor   = 0;
  uint8_t    bandwidth         = 0;
  uint8_t    codingRate        = 0;
  uint8_t    bleChannelNr      = 0;
#if ANTENNARSSITEST
#if SENSOR_HOST
  float      hostEfeHzMean     = 0.0; // mean value of received ESTIMATED_FREQUENCY_ERROR from host in Hz
  float      hostRssiMean      = 0.0; // mean value of Communication RSSI of packet received by the node
  float      hostSnrMean       = 0.0; // mean value of received SNR of last packet received by the node
  float      nodeEfeHzMean     = 0.0; // mean value of received ESTIMATED_FREQUENCY_ERROR from node in Hz
  float      nodeRssiMean      = 0.0; // mean value of Communication RSSI of packet received by the host
  float      nodeSnrMean       = 0.0; // mean value of received SNR of last packet received by the host
  uint8_t    emptyModuleTable  = 0;
#endif
#endif
  uint8_t    exchangeIteration = 1;   // the number of host-node iterations with confirmation message (meaning that communication iteration is done)                                      // if this is 10, then a new round of settings can be used.
  uint8_t    exchangeTrial     = 0;   // the number of host-node trial communications. Each trial, the exchangeIteration number is copied. If after 5 trials the
                                      // exchangeIteration number is not changed, then it means that the communication will not work and we go to the next settings

  spreadingFactor   = LORA_SF10;
  bandwidth         = LORA_BW_0800;
  codingRate        = LORA_CR_4_8;
  bleChannelNr      = 0;

  PCF2131_time_t rtcTimeStampMessageReceived;

#if !SENSOR_HOST
  struct tm      *gnss_tmStructure;
  PCF2131_time_t  gnss_time;
  PCF2131_time_t  rtcTimeBeforeChange;
  uint64_t        timeStampSetRTC1       = 0;
  uint64_t        timeStampStartRTC1     = 0;
  uint32_t        timeSetError1          = 0;
  uint64_t        timeStampSetRTC2       = 0;
  uint64_t        timeStampStartRTC2     = 0;
  uint32_t        timeSetError2          = 0;
  int32_t         timeDifferenceNodeHost = 0;
  uint64_t        received_epoch_check   = 0;
  uint32_t        recSubSecondsCheck     = 0;
#endif

  TickType_t xLastWakeTimeNetCon;
  BaseType_t xWasDelayedNetCon;
  const TickType_t xFrequencyNetCon = 10000;


#if SENSOR_HOST
  xLastWakeTimeNetCon = xTaskGetTickCount() - 500U; // -200 to compensate for the start-up of the radio
#else
  xLastWakeTimeNetCon = xTaskGetTickCount();
  TickType_t timeToListenToBroadcast = pdMS_TO_TICKS(11000); // initial time to listen for a broadcast message from a host. Once the host is known, listen for 3s
#endif

  for(;;)
  { // Infinite loop

//#if PRINTF_APP_NETWORK_CONNECT
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [network_connect] Start new iteration in endless loop.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
	// wake up radio:
	// pairingOngoing = 1; // 20240623 pairingOngoing is only changed to 1 in app_rtc_sync
	SX1280GetStatus();

#if SENSOR_HOST
	SetTypicalRegisterSettings(LORA, 20, spreadingFactor, bandwidth, codingRate, bleChannelNr);
    // Set DIO interrupt request parameters
//    txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
//    txBuf[1] = 0x01;  //irqMask   7:0  enable TxDone interrupt
//    txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
//    txBuf[3] = 0x01;  //dio1Mask  7:0  map TXDone to DIO1
//    txBuf[4] = 0x00;  //dio2Mask 15:8
//    txBuf[5] = 0x00;  //dio2Mask  7:0
//    txBuf[6] = 0x00;  //dio3Mask 15:8
//    txBuf[7] = 0x00;  //dio3Mask  7:0
//    SX1280HalWriteCommand(RADIO_SET_DIOIRQPARAMS, txBuf, 8); //0x8D

	whileIterations   = 0;
    xTaskNotifyStateClear(netwConThreadHandler);
    notificationValue = 0;
    while ((notificationValue & NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST) != NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST)
    { // waiting for a notification value from the GNSSPPS to align the start of the endless loop
      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2000));
      if ((notificationValue & NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST) == NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST)
      {
//#if PRINTF_APP_NETWORK_CONNECT
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [network_connect] Notification from GNSSPP10S #%u received.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps);
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
        //xLastWakeTimeNetCon = xTaskGetTickCount() - 600; // -200 to compensate for the start-up of the radio
      }
      else
      {
        if (!notificationValue)
        {
#if PRINTF_APP_NETWORK_CONNECT
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [network_connect] No notification from GNSSPP10S in the last 2s to start broadcast.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        else
        {
#if PRINTF_APP_NETWORK_CONNECT
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [network_connect] Wrong notification value received to start broadcast: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        xTaskNotifyStateClear(netwConThreadHandler);
        notificationValue = 0;
      }
      if (whileIterations++ > 3)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_led] Error: Jump out of while loop waiting for GNSSPP10S to start broadcasting.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
        notificationValue = NOTIFICATION_FROM_GNSSPP10S_TO_START_BROADCAST;
        // todo: check what the error can be
      }
    }
#endif
//    HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_SET);   // Keep Supercap on during this process to avoid extra waiting time afterwards

    // Parameter settings for SetTypicalRegisterSettings(uint8_t packetType, uint8_t payloadLength, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t bleChannelNr)
    // uint8_t spreadingFactor: SF5 = 0x50, SF12 = 0xC0
    // uint8_t bandwidth: LORA_BW_1600 = 0x0A = 0b000 1010, LORA_BW_0800 = 0x18 = 0b0001 1000, LORA_BW_0400 = 0x26 = 0b0010 0110, LORA_BW_0200 = 0x34 = 0b0011 0100
    // uint8_t codingRate: LORA_CR_4_5 = 0x01, LORA_CR_4_8 = 0x04, LORA_CR_LI_4_5 = 0x05, LORA_CR_LI_4_6 = 0x06, LORA_CR_LI_4_7 = 0x07
    // uint8_t bleChannelNr: from 0 to 39
#if ANTENNARSSITEST
	exchangeTrial++;
	if (exchangeTrial > 50)
	{// either host or node have 6 communication trials which failed: go to the next settings
      exchangeTrial = 0;
	  exchangeIteration = 11;
	}
    if (exchangeIteration > 10)
    {
      exchangeIteration = 1;
      bleChannelNr++;
      if (bleChannelNr > 39)
      {
        bleChannelNr = 0;
        codingRate++;
        if (codingRate > 7)
        {
          codingRate = 1;
          spreadingFactor -= 0x10;
          if (spreadingFactor < 0x50)
          {
            spreadingFactor = 0xC0;
            switch (bandwidth)
            {
              case LORA_BW_1600:
            	bandwidth = LORA_BW_0800;
                break;
              case LORA_BW_0800:
            	bandwidth = LORA_BW_0400;
                break;
              case LORA_BW_0400:
            	bandwidth = LORA_BW_0200;
                break;
              case LORA_BW_0200:
            	bandwidth = LORA_BW_1600;
            }
          }
        }
      }
    }
//#if PRINTF_APP_NETWORK_CONNECT
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Exchange trial #%u, exchange iteration #%u.\r\n",(unsigned int) xTaskGetTickCount(), exchangeTrial, exchangeIteration);
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
#endif

#if SENSOR_HOST
    // Broadcast my ID with current RTC time to any node waiting to be paired

//20240622    SetTypicalRegisterSettings(LORA, 20, spreadingFactor, bandwidth, codingRate, bleChannelNr);
//***********************
    BroadCastMyID();
//***********************

    timeStamp5 = startOfTransmit;  // timeStamp5 is RxStart from the host
    // Listen after sending the broadcasted pairing message to see if someone wants to pair.
    SetTypicalRegisterSettings(LORA, 41, spreadingFactor, bandwidth, codingRate, bleChannelNr);
    moduleExists = 0;
#if STM32WBAUSED
    HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
//    SX1280WaitOnBusy();
#else
    HAL_GPIO_WritePin(NSS_CTS_GPIO_Port, NSS_CTS_Pin, GPIO_PIN_RESET);
#endif
    txBuf[0] = RADIO_SET_DIOIRQPARAMS;
    txBuf[1] = 0x40;  //irqMask  15:8  enable RxTxTimeout
    txBuf[2] = 0x02;  //irqMask   7:0  enable RxDone interrupt
    txBuf[3] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
    txBuf[4] = 0x02;  //dio1Mask  7:0  map RXDone to DIO1
    txBuf[5] = 0x00;  //dio2Mask 15:8
    txBuf[6] = 0x00;  //dio2Mask  7:0
    txBuf[7] = 0x00;  //dio3Mask 15:8
    txBuf[8] = 0x00;  //dio3Mask  7:0
    HAL_SPI_Transmit(&hspi1, (uint8_t *)txBuf, 9, 100);
#if STM32WBAUSED
    HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
//    SX1280WaitOnBusy();
//20240512    while(HAL_GPIO_ReadPin(BUSY_SX1280_GPIO_Port, BUSY_SX1280_Pin) == 1)
#else
    HAL_GPIO_WritePin(NSS_CTS_GPIO_Port, NSS_CTS_Pin, GPIO_PIN_SET);
    while(HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == 1)
#endif
    {
#if PRINTF_APP_NETWORK_CONNECT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_network_connect] Radio busy for DIOIRQPARAMS setting.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    ClearIrqStatus(IRQ_RADIO_ALL);//clear the interrupt //not working correctly?
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [network_connect] Start listening to search for a new device. RTC PID controller is skipped during this time.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
    notificationValue = 0;
    xTaskNotifyStateClear(netwConThreadHandler);

//***********************
    SetRx(); // Sensor Host to receive pairing Answer Command from a sensor node
//***********************
    // buffer to be received by host from Sensor Node:
    // rxBuf[ 0] = STARTDELIMITER;        // "S" Start Delimiter
    // rxBuf[ 1] = moduleTable[0].ID[0];  // Node ID LSB
    // rxBuf[ 2] = moduleTable[0].ID[1];
    // rxBuf[ 3] = moduleTable[0].ID[2];
    // rxBuf[ 4] = moduleTable[0].ID[3];
    // rxBuf[ 5] = moduleTable[0].ID[4];
    // rxBuf[ 6] = moduleTable[0].ID[5];
    // rxBuf[ 7] = moduleTable[0].ID[6];
    // rxBuf[ 8] = moduleTable[0].ID[7];  // Node ID MSB
    // rxBuf[ 9] = moduleTable[1].ID[0];  // Host ID LSB
    // rxBuf[10] = moduleTable[1].ID[1];
    // rxBuf[11] = moduleTable[1].ID[2];
    // rxBuf[12] = moduleTable[1].ID[3];
    // rxBuf[13] = moduleTable[1].ID[4];
    // rxBuf[14] = moduleTable[1].ID[5];
    // rxBuf[15] = moduleTable[1].ID[6];
    // rxBuf[16] = moduleTable[1].ID[7];  // Host ID MSB
    // rxBuf[17] = rawEfeBuf[1];          // received ESTIMATED_FREQUENCY_ERROR MSB
    // rxBuf[18] = rawEfeBuf[2];
    // rxBuf[19] = rawEfeBuf[3];          // received ESTIMATED_FREQUENCY_ERROR LSB
    // rxBuf[20] = packetStatusBuf[1];    // rssiSync: Communication RSSI of packet received by the node. Actual signal power is -(rssiSync)/2 dBm
    // rxBuf[21] = packetStatusBuf[2];    // snr: estimation of SNR of last package received. In two's complement format x 4. Actual SNR is (snr)/4 dB
    // rxBuf[22] = NODEIAMHERE;           // "Y" pairing ANSWER command, "Yes, I'm here!"
    // rxBuf[23] = (uint8_t)CheckSumCalc; // Check Sum
    // rxBuf[24] = ENDDELIMITER;          // "E" Message End delimiter
    //
    //  example:
    //
    //  5380F3A30A07391D0081F3A30A073C1C000000000000590F45
    //
    if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(3000)))  // Listen maximum 3s.
    {// Received message with no time-out, DIOx received
      xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
      timeStamp4  = timeStampInt;
      xSemaphoreGive(timeStampIntMutex);
      // take time stamp Lh(t5):
      while (!pcf2131_is_running())
      {
        pcf2131_start_clock();
#if PRINTF_APP_NETWORK_CONNECT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at reading Lh(t5).\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
        vTaskDelay(20);
      }

      pcf2131_get_time(&rtc_time);
      // Calculate Epoch:
      rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
      rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
      rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
      rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
      rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
      rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
      rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
      // calculate epoch, mktime will also calculate tm_wday and tm_yday.
      // epoch = #s elapsed since 1/1/1970
      // mktime consumes 5.46KB of FASH memory!!
      rtc_epoch = mktime(&rtc_tmStructure);
      xEpochLht5 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds - 50;

      if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
      { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
        ClearIrqStatus(IRQ_RADIO_ALL);//clear the interrupt
//#if PRINTF_APP_NETWORK_CONNECT
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_network_connect] RxDone.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//
        ReadRXBuffer(0, rxBuf, 41);
//#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_network_connect] Payload received: ",(unsigned int) xTaskGetTickCount());
        for (unsigned int i = 0; i < 41; i++)
        {
          npf_snprintf(byteString, 3, "%02X", rxBuf[i]);
          strcat(uart_buf, byteString);
        }
        strcat(uart_buf, ".\n");
        huart2print(uart_buf, strlen(uart_buf));
#endif
//#endif
        // First sanity check of received message:
        if ((rxBuf[0] == STARTDELIMITER) && (rxBuf[22] == NODEIAMHERE) && (rxBuf[40] == ENDDELIMITER))
        {
          CheckSumCalc = 0;
          for (unsigned int i = 0; i < 39; i++)
          {
            CheckSumCalc = (CheckSumCalc ^ rxBuf[i]);
          }
          if (rxBuf[39] == CheckSumCalc)
          { // received message is genuine
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_network_connect] Genuine message received.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
            for (unsigned int i = 1; i < 9; i++)
            {
              receivedSensorID[i-1] = rxBuf[i];
              receivedTargetID[i-1] = rxBuf[i+8];
            }
            // remove status bits from ID's:
            receivedSensorID[0] &= 0b10000001;
            receivedTargetID[0] &= 0b10000001;
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_network_connect] Received Sensor Unique ID: ",(unsigned int) xTaskGetTickCount());
            for (unsigned int i = 0; i < 8; i++)
            {
              npf_snprintf(byteString, 3, "%02X", receivedSensorID[i]);
              strcat(uart_buf, byteString);
            }
            strcat(uart_buf, ".\n");
            huart2print(uart_buf, strlen(uart_buf));
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_network_connect] Received Target Unique ID: ",(unsigned int) xTaskGetTickCount());
            for (unsigned int i = 0; i < 8; i++)
            {
              npf_snprintf(byteString, 3, "%02X", receivedTargetID[i]);
              strcat(uart_buf, byteString);
            }
            strcat(uart_buf, ".\n");
            huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
            if (IdenticalIDs(receivedTargetID, moduleTable[0].ID))
            { // message for me: it is either first connection from a node, connected node or from another host who is specifically addressing me
              //read received packet status register for the RSSI byte
              SX1280HalReadCommand(RADIO_GET_PACKETSTATUS, packetStatusBuf, 6);
              // packetStatusBuf[0] = status?
              // packetStatusBuf[1] = packetStatus[ 7: 0] = rssiSync: RSSI value latched upon the detection of the sync address. Actual signal power is -(rssiSync)/2 dBm
              // packetStatusBuf[2] = packetStatus[15: 8] = snr: estimation of SNR of last package received. In two's complement format x 4. Actual SNR is (snr)/4 dB
              // packetStatusBuf[3] = packetStatus[23:16] not used in Lora
              // packetStatusBuf[4] = packetStatus[31:24] not used in Lora
              // packetStatusBuf[5] = packetStatus[39:32] not used in Lora
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_network_connect] Packet Status: ",(unsigned int) xTaskGetTickCount());
              for (unsigned int i = 0; i < 6; i++)
              {
                npf_snprintf(byteString, 3, "%02X", packetStatusBuf[i]);
                strcat(uart_buf, byteString);
              }
              strcat(uart_buf, ".\n");
              huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
              nodeRssi = (float) - (packetStatusBuf[1] / 2); // Communication RSSI of packet received by the host
              nodeSnr  = (float) complement2(packetStatusBuf[2], 8) / 4;
              //todo: not used with this method of ranging: investigate more in detail!!!
              // Lora Frequency error indicator (FEI), this is only reliable for positive SNR
              ReadRegister_16bAddress(REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, rawEfeBuf, 4);
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_network_connect] Raw estimated frequency error: ",(unsigned int) xTaskGetTickCount());
              for (unsigned int i = 0; i < 4; i++)
              {
                npf_snprintf(byteString, 3, "%02X", rawEfeBuf[i]);
                strcat(uart_buf, byteString);
              }
              strcat(uart_buf, ".\n");
              huart2print(uart_buf, strlen(uart_buf));
#endif
#endif

              nodeEfe = (rawEfeBuf[1] << 16) | (rawEfeBuf[2] << 8) | rawEfeBuf[3];
              nodeEfe &= 0x000FFFFF;
              nodeEfeHz = 1.55 * (float) complement2(hostEfe, 20) / (1600.0 / 1625.0);
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_network_connect] RSSI = %6.2fdBm, efeHz = %6.2fHz, SNR = %6.2fdB.\r\n",(unsigned int) xTaskGetTickCount(), hostRssi, hostEfeHz, hostSnr);
              huart2print(uart_buf, strlen(uart_buf));
#endif
#endif

              // Frequency error of former radio exchange received by the node and send back to me:
              hostEfe = (rxBuf[17] << 16) | (rxBuf[18] << 8) | rxBuf[19];
              hostEfe &= 0x000FFFFF;
              hostEfeHz = 1.55 * (float) complement2(hostEfe, 20) / (1600.0 / 1625.0);
              // Communication RSSI of former radio exchange received by the node and send back to the host:
              hostRssi = (float) -(rxBuf[20] / 2);
              // estimated SNR of former radio exchange received by the node and send back to the host:
              hostSnr  = (float) complement2(rxBuf[21], 8) / 4;
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_network_connect] Received from Node: RSSI = %6.2fdBm, efeHz = %6.2fHz, SNR = %6.2fdB.\r\n",(unsigned int) xTaskGetTickCount(), hostRssi, hostEfeHz, hostSnr);
              huart2print(uart_buf, strlen(uart_buf));
#endif
#endif

              for (unsigned int i = 1; i < MAXNRMODULES; i++) // moduleTable[0] does not need to be checked
              { // check if new module already exists in the moduleTable
                if (!moduleTable[i].number) // number contains the record number. if this is 0, it means that the record is empty
                { // first empty record, if module has not been found yet, then it can be added as a new module
                  moduleTable[i].number = moduleTable[i-1].number + 1U;
                  moduleTable[i].hostfei    = hostEfeHz;
                  moduleTable[i].hostrssi   = hostRssi;
                  moduleTable[i].hostsnr    = hostSnr;
                  moduleTable[i].nodefei    = nodeEfeHz;
                  moduleTable[i].noderssi   = nodeRssi;
                  moduleTable[i].nodesnr    = nodeSnr;

                  for (unsigned int j = 0; j < 8; j++)
                  { // store node ID to my module table
                    moduleTable[i].ID[j] = receivedSensorID[j];
                  }
                  moduleTable[i].type = receivedSensorID[0] & 1; // is 1 if Host, 0 if Node --> this should be a Node!!
//                  pcf2131_get_time(&rtc_time);
//                  // Calculate Epoch:
//                  rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
//                  rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
//                  rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
//                  rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
//                  rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
//                  rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
//                  rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
//                  // calculate epoch, mktime will also calculate tm_wday and tm_yday.
//                  // epoch = #s elapsed since 1/1/1970
//                  // mktime consumes 5.46KB of FASH memory!!
//                  rtc_epoch = mktime(&rtc_tmStructure);
                  // Host: store the epoch when this new module is being connected to the network (this is time stamp Lh(t5)
                  moduleTable[i].epochConnected = ( time_t) (xEpochLht5 / 100);
                  recSubSeconds                 = (uint8_t) (xEpochLht5 % 100);
#if PRINTF_APP_NETWORK_CONNECT
#if !ANTENNARSSITEST
                  // print out the module table:
                  // Example:
                  //  number |   Full Unique ID   | type |        Connected       | FEI (Hz) | RSSI (dBm) | SNR (dB)
                  //  -------+--------------------+------+------------------------+----------+------------+---------
                  //     1   | 0x80F3A30A073C1C00 | Node | 19/05/2023 14:07:22.50 |    0.00  |     0.00   |   0.00

                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "%u [app_network_connect] New sensor added to my module table.\r\n",(unsigned int) xTaskGetTickCount());
                  huart2print(uart_buf, strlen(uart_buf));
#endif
#if ANTENNARSSITEST
                  if (exchangeIteration > 9)
                  { // print table only once
#endif
                  huart2print(uart_buf, strlen(uart_buf));
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "number |   Full Unique ID   | type |  Started or connected  |  SF  |  BW  |  CR   |Channel| HFEI (Hz) | HRSSI (dBm) | HSNR (dB) | NFEI (Hz) | NRSSI (dBm) | NSNR (dB)\r\n");
                  huart2print(uart_buf, strlen(uart_buf));
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "-------+--------------------+------+------------------------+------|------|-------|-------|-----------+-------------+-----------+-----------+-------------+-----------\r\n");
                  huart2print(uart_buf, strlen(uart_buf));
#if ANTENNARSSITEST
                  hostEfeHzMean = 0.0;
                  hostRssiMean  = 0.0;
                  hostSnrMean   = 0.0;
                  nodeEfeHzMean = 0.0;
                  nodeRssiMean  = 0.0;
                  nodeSnrMean   = 0.0;
#endif
                  for (unsigned int j = 1; j <= i; j++)
                  { // j = 1; j <= i; j++
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, " %05u | 0x",(unsigned int)moduleTable[j].number);
                    for (unsigned int k = 0; k < 8; k++)
                    {
                      npf_snprintf(byteString, 3, "%02X", moduleTable[j].ID[k]);
                      strcat(uart_buf, byteString);
                    }
                    if (moduleTable[j].type)
                    {
                      strcat(uart_buf, " | Host | ");
                    }
                    else
                    {
                      strcat(uart_buf, " | Node | ");
                    }
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    rec_tmStructure = gmtime(&moduleTable[j].epochConnected);
                    npf_snprintf(uart_buf, 200, "%02d/%02d/%02d %02d:%02d:%02d.%02d |  ",
                        rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                        rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    switch (spreadingFactor)
                    {
                      case LORA_SF5:
                        npf_snprintf(uart_buf, 200, " 5  |");
                        break;
                      case LORA_SF6:
                        npf_snprintf(uart_buf, 200, " 6  |");
                        break;
                      case LORA_SF7:
                        npf_snprintf(uart_buf, 200, " 7  |");
                        break;
                      case LORA_SF8:
                        npf_snprintf(uart_buf, 200, " 8  |");
                        break;
                      case LORA_SF9:
                        npf_snprintf(uart_buf, 200, " 9  |");
                        break;
                      case LORA_SF10:
                        npf_snprintf(uart_buf, 200, "10  |");
                        break;
                      case LORA_SF11:
                        npf_snprintf(uart_buf, 200, "11  |");
                        break;
                      case LORA_SF12:
                        npf_snprintf(uart_buf, 200, "12  |");
                    }
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    switch (bandwidth)
                    {
                      case LORA_BW_1600:
                        npf_snprintf(uart_buf, 200, " 1600 |");
                        break;
                      case LORA_BW_0800:
                        npf_snprintf(uart_buf, 200, "  800 |");
                        break;
                      case LORA_BW_0400:
                        npf_snprintf(uart_buf, 200, "  400 |");
                        break;
                      case LORA_BW_0200:
                        npf_snprintf(uart_buf, 200, "  200 |");
                    }
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    switch (codingRate)
                    {
                      case LORA_CR_4_5:
                        npf_snprintf(uart_buf, 200, "  4/5  |");
                        break;
                      case LORA_CR_4_6:
                        npf_snprintf(uart_buf, 200, "  4/6  |");
                        break;
                      case LORA_CR_4_7:
                        npf_snprintf(uart_buf, 200, "  4/7  |");
                        break;
                      case LORA_CR_4_8:
                        npf_snprintf(uart_buf, 200, "  4/8  |");
                        break;
                      case LORA_CR_LI_4_5:
                        npf_snprintf(uart_buf, 200, "LI 4/5 |");
                        break;
                      case LORA_CR_LI_4_6:
                        npf_snprintf(uart_buf, 200, "LI 4/6 |");
                        break;
                      case LORA_CR_LI_4_8:
                        npf_snprintf(uart_buf, 200, "LI 4/7 |");
                        break;
                    }
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "   %02u  |",bleChannelNr);
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "   %07.2f |     %06.2f  |   %06.2f  |  %07.2f  |    %06.2f   |   %06.2f\r\n", moduleTable[j].hostfei, moduleTable[j].hostrssi, moduleTable[j].hostsnr, moduleTable[j].nodefei, moduleTable[j].noderssi, moduleTable[j].nodesnr);
                    huart2print(uart_buf, strlen(uart_buf));

#if ANTENNARSSITEST
                    hostEfeHzMean += moduleTable[j].hostfei;
                    hostRssiMean  += moduleTable[j].hostrssi;
                    hostSnrMean   += moduleTable[j].hostsnr;
                    nodeEfeHzMean += moduleTable[j].nodefei;
                    nodeRssiMean  += moduleTable[j].noderssi;
                    nodeSnrMean   += moduleTable[j].nodesnr;
                    if (j == (MAXNRMODULES - 1))
                    {
                      hostEfeHzMean = hostEfeHzMean / 10.0;
                      hostRssiMean  = hostRssiMean / 10.0;
                      hostSnrMean   = hostSnrMean / 10.0;
                      nodeEfeHzMean /= 10.0;
                      nodeRssiMean  /= 10.0;
                      nodeSnrMean   /= 10.0;
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "-------+--------------------+------+------------------------+------|------|-------|-------|-----------+-------------+-----------+-----------+-------------+-----------\r\n");
                      huart2print(uart_buf, strlen(uart_buf));
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, " Mean values                                                                                  %07.2f |     %06.2f  |   %06.2f  |  %07.2f  |    %06.2f   |   %06.2f\r\n", hostEfeHzMean, hostRssiMean, hostSnrMean, nodeEfeHzMean, nodeRssiMean, nodeSnrMean);
                      huart2print(uart_buf, strlen(uart_buf));
                      // now module table can be emptied:
                      emptyModuleTable = 1;
                    }
#endif
                  } // end of j = 1; j <= i; j++
#if ANTENNARSSITEST
                  } // endif exchangeIteration > 9 (print table only once)
#endif

#endif
                  vTaskDelay(500); // to give the Node time to listen
                  // synchronize the clock further and prepare for the ranging process
                  // Give connection to a host confirmation to the node. If a node does not receive this confirmation, it will look for another host
                  // Information to send back to the node:
                  //   1. sequence number so that the node knows when to send data (this is also an indication how many nodes are connected to this host)
                  //   2. timing information to fine tune the synchronisation process
                  //   3. anything else?
                  SetTypicalRegisterSettings(LORA, 41, spreadingFactor, bandwidth, codingRate, bleChannelNr);
//*******************************************************
                  ConfirmPairing(moduleTable[i].number); // NODEIAMHERE, also used to do first timing synchronization based on Lightweight Tree based synchronization
//*******************************************************
                  //pairingOngoing = 0;
#if ANTENNARSSITEST
                  if (emptyModuleTable)
                  {
                    for (unsigned int l = 1; l < MAXNRMODULES; l++)
                    {
                      moduleTable[l].number = 0;
                    }
                    emptyModuleTable = 0;
                  }
                  exchangeIteration++;
                  exchangeTrial = 0;
#endif
                  // todo
                  i = MAXNRMODULES - 1; // jump out of for..next loop
                } // end of adding a new module
                else
                { // the module table contains already a Node on this record, check if this is the same as the received message
                  if (IdenticalIDs(receivedSensorID, moduleTable[i].ID))
                  { // module is already known, if not, check next record
                    //pairingOngoing = 0;
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "%u [app_network_connect] Known module giving me a message.\r\n",(unsigned int) xTaskGetTickCount());
                    huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
                    // this message is to further connect to the network or to do something special... this is not used to send regular data
                    // todo ...

                    // in this case the message is coming from a node which most probably had a reset. For testing purposes, this node will be deleted from the host table so
                    // that he can be donnected again on the next broad cast message.
                    moduleTable[i].number = 0;
#if ANTENNARSSITEST
                    // fill up the module table again for RSSI test purposes... so just jump to the next position in the module table
#else
                    i = MAXNRMODULES - 1; // jump out of for..next loop
#endif
                  }
                }
              } // end of for loop to run through moduleTable to check if new module already exists
            } // end of message for me (either point to point or broadcast)
            else
            {
              //pairingOngoing = 0;
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [network_connect] AnswerPairing message is not addressed to me.\r\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
            }
          } // end of received message is genuine
          else
          {
            //pairingOngoing = 0;
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [network_connect] Check sum error on AnswerPairing message.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
            // todo
            // What can be done in this case?
          }
        } // end of first sanity check of received message
        else
        {
          //pairingOngoing = 0;
#if PRINTF_APP_NETWORK_CONNECT
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [network_connect] Not a valid AnswerPairing message received.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      } // end of notificationValue == NOTIFICATION_FROM_DIO1
    } // end of received message with no time-out, DIOx received
    else
    {// time-out on Radio Rx, waiting for pairing answer, no DIOx received
      //pairingOngoing = 0;
#if PRINTF_APP_NETWORK_CONNECT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [network_connect] Nothing received within 3s after sending broadcast message.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    GoToStandby(0);
#if SUPERCAPUSED
#if !AUTOMATESUPERCAP
    if (superCapAvailable != 100)
    {
   	  ScapThreadNotify(NOTIFICATION_LOAD_SCAP);     // first make sure that the supercap is loaded
   	  whileIterations   = 0; // to prevent lock into while loop
   	  xTaskNotifyStateClear(netwConThreadHandler);
   	  notificationValue = 0;
   	  while ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
   	  { // waiting for a notification value from the app_supercap that the super capacitor is loaded
   	    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(10000));
   	    if ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) == NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
   	    {
#if PRINTF_APP_NETWORK_CONNECT
   	      waitToPrint();
   	      npf_snprintf(uart_buf, 200, "%u [network_connect] Super capacitor loaded. Ready to start.\r\n",(unsigned int) xTaskGetTickCount());
   	      huart2print(uart_buf, strlen(uart_buf));
#endif
   	    }
   	    else
   	    {
   	      if (!notificationValue)
   	      {
#if PRINTF_APP_NETWORK_CONNECT
   	          waitToPrint();
   	          npf_snprintf(uart_buf, 200, "%u [network_connect] No notification from supercap in the last 10s.\r\n",(unsigned int) xTaskGetTickCount());
   	          huart2print(uart_buf, strlen(uart_buf));
#endif
   	      }
   	      else
   	      {
#if PRINTF_APP_NETWORK_CONNECT
   	          waitToPrint();
   	          npf_snprintf(uart_buf, 200, "%u [network_connect] Wrong notification value received to load supercap: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
   	          huart2print(uart_buf, strlen(uart_buf));
#endif
   	      }
   	      xTaskNotifyStateClear(netwConThreadHandler);
   	      notificationValue = 0;
   	    }
   	    if (whileIterations++ > 10)
   	    {
   	      waitToPrint();
   	      npf_snprintf(uart_buf, 200, "%u [network_connect] Error: Jump out of while loop waiting for a notification from supercap.\r\n",(unsigned int) xTaskGetTickCount());
   	      huart2print(uart_buf, strlen(uart_buf));
   	      notificationValue = NOTIFICATION_FROM_SCAP_SUPERCAP_READY;
   	    }
   	  }
    }
#endif // !AUTOMATESUPERCAP
#endif // SUPERCAPUSED
//    SX1280SetSleep();
    pairingOngoing = 0;
//#if PRINTF_APP_NETWORK_CONNECT
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_network_connect] Pairing process done.\r\n", (unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
//#endif

#if ANTENNARSSITEST
//    vTaskDelayUntil(&xLastWakeTime, 2000U);
#else
//    vTaskDelayUntil(&xLastWakeTimeNetCon, 10000U);
	// Wait for the next cycle.
    xWasDelayedNetCon = xTaskDelayUntil(&xLastWakeTimeNetCon, xFrequencyNetCon);

#endif
#else // SENSOR_NODE

//#if PRINTF_APP_NETWORK_CONNECT
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [network_connect] Start new iteration in endless loop for node.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    // wake up radio:
//    SX1280GetStatus();
//
//    spreadingFactor   = LORA_SF8;
//    bandwidth         = LORA_BW_0800;
//    codingRate        = LORA_CR_4_8;
//    bleChannelNr      = 5;

    // settings for gateway test
//    spreadingFactor   = LORA_SF12;
//    bandwidth         = LORA_BW_0800;
//    codingRate        = LORA_CR_LI_4_8;
//    bleChannelNr      = 2;

#if PLANTSENSOR
	// for test gateway:
    SetTypicalRegisterSettings(LORA, 10, spreadingFactor, bandwidth, codingRate, bleChannelNr);
#else
    SetTypicalRegisterSettings(LORA, 20, spreadingFactor, bandwidth, codingRate, bleChannelNr);
#endif
    moduleExists = 0;
    txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
    txBuf[1] = 0x02;  //irqMask   7:0  enable RxDone interrupt
    txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
    txBuf[3] = 0x02;  //dio1Mask  7:0  map RXDone to DIO1
    txBuf[4] = 0x00;  //dio2Mask 15:8
    txBuf[5] = 0x00;  //dio2Mask  7:0
    txBuf[6] = 0x00;  //dio3Mask 15:8
    txBuf[7] = 0x00;  //dio3Mask  7:0
    SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); // 0x8D
    ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt
//    HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin,  GPIO_PIN_SET);   // Keep Supercap on during listening process to avoid waiting time afterwards
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[network_connect]   Listening starts at %ums -> Searching for a message from a host.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
    notificationValue = 0;
    xTaskNotifyStateClear(netwConThreadHandler);
//***********************
    SetRx(); // Sensor Node to receive pairing command from a Host
//***********************
    // rxBuf[ 0] = STARTDELIMITER;           // "S" Start Delimiter
    // rxBuf[ 1] = moduleTable[1].ID[0];     // HostID LSB
    // rxBuf[ 2] = moduleTable[1].ID[1];
    // rxBuf[ 3] = moduleTable[1].ID[2];
    // rxBuf[ 4] = moduleTable[1].ID[3];
    // rxBuf[ 5] = moduleTable[1].ID[4];
    // rxBuf[ 6] = moduleTable[1].ID[5];
    // rxBuf[ 7] = moduleTable[1].ID[6];
    // rxBuf[ 8] = moduleTable[1].ID[7];     // HostID MSB
    // rxBuf[ 9] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch LSB
    // rxBuf[10] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch 2nd Byte
    // rxBuf[11] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch 3rd Byte
    // rxBuf[12] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch 4th Byte
    // rxBuf[13] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch 5th Byte
    // rxBuf[14] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch 6th Byte
    // rxBuf[15] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch 7th Byte
    // rxBuf[16] = (uint8_t)(xEpoch & 0xFF); // Host xEpoch MSB
    // rxBuf[17] = HOSTISTHEREANYONE;        // "P" pairing command, "Hello, is there anyone?"
    // rxBuf[18] = (uint8_t)CheckSumCalc;    // check sum
    // rxBuf[19] = ENDDELIMITER;             // "E" Message End delimiter
    if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, timeToListenToBroadcast)) // Listen maximum 11s if no host is known yet.
    {// Received message with no time-out, DIOx received
      xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
      timeStamp8  = timeStampInt;
      rtosTimeStampBCastDone = timeStampInt;
      xSemaphoreGive(timeStampIntMutex);
      // take RTC time stamp
//      pcf2131_get_time(&rtcTimeStampMessageReceived);

      if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
      { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
        ClearIrqStatus(IRQ_RADIO_ALL);//clear the interrupt //not working correctly?
//#if PRINTF_APP_NETWORK_CONNECT
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_network_connect] RxDone.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//
#if PLANTSENSOR
        // for gateway test only 14 bytes:
        ReadRXBuffer(0, rxBuf, 10);
#else
        ReadRXBuffer(0, rxBuf, 20);
#endif
        GoToStandby(0); //20240423 no need for the radio to stay switched on...
#if PRINTF_APP_NETWORK_CONNECT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [network_connect] Message received. Payload: ",(unsigned int) timeStamp8);
        // for gateway test only 14 bytes:
#if PLANTSENSOR
        // for gateway test only 14 bytes:
        for (unsigned int i = 0; i < 10; i++)
#else
        for (unsigned int i = 0; i < 20; i++)
#endif
        {
          npf_snprintf(byteString, 3, "%02X", rxBuf[i]);
          strcat(uart_buf, byteString);
        }
        strcat(uart_buf, ".\n");
        huart2print(uart_buf, strlen(uart_buf));
#endif
        // First sanity check of received message:
        if ((rxBuf[0] == STARTDELIMITER) && (rxBuf[17] == HOSTISTHEREANYONE) && (rxBuf[19] == ENDDELIMITER) && (rxBuf[1] & 1))
        {
          CheckSumCalc = 0;
          for (uint8_t i = 0; i < 18; i++)
          {
            CheckSumCalc = (CheckSumCalc ^ rxBuf[i]);
          }
          if (rxBuf[18] == CheckSumCalc)
          { // received message is genuine
//#if PRINTF_APP_NETWORK_CONNECT
//            waitToPrint();
//            npf_snprintf(uart_buf, 200, "%u [network_connect] Checksum done, genuine message received.\r\n",(unsigned int) xTaskGetTickCount());
//            huart2print(uart_buf, strlen(uart_buf));
//#endif
            for (unsigned int i = 1; i < 9; i++)
            {
              receivedSensorID[i-1] = rxBuf[i];
            }
            // remove status bits from ID's:
            receivedSensorID[0] &= 0b10000001;
            //read received packet status register for the RSSI byte
            SX1280HalReadCommand(RADIO_GET_PACKETSTATUS, packetStatusBuf, 6);
            // packetStatusBuf[0] = status?
            // packetStatusBuf[1] = packetStatus[ 7: 0] = rssiSync: RSSI value latched upon the detection of the sync address. Actual signal power is -(rssiSync)/2 dBm
            // packetStatusBuf[2] = packetStatus[15: 8] = snr: estimation of SNR of last package received. In two's complement format x 4. Actual SNR is (snr)/4 dB
            // packetStatusBuf[3] = packetStatus[23:16] not used in Lora
            // packetStatusBuf[4] = packetStatus[31:24] not used in Lora
            // packetStatusBuf[5] = packetStatus[39:32] not used in Lora
//#if PRINTF_APP_NETWORK_CONNECT
//            waitToPrint();
//            npf_snprintf(uart_buf, 200, "%u [app_network_connect] Packet Status: ",(unsigned int) xTaskGetTickCount());
//            for (unsigned int i = 0; i < 6; i++)
//            {
//              npf_snprintf(byteString, 3, "%02X", packetStatusBuf[i]);
//              strcat(uart_buf, byteString);
//            }
//            strcat(uart_buf, ".\n");
//            huart2print(uart_buf, strlen(uart_buf));
//#endif
            hostRssi = (float) - (packetStatusBuf[1] / 2); // Communication RSSI of packet received by the node
            hostSnr  = (float) complement2(packetStatusBuf[2], 8) / 4;
            //todo: not used with this method of ranging: investigate more in detail!!!
            // Lora Frequency error indicator (FEI), this is only reliable for positive SNR
            ReadRegister_16bAddress(REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, rawEfeBuf, 4);
//#if PRINTF_APP_NETWORK_CONNECT
//            waitToPrint();
//            npf_snprintf(uart_buf, 200, "%u [app_network_connect] Raw estimated frequency error: ",(unsigned int) xTaskGetTickCount());
//            for (unsigned int i = 0; i < 4; i++)
//            {
//              npf_snprintf(byteString, 3, "%02X", rawEfeBuf[i]);
//              strcat(uart_buf, byteString);
//            }
//            strcat(uart_buf, ".\n");
//            huart2print(uart_buf, strlen(uart_buf));
//#endif
            hostEfe = (rawEfeBuf[1] << 16) | (rawEfeBuf[2] << 8) | rawEfeBuf[3];
            hostEfe &= 0x000FFFFF;
            hostEfeHz = 1.55 * (float) complement2(hostEfe, 20) / (1600.0 / 1625.0);
#if PRINTF_APP_NETWORK_CONNECT
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [network_connect] Packet status received message RSSI = %6.2fdBm, efeHz = %6.2fHz, SNR = %6.2fdB.\r\n",(unsigned int) xTaskGetTickCount(), hostRssi, hostEfeHz, hostSnr);
            huart2print(uart_buf, strlen(uart_buf));
#endif
            // message for me (either first connection to a host, connected host with timing synchronization info or new host)
            if (!moduleTable[1].sequence) // Sequence contains the sequence number assigned by the Host with the confirmation message.
            { // If this is 0, it means that this Node is not connected yet to a Host (or last attempt failed).
              moduleTable[1].number = 1;
              for (unsigned int i = 0; i < 8; i++)
              { // store host ID to my host ID
                moduleTable[1].ID[i] = receivedSensorID[i];
              }
              moduleTable[1].type = 1;
              xEpoch = (uint64_t)(((((((rxBuf[16] << 8) | rxBuf[15]) << 8 | rxBuf[14]) << 8 | rxBuf[13]) << 8 | rxBuf[12]) << 8 | rxBuf[11]) << 8 | rxBuf[10]) << 8 | rxBuf[9];

              // take time stamp at the start of the RTC setting:
              timeStampSetRTC1 = (uint64_t) xTaskGetTickCount();

              moduleTable[1].epochConnected = (time_t) (xEpoch / 100);
              recSubSeconds = (uint8_t) (xEpoch % 100);
//              moduleTable[1].epochConnected++; // add one second
              // Start RTC with received date/timing info:
//              xSemaphoreTake(recEpochMutex, pdMS_TO_TICKS(400));
//              received_epoch = moduleTable[1].epochConnected;
//              xSemaphoreGive(recEpochMutex);

              // notify app_hal_pps that the rtc needs to set
//              app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC);
              // in stead of notifying app_rtc, the rtc is being set/started here:
              gnss_tmStructure = gmtime(&moduleTable[1].epochConnected);
              gnss_time.year          = (uint16_t)(gnss_tmStructure->tm_year - 100); // only year, no century: 0-99 (tm_year is since 1900)
              gnss_time.month         = (uint8_t) (gnss_tmStructure->tm_mon  + 1);   // 1-12 (tm_mon is 0-11)
              gnss_time.dayOfMonth    = (uint8_t)  gnss_tmStructure->tm_mday;        // 1-31
              gnss_time.dayOfWeek     = (uint8_t)  gnss_tmStructure->tm_wday;        // Sunday = 0 - 6
              gnss_time.hours         = (uint8_t)  gnss_tmStructure->tm_hour;
              gnss_time.minutes       = (uint8_t)  gnss_tmStructure->tm_min;
              gnss_time.seconds       = (uint8_t)  gnss_tmStructure->tm_sec;
              gnss_time.subSeconds    = recSubSeconds;
              gnss_time.mode          = 0;
              pcf2131_set_time(&gnss_time);
//              vTaskDelay(1000);
              // notify app_rtc that the rtc can be started:
//              app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC);
              pcf2131_start_clock();
              // check if the RTC is running and if not, start the RTC! (this should never happen as it will not be in sync anymore)
              while (!pcf2131_is_running())
              {
        	// todo: if this happens than the clock is not correctly set
                pcf2131_start_clock();
#if PRINTF_APP_NETWORK_CONNECT
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at the start of a new RTC sync! Try to start the clock again.\n",(unsigned int) xTaskGetTickCount());
                huart2print(uart_buf, strlen(uart_buf));
#endif
                vTaskDelay(20);
              }
//              compensateTicks = (float) (recSubSeconds * 10);
              // take time stamp at the start of the RTC:
              timeStampStartRTC1 = (uint64_t) xTaskGetTickCount(); // this is the time stamp when the RTC started...
              // take actual time of RTC to check:
              pcf2131_get_time(&rtc_time);
              // calculate the time that this RTC set/start takes, this is an extra error for the correct setting of the RTC to take into account when
              // we set/start the clock for the second time including the exact offset between host and node:
              timeSetError1 = timeStampStartRTC1 - timeStampSetRTC1;
              // now print the data to check what happened
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [network_connect] Received Host Date: %d/%d/%d, UTC Time: %d:%d:%d.%d. TS of start RTC = %ums, TS RTC started = %ums.\r\n", (unsigned int) xTaskGetTickCount(),
                  gnss_tmStructure->tm_mday, gnss_tmStructure->tm_mon + 1, gnss_tmStructure->tm_year + 1900,
                  gnss_tmStructure->tm_hour, gnss_tmStructure->tm_min, gnss_tmStructure->tm_sec, (unsigned int)recSubSeconds,
		          (unsigned int)rtosTimeStampStartRTC, (unsigned int)timeStampStartRTC1);
              huart2print(uart_buf, strlen(uart_buf));
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [network_connect]  Current RTC Date: %d/%d/%d, UTC Time: %d:%d:%d.%d, time to set/started RTC = %u.\r\n", (unsigned int) xTaskGetTickCount(),
                   rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds,
		           (unsigned int) timeSetError1);
              huart2print(uart_buf, strlen(uart_buf));

              // Send my ID, ... info back to requested Host to identify my presence to continue the network connect process
              SetTypicalRegisterSettings(LORA, 41, spreadingFactor, bandwidth, codingRate, bleChannelNr);
//******************************
              AnswerPairing(); // NODEIAMHERE, also used to do first timing synchronization based on Lightweight Tree based synchronization
//******************************
              // Now we need confirmation from the Host that the Node has been added to his module table:
#if PRINTF_APP_NETWORK_CONNECT
              waitToPrint();
              npf_snprintf(uart_buf, 200, "[network_connect]   Listening starts at %ums -> to receive Confirmation Message from Host.\r\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
#endif
              SetTypicalRegisterSettings(LORA, 41, spreadingFactor, bandwidth, codingRate, bleChannelNr);
              txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
              txBuf[1] = 0x02;  //irqMask   7:0  enable RxDone interrupt
              txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
              txBuf[3] = 0x02;  //dio1Mask  7:0  map RXDone to DIO1
              txBuf[4] = 0x00;  //dio2Mask 15:8
              txBuf[5] = 0x00;  //dio2Mask  7:0
              txBuf[6] = 0x00;  //dio3Mask 15:8
              txBuf[7] = 0x00;  //dio3Mask  7:0
              SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); // 0x8D
              ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt
              notificationValue = 0;
              xTaskNotifyStateClear(netwConThreadHandler);
//***********************
              SetRx(); // Sensor Node to receive confirmation from the Host
//***********************
              // rxBuf[ 0] = STARTDELIMITER;                  // "S" Start Delimiter
              // rxBuf[ 1] = moduleTable[0].ID[0];            // MyID LSB
              // rxBuf[ 2] = moduleTable[0].ID[1];
              // rxBuf[ 3] = moduleTable[0].ID[2];
              // rxBuf[ 4] = moduleTable[0].ID[3];
              // rxBuf[ 5] = moduleTable[0].ID[4];
              // rxBuf[ 6] = moduleTable[0].ID[5];
              // rxBuf[ 7] = moduleTable[0].ID[6];
              // rxBuf[ 8] = moduleTable[0].ID[7];            // MyID MSB
              // rxBuf[ 9] = moduleTable[ModuleNr].ID[0];     // Node ID LSB
              // rxBuf[10] = moduleTable[ModuleNr].ID[1];
              // rxBuf[11] = moduleTable[ModuleNr].ID[2];
              // rxBuf[12] = moduleTable[ModuleNr].ID[3];
              // rxBuf[13] = moduleTable[ModuleNr].ID[4];
              // rxBuf[14] = moduleTable[ModuleNr].ID[5];
              // rxBuf[15] = moduleTable[ModuleNr].ID[6];
              // rxBuf[16] = moduleTable[ModuleNr].ID[7];      // Node ID MSB
              // rxBuf[17] = CONFIRMPAIRING;                   // "C" Confirm pairing command, "You are my friend now!"
              // rxBuf[18] = (uint8_t)(sequenceNumber & 0xFF); // Sequence Number LSB
              // rxBuf[19] = (uint8_t)(sequenceNumber & 0xFF);
              // rxBuf[20] = (uint8_t)(sequenceNumber & 0xFF);
              // rxBuf[21] = (uint8_t)(sequenceNumber & 0xFF); // Sequence Number MSB
              // rxBuf[22] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 LSB
              // rxBuf[23] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 2nd Byte
              // rxBuf[24] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 3rd Byte
              // rxBuf[25] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 4th Byte
              // rxBuf[26] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 5th Byte
              // rxBuf[27] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 6th Byte
              // rxBuf[28] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 7th Byte
              // rxBuf[29] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 MSB
              // rxBuf[30] = (uint8_t)CheckSumCalc;            // Check Sum
              // rxBuf[31] = ENDDELIMITER;                     // "E" Message End delimiter
              if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(9000) )) // Listen maximum 9s.
              {// Received message with no time-out, DIOx received
                xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
                timeStamp8  = timeStampInt;
                xSemaphoreGive(timeStampIntMutex);

                // take time stamp Ln(t8):

                while (!pcf2131_is_running())
                {
                  pcf2131_start_clock();
#if PRINTF_APP_NETWORK_CONNECT
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at reading Ln(t8).\n",(unsigned int) xTaskGetTickCount());
                  huart2print(uart_buf, strlen(uart_buf));
#endif
                  vTaskDelay(20);
                }


                pcf2131_get_time(&rtc_time);
                // Calculate Epoch:
                rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
                rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
                rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
                rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
                rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
                rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
                rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
                // calculate epoch, mktime will also calculate tm_wday and tm_yday.
                // epoch = #s elapsed since 1/1/1970
                // mktime consumes 5.46KB of FASH memory!!
                rtc_epoch = mktime(&rtc_tmStructure);
                xEpochLnt8 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds;


                if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
                { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
                  ClearIrqStatus(IRQ_RADIO_ALL);//clear the interrupt //not working correctly?
//#if PRINTF_APP_NETWORK_CONNECT
                  // waitToPrint();
                  // npf_snprintf(uart_buf, 200, "%u [app_network_connect] RxDone.\r\n",(unsigned int) xTaskGetTickCount());
                  // huart2print(uart_buf, strlen(uart_buf));
//#endif
                  ReadRXBuffer(0, rxBuf, 41);
#if PRINTF_APP_NETWORK_CONNECT
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "[network_connect]   Confirmation Message received at %ums -> Payload received: ",(unsigned int) timeStamp8);
                  for (unsigned int i = 0; i < 41; i++)
                  {
                    npf_snprintf(byteString, 3, "%02X", rxBuf[i]);
                    strcat(uart_buf, byteString);
                  }
                  strcat(uart_buf, ".\n");
                  huart2print(uart_buf, strlen(uart_buf));
#endif
                  // First sanity check of received message:
                  if ((rxBuf[0] == STARTDELIMITER) && (rxBuf[17] == CONFIRMPAIRING) && (rxBuf[40] == ENDDELIMITER))
                  {
                    CheckSumCalc = 0;
                    for (uint8_t i = 0; i < 39; i++)
                    {
                      CheckSumCalc = (CheckSumCalc ^ rxBuf[i]);
                    }
                    if (rxBuf[39] == CheckSumCalc)
                    { // received confirmation message is genuine
#if PRINTF_APP_NETWORK_CONNECT
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "[network_connect]         Check done at %ums -> Genuine Confirmation message. ",(unsigned int) xTaskGetTickCount());
                      huart2print(uart_buf, strlen(uart_buf));
#endif
                      for (unsigned int i = 1; i < 9; i++)
                      {
                        receivedSensorID[i-1] = rxBuf[i];
                        receivedTargetID[i-1] = rxBuf[i+8];
                      }
                      // remove status bits from ID's:
                      receivedSensorID[0] &= 0b10000001;
                      receivedTargetID[0] &= 0b10000001;
#if PRINTF_APP_NETWORK_CONNECT
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "Received Host ID: ");
                      for (unsigned int i = 0; i < 8; i++)
                      {
                        npf_snprintf(byteString, 3, "%02X", receivedSensorID[i]);
                        strcat(uart_buf, byteString);
                      }
                      strcat(uart_buf, ", ");
                      huart2print(uart_buf, strlen(uart_buf));
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "Received Target ID: ");
                      for (unsigned int i = 0; i < 8; i++)
                      {
                        npf_snprintf(byteString, 3, "%02X", receivedTargetID[i]);
                        strcat(uart_buf, byteString);
                      }
                      strcat(uart_buf, ".\n");
                      huart2print(uart_buf, strlen(uart_buf));
#endif
                      if (IdenticalIDs(receivedTargetID, moduleTable[0].ID) && IdenticalIDs(receivedSensorID, moduleTable[1].ID))
                      { // message for me: Host is confirming that I'm added to the host module table
//                    	exchangeIteration++;
//                    	exchangeTrial = 0;
//                        // now module table can be emptied:
//                    	if (exchangeIteration > 10)
//                    	{
//                          for (unsigned int l = 1; l < MAXNRMODULES; l++)
//                          {
//                            moduleTable[l].number = 0;
//                          }
//                    	}
                    	// receiving synchronization info:

                    	xEpochLht5 = (uint64_t)(((((((rxBuf[30] << 8) | rxBuf[29]) << 8 | rxBuf[28]) << 8 | rxBuf[27]) << 8 | rxBuf[26]) << 8 | rxBuf[25]) << 8 | rxBuf[24]) << 8 | rxBuf[23];
                    	xEpochLht6 = (uint64_t)(((((((rxBuf[38] << 8) | rxBuf[37]) << 8 | rxBuf[36]) << 8 | rxBuf[35]) << 8 | rxBuf[34]) << 8 | rxBuf[33]) << 8 | rxBuf[32]) << 8 | rxBuf[31];

                    	xEpochOffset = (int32_t) ((((int64_t)xEpochLnt1 - (int64_t)xEpochLht5) + ((int64_t)xEpochLnt8 - (int64_t)xEpochLht6))/2);

                    	// take time stamp at the start of the RTC setting:
                        timeStampSetRTC2 = (uint64_t) xTaskGetTickCount();

                        pcf2131_get_time(&rtc_time);
                        // Calculate Epoch:
                        rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
                        rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
                        rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
                        rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
                        rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
                        rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
                        rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
                        // calculate epoch, mktime will also calculate tm_wday and tm_yday.
                        // epoch = #s elapsed since 1/1/1970
                        // mktime consumes 5.46KB of FASH memory!!
                        rtc_epoch = mktime(&rtc_tmStructure);
                        rtcTimeBeforeChange = rtc_time;
//                        xEpochLnt8 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds;


//                    	// take the RTC timestamp when the message was received:
//                        rtc_tmStructure.tm_year  = (uint16_t)(rtcTimeStampMessageReceived.year - 1900); // Year - 1900
//                        rtc_tmStructure.tm_mon   = (uint8_t) (rtcTimeStampMessageReceived.month -   1); // Month, where 0 = jan
//                        rtc_tmStructure.tm_mday  = (uint8_t) (rtcTimeStampMessageReceived.dayOfMonth);  // Day of the month
//                        rtc_tmStructure.tm_hour  = (uint8_t)  rtcTimeStampMessageReceived.hours;        // 24 hours format
//                        rtc_tmStructure.tm_min   = (uint8_t)  rtcTimeStampMessageReceived.minutes;
//                        rtc_tmStructure.tm_sec   = (uint8_t)  rtcTimeStampMessageReceived.seconds;      // because the clock will be started on the next PPS
//                        rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
//                        // calculate epoch, mktime will also calculate tm_wday and tm_yday.
//                        // epoch = #s elapsed since 1/1/1970
//                        // mktime consumes 5.46KB of FASH memory!!
//                        rtc_epoch = mktime(&rtc_tmStructure);
//                        rtc_epoch++; // add one second
                        // restart RTC with received offset info:
                        if (xEpochOffset > 0)
                        {
                          xEpoch = (((uint64_t)(rtc_epoch)) * ((uint64_t)100)) + (uint64_t)rtc_time.subSeconds - (uint64_t)xEpochOffset + (uint64_t)(timeSetError1 / 10);// + 2 - 100;
//                          xEpoch = (((uint64_t)(rtc_epoch)) * ((uint64_t)100)) + (uint64_t)rtc_time.subSeconds + (uint64_t)xEpochOffset;
                        }
                        else
                        {
                          xEpochOffset = -xEpochOffset;
                          xEpoch = (((uint64_t)(rtc_epoch)) * ((uint64_t)100)) + (uint64_t)rtc_time.subSeconds + (uint64_t)xEpochOffset + (uint64_t)(timeSetError1 / 10);// + 2 - 100;
//                          xEpoch = (((uint64_t)(rtc_epoch)) * ((uint64_t)100)) + (uint64_t)rtc_time.subSeconds - (uint64_t)xEpochOffset;
                        }
                        // Set RTC with received date/timing info:
                        rtc_epoch     = ( time_t) (xEpoch / 100);
                        recSubSeconds = (uint8_t) (xEpoch % 100);
                        // notify app_hal_pps that the rtc needs to set
          //              app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC);
                        // in stead of notifying app_rtc, the rtc is being set/started here:
                        gnss_tmStructure = gmtime(&rtc_epoch);
                        gnss_time.year          = (uint16_t)(gnss_tmStructure->tm_year - 100); // only year, no century: 0-99 (tm_year is since 1900)
                        gnss_time.month         = (uint8_t) (gnss_tmStructure->tm_mon  + 1);   // 1-12 (tm_mon is 0-11)
                        gnss_time.dayOfMonth    = (uint8_t)  gnss_tmStructure->tm_mday;        // 1-31
                        gnss_time.dayOfWeek     = (uint8_t)  gnss_tmStructure->tm_wday;        // Sunday = 0 - 6
                        gnss_time.hours         = (uint8_t)  gnss_tmStructure->tm_hour;
                        gnss_time.minutes       = (uint8_t)  gnss_tmStructure->tm_min;
                        gnss_time.seconds       = (uint8_t)  gnss_tmStructure->tm_sec;
                        gnss_time.subSeconds    = recSubSeconds;
                        gnss_time.mode          = 0;

                        pcf2131_set_time(&gnss_time);

                        // wait 1s HAL/RTOS time - time between receiving message and coming here
          //              delayRtcStartup = 1010 - (xTaskGetTickCount() - timeStamp8) - recSubSeconds + 86;
          //              delayRtcStartup = 1000; // start the RTC 1000 RTOS ticks = 1s later
                        vTaskDelay((TickType_t) (timeSetError1 % 10));
                        // notify app_rtc that the rtc can be started:
          //              app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC);
                        pcf2131_start_clock();
                        // check if the RTC is running and if not, start the RTC! (this should never happen as it will not be in sync anymore)
                        while (!pcf2131_is_running())
                        {
                          pcf2131_start_clock();
#if PRINTF_APP_NETWORK_CONNECT
                          waitToPrint();
                          npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at the start of a new RTC sync! Start the clock.\n",(unsigned int) xTaskGetTickCount());
                          huart2print(uart_buf, strlen(uart_buf));
                          vTaskDelay(20);
#endif
                        }
                        //xSemaphoreTake(CompTicksMutex, pdMS_TO_TICKS(400));
//                        compensateTicks = (float) (recSubSeconds * 10);
                        //xSemaphoreGive(CompTicksMutex);
                        // print current date/time (after offset correction):

                        // take time stamp at the start of the RTC:
                        timeStampStartRTC2 = (uint64_t) xTaskGetTickCount();
                        // calculate the time that this RTC set/start takes, this is an extra error for the correct setting of the RTC to take into account when
                        // we set/start the clock for the second time including the exact offset between host and node:
                        timeSetError2 = timeStampStartRTC2 - timeStampSetRTC2;


                        pcf2131_get_time(&rtc_time);
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "%u [network_connect]  Current RTC Date: %d/%d/%d, UTC Time: %d:%d:%d.%d, Before change UTC Time: %d:%d:%d.%d, time to set/start RTC = %u.\r\n", (unsigned int) xTaskGetTickCount(),
                             rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds,
                             rtcTimeBeforeChange.hours, rtcTimeBeforeChange.minutes, rtcTimeBeforeChange.seconds, rtcTimeBeforeChange.subSeconds, (unsigned int) timeSetError2);
                        huart2print(uart_buf, strlen(uart_buf));




//                    	pcf2131_get_time(&rtc_time);
//                        // Calculate Epoch:
//                        rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
//                        rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
//                        rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
//                        rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
//                        rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
//                        rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
//                        rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
                        // calculate epoch, mktime will also calculate tm_wday and tm_yday.
                        // epoch = #s elapsed since 1/1/1970
                        // mktime consumes 5.46KB of FASH memory!!
//                        rtc_epoch = mktime(&rtc_tmStructure);
                        // restart RTC with received offset info:
//                        xEpoch = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds + xEpochOffset;
//                        xSemaphoreTake(recEpochMutex, pdMS_TO_TICKS(400));
//                        receivedxEpoch = xEpoch;
//                        xSemaphoreGive(recEpochMutex);
                        // notify app_hal_pps that the rtc needs to set
//                        app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC);
                        // wait 1s HAL/RTOS time - time between receiving message and coming here
//                        delayRtcStartup = 1000; // start the RTC 1000 RTOS ticks = 1s later
//                        vTaskDelay(1000);
                        // notify app_rtc that the rtc can be started:
//                        app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC);

#if PRINTF_APP_NETWORK_CONNECT
                        // Example of what is being printed:
                        // 64438 [network_connect]  Current RTC Date: 26/4/2024, UTC Time: 15:54:18.81, Before change UTC Time: 15:54:18.69, time to set/start RTC = 1.
                        //[network_connect]         Offset at 64443ms -> xEpochLhn1(3910961281cs)-xEpochLht5(3910961306cs)+xEpochLnt8(3910961322cs)-xEpochLht6(3910961321cs) = offset(12cs).
                        // xEpochLnt1 = 3910961281cs, Ln(t1) = 26/04/2024 15:54:18.25
                        // xEpochLnt8 = 3910961322cs, Ln(t8) = 26/04/2024 15:54:18.66
                        // xEpochLht5 = 171414683648.000000cs, EpochLht5 check = 1714146816.000000s, SubSecondsCheck = 50.000000cs, Lh(t5) = 26/04/2024 15:54:18.50
                        // xEpochLht6 = 3910961321cs, Lh(t6) = 26/04/2024 15:54:18.65
                        // Number |   Full Unique ID   | type |  Started or connected  | Sequence | FEI (Hz) | RSSI (dBm) | SNR (dB)
                        // -------+--------------------+------+------------------------+----------+----------+------------+---------
                        //  00000 | 0x0079B94820251400 | Node | 01/01/1970 00:00:00.65 | 00001 |  0000.00 |  00.00 |  00.00
                        //  00001 | 0x0179B94820253700 | Host | 26/04/2024 15:54:18.65 | 00001 |  0000.00 |  00.00 |  00.00
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "[network_connect]         Offset at %ums -> xEpochLhn1(%ucs)-xEpochLht5(%ucs)+xEpochLnt8(%ucs)-xEpochLht6(%ucs) = offset(%dcs).\r\n",
                            (unsigned int) xTaskGetTickCount(), (unsigned int) xEpochLnt1, (unsigned int) xEpochLht5, (unsigned int) xEpochLnt8, (unsigned int) xEpochLht6, (int) xEpochOffset);
                        huart2print(uart_buf, strlen(uart_buf));
                        recSubSeconds = xEpochLnt1 % 100;
                        received_epoch = (time_t) (xEpochLnt1 / 100);
                        rec_tmStructure = gmtime(&received_epoch);
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "xEpochLnt1 = %ucs, Ln(t1) = %02d/%02d/%02d %02d:%02d:%02d.%02u \r\n", (unsigned int) xEpochLnt1,
                            rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                            rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                        huart2print(uart_buf, strlen(uart_buf));
                        recSubSeconds = xEpochLnt8 % 100;
                        received_epoch = (time_t) (xEpochLnt8 / 100);
                        rec_tmStructure = gmtime(&received_epoch);
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "xEpochLnt8 = %ucs, Ln(t8) = %02d/%02d/%02d %02d:%02d:%02d.%02u \r\n", (unsigned int) xEpochLnt8,
                            rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                            rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                        huart2print(uart_buf, strlen(uart_buf));
                        received_epoch_check = (uint64_t) (xEpochLht5 / (uint64_t) 100);
                        recSubSecondsCheck = (uint32_t) (xEpochLht5 - received_epoch_check * 100);
                        recSubSeconds = xEpochLht5 % 100;
                        received_epoch = (time_t) (xEpochLht5 / 100);
                        rec_tmStructure = gmtime(&received_epoch);
                        // how to print a uint64_t (does not work):
                        // xEpochLht5 = %lldcs,                EpochLht5 check = %llds,              SubSecondsCheck = 0cs,         Lh(t5) = -384005990/39/1714146858 00:50:26.04
                        // #include <inttypes.h>
                        // int64_t my_int = 999999999999999999;
                        // printf("%" PRId64 "\n", my_int);
                        // waitToPrint();
                        // npf_snprintf(uart_buf, 200, "xEpochLht5 = %" PRId64 "cs, EpochLht5 check = %" PRId64 "s, SubSecondsCheck = %" PRId32 "cs, Lh(t5) = %02d/%02d/%02d %02d:%02d:%02d.%02u \r\n",
                        //     xEpochLht5, received_epoch_check, recSubSecondsCheck,
                        //     rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                        //     rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                        // huart2print(uart_buf, strlen(uart_buf));
                        // how to print a uint64_t - see also https://stackoverflow.com/questions/9225567/how-to-portably-print-a-int64-t-type-in-c
                        //printf("%ld", (long)my_int);
                        //printf("%lld", (long long)my_int); /* C89 didn't define `long long` */
                        //printf("%f", (double)my_int);
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "xEpochLht5 = %fcs, EpochLht5 check = %fs, SubSecondsCheck = %fcs, Lh(t5) = %02d/%02d/%02d %02d:%02d:%02d.%02u \r\n",
                            (unsigned int) xEpochLht5, (unsigned int) received_epoch_check, (unsigned int) recSubSecondsCheck,
                            rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                            rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                        huart2print(uart_buf, strlen(uart_buf));
                        recSubSeconds  = xEpochLht6 % 100;
                        received_epoch = (time_t) (xEpochLht6 / 100);
                        rec_tmStructure = gmtime(&received_epoch);
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "xEpochLht6 = %ucs, Lh(t6) = %02d/%02d/%02d %02d:%02d:%02d.%02u \r\n", (unsigned int) xEpochLht6,
                            rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                            rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                        huart2print(uart_buf, strlen(uart_buf));
#endif
                        // adding sequence number received from the Host
                        moduleTable[1].sequence = (uint32_t) (((rxBuf[21] << 8 | rxBuf[20]) << 8 | rxBuf[19]) << 8 | rxBuf[18]);
                        //20240421 reset xLastWakeTime:
//                        xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
//                        xLastWakeTimeNetCon = rtosTimeStampBCastDone - 500; // from now on, start to listen 500ms earlier than the received broadcast message from the host
//                        xSemaphoreGive(timeStampIntMutex);
//                        timeToListenToBroadcast = pdMS_TO_TICKS(3000);      // and listen during 3s.
                        // wait a bit so that the RTC has been started
//                        vTaskDelay(100);
//                        printExtraInfo = 1;
#if PRINTF_APP_NETWORK_CONNECT
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "Number |   Full Unique ID   | type |  Started or connected  | Sequence | FEI (Hz) | RSSI (dBm) | SNR (dB)\r\n");
                        huart2print(uart_buf, strlen(uart_buf));
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "-------+--------------------+------+------------------------+----------+----------+------------+---------\r\n");
                        huart2print(uart_buf, strlen(uart_buf));
                        for (unsigned int j = 0; j < 2; j++)
                        {
                          waitToPrint();
                          npf_snprintf(uart_buf, 200, " %05u | 0x",(unsigned int)moduleTable[j].number);
                          for (unsigned int k = 0; k < 8; k++)
                          {
                            npf_snprintf(byteString, 3, "%02X", moduleTable[j].ID[k]);
                            strcat(uart_buf, byteString);
                          }
                          if (moduleTable[j].type)
                          {
                            strcat(uart_buf, " | Host | ");
                          }
                          else
                          {
                            strcat(uart_buf, " | Node | ");
                          }
                          huart2print(uart_buf, strlen(uart_buf));
                          waitToPrint();
                          rec_tmStructure = gmtime(&moduleTable[j].epochConnected);
                          npf_snprintf(uart_buf, 200, "%02d/%02d/%02d %02d:%02d:%02d.%02d | ",
                              rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                              rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                          huart2print(uart_buf, strlen(uart_buf));
                          waitToPrint();
                          npf_snprintf(uart_buf, 200, "%05u |  %07.2f |  %05.2f |  %05.2f\r\n", (unsigned int) moduleTable[1].sequence, moduleTable[j].hostfei, moduleTable[j].hostrssi, moduleTable[j].hostsnr);
                          huart2print(uart_buf, strlen(uart_buf));
                        }
#endif
                      }
                    }
                    else
                    { // checkSumCalc of confirmation message is not correct
#if PRINTF_APP_NETWORK_CONNECT
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "%u [app_network_connect] Check sum of Confirmation Message is not correct.\r\n",(unsigned int) xTaskGetTickCount());
                      huart2print(uart_buf, strlen(uart_buf));
#endif
                      //todo inform host that received confirmation message is not correct
                    }
                  } // end of first sanity check of Confirmation Message
                  else
                  {
#if PRINTF_APP_NETWORK_CONNECT
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "%u [app_network_connect] Received Confirmation Message did not pass first sanity check.\r\n",(unsigned int) xTaskGetTickCount());
                    huart2print(uart_buf, strlen(uart_buf));
#endif
                  }
                } // end of notificationValue == NOTIFICATION_FROM_DIO1
                else
                {
#if PRINTF_APP_NETWORK_CONNECT
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "%u [app_network_connect] Wrong notification value received while waiting for Confirmation Message.\r\n",(unsigned int) xTaskGetTickCount());
                  huart2print(uart_buf, strlen(uart_buf));
#endif
                }
              } // end of Received message with no time-out, DIOx received for Confirmation Message
              else
              {
#if PRINTF_APP_NETWORK_CONNECT
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [app_network_connect] Time out occurred while waiting for Confirmation Message.\r\n",(unsigned int) xTaskGetTickCount());
                huart2print(uart_buf, strlen(uart_buf));
#endif
              }
            }
            else
            { // message is either from my host, or another host
              if (IdenticalIDs(receivedSensorID, moduleTable[1].ID))
              { // this message is from my host
                if (rxBuf[17] == HOSTISTHEREANYONE)
                { // use message for synchronization purposes
        	      app_rtc_sync_notify(NOTIFICATION_FROM_HOST_PP10S);
                }
#if PRINTF_APP_NETWORK_CONNECT
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [app_network_connect] Broadcast message from my Host received.\r\n",(unsigned int) xTaskGetTickCount());
                huart2print(uart_buf, strlen(uart_buf));
#endif
//              if (changeClock)
//              { // change clock with the received epoch
                // Set my RTC with the received epoch
                xEpoch = (uint64_t)(((((((rxBuf[16] << 8) | rxBuf[15]) << 8 | rxBuf[14]) << 8 | rxBuf[13]) << 8 | rxBuf[12]) << 8 | rxBuf[11]) << 8 | rxBuf[10]) << 8 | rxBuf[9];
                pcf2131_get_time(&rtc_time);
                // add TIME_DELAY_TO_START_RTC and 1s, + 10ms for inherent delay for the node
//                xEpoch = xEpoch + 1010;
                xSemaphoreTake(recEpochMutex, pdMS_TO_TICKS(400));
                received_epoch = (time_t)(xEpoch / 100);
                xSemaphoreGive(recEpochMutex);
                recSubSeconds = (uint8_t)(xEpoch % 100);
                rec_tmStructure = gmtime(&received_epoch);
                timeDifferenceNodeHost =   rec_tmStructure->tm_min * 6000 + rec_tmStructure->tm_sec * 100 + recSubSeconds
                                         - rtc_time.minutes        * 6000 - rtc_time.seconds        * 100 - rtc_time.subSeconds;
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [network_connect]         Host Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n", (unsigned int) xTaskGetTickCount(),
                    rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                     rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, recSubSeconds);
                huart2print(uart_buf, strlen(uart_buf));
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [network_connect]  Current RTC Date: %d/%d/%d, UTC Time: %d:%d:%d.%d, time difference = %dcs\r\n", (unsigned int) xTaskGetTickCount(),
                     rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds, (int) timeDifferenceNodeHost);
                huart2print(uart_buf, strlen(uart_buf));
//                  // notify app_hal_pps that the rtc needs to be set
//                  app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC);
//                  // wait 1s HAL/RTOS time - time between receiving message and coming here
//                  delayRtcStartup = 1010 - (xTaskGetTickCount() - timeStamp8) - recSubSeconds + 86;
//                  vTaskDelay(delayRtcStartup);
//                  // notify app_rtc that the rtc can be started:
//                  app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC);
//                  // wait a bit so that the RTC has been started
//                  vTaskDelay(100);
//                  printExtraInfo = 1;

                // for testing purposes, we reply to this received message from my host:
                // wait until the host is ready to listen:
//                vTaskDelay(10);
#if ANTENNARSSITEST
                SetTypicalRegisterSettings(LORA, 25, spreadingFactor, bandwidth, codingRate, bleChannelNr);
                AnswerPairing();
                // Now we need confirmation from the Host that the Node has been added to his module table:
#if PRINTF_APP_NETWORK_CONNECT
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Listen again to receive Confirmation Message from Host.\r\n",(unsigned int) xTaskGetTickCount());
                huart2print(uart_buf, strlen(uart_buf));
#endif
                SetTypicalRegisterSettings(LORA, 25, spreadingFactor, bandwidth, codingRate, bleChannelNr);
                txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
                txBuf[1] = 0x02;  //irqMask   7:0  enable RxDone interrupt
                txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
                txBuf[3] = 0x02;  //dio1Mask  7:0  map RXDone to DIO1
                txBuf[4] = 0x00;  //dio2Mask 15:8
                txBuf[5] = 0x00;  //dio2Mask  7:0
                txBuf[6] = 0x00;  //dio3Mask 15:8
                txBuf[7] = 0x00;  //dio3Mask  7:0
                SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); // 0x8D
                ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt
                notificationValue = 0;
                xTaskNotifyStateClear(netwConThreadHandler);
                SetRx(); // Sensor Node to receive confirmation from the Host
                // rxBuf[ 0] = STARTDELIMITER;                  // "S" Start Delimiter
                // rxBuf[ 1] = moduleTable[0].ID[0];            // MyID LSB
                // rxBuf[ 2] = moduleTable[0].ID[1];
                // rxBuf[ 3] = moduleTable[0].ID[2];
                // rxBuf[ 4] = moduleTable[0].ID[3];
                // rxBuf[ 5] = moduleTable[0].ID[4];
                // rxBuf[ 6] = moduleTable[0].ID[5];
                // rxBuf[ 7] = moduleTable[0].ID[6];
                // rxBuf[ 8] = moduleTable[0].ID[7];            // MyID MSB
                // rxBuf[ 9] = moduleTable[ModuleNr].ID[0];     // Node ID LSB
                // rxBuf[10] = moduleTable[ModuleNr].ID[1];
                // rxBuf[11] = moduleTable[ModuleNr].ID[2];
                // rxBuf[12] = moduleTable[ModuleNr].ID[3];
                // rxBuf[13] = moduleTable[ModuleNr].ID[4];
                // rxBuf[14] = moduleTable[ModuleNr].ID[5];
                // rxBuf[15] = moduleTable[ModuleNr].ID[6];
                // rxBuf[16] = moduleTable[ModuleNr].ID[7];      // Node ID MSB
                // rxBuf[17] = CONFIRMPAIRING;                   // "C" Confirm pairing command, "You are my friend now!"
                // rxBuf[18] = (uint8_t)(sequenceNumber & 0xFF); // Sequence Number LSB
                // rxBuf[19] = (uint8_t)(sequenceNumber & 0xFF);
                // rxBuf[20] = (uint8_t)(sequenceNumber & 0xFF);
                // rxBuf[21] = (uint8_t)(sequenceNumber & 0xFF); // Sequence Number MSB
                // rxBuf[22] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 LSB
                // rxBuf[23] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 2nd Byte
                // rxBuf[24] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 3rd Byte
                // rxBuf[25] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 4th Byte
                // rxBuf[26] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 5th Byte
                // rxBuf[27] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 6th Byte
                // rxBuf[28] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 7th Byte
                // rxBuf[29] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpoch1 MSB
                // rxBuf[30] = (uint8_t)CheckSumCalc;            // Check Sum
                // rxBuf[31] = ENDDELIMITER;                     // "E" Message End delimiter
                if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(9000) )) // Listen maximum 9s.
                {// Received message with no time-out, DIOx received
                  xSemaphoreTake(timeStampIntMutex, pdMS_TO_TICKS(400));
                  timeStamp8  = timeStampInt;
                  xSemaphoreGive(timeStampIntMutex);



                  if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
                  { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
                    ClearIrqStatus(IRQ_RADIO_ALL);//clear the interrupt //not working correctly?
  //#if PRINTF_APP_NETWORK_CONNECT
                    // waitToPrint();
                    // npf_snprintf(uart_buf, 200, "%u [app_network_connect] RxDone.\r\n",(unsigned int) xTaskGetTickCount());
                    // huart2print(uart_buf, strlen(uart_buf));
  //#endif
                    ReadRXBuffer(0, rxBuf, 32);
#if PRINTF_APP_NETWORK_CONNECT
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Payload received: ",(unsigned int) xTaskGetTickCount());
                    for (unsigned int i = 0; i < 32; i++)
                    {
                      npf_snprintf(byteString, 3, "%02X", rxBuf[i]);
                      strcat(uart_buf, byteString);
                    }
                    strcat(uart_buf, ".\n");
                    huart2print(uart_buf, strlen(uart_buf));
#endif
                    // First sanity check of received message:
                    if ((rxBuf[0] == STARTDELIMITER) && (rxBuf[17] == CONFIRMPAIRING) && (rxBuf[31] == ENDDELIMITER))
                    {
                      CheckSumCalc = 0;
                      for (uint8_t i = 0; i < 30; i++)
                      {
                        CheckSumCalc = (CheckSumCalc ^ rxBuf[i]);
                      }
                      if (rxBuf[30] == CheckSumCalc)
                      { // received confirmation message is genuine
#if PRINTF_APP_NETWORK_CONNECT
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Genuine Confirmation message received.\r\n",(unsigned int) xTaskGetTickCount());
                        huart2print(uart_buf, strlen(uart_buf));
#endif
                        for (unsigned int i = 1; i < 9; i++)
                        {
                          receivedSensorID[i-1] = rxBuf[i];
                          receivedTargetID[i-1] = rxBuf[i+8];
                        }
                        // remove status bits from ID's:
                        receivedSensorID[0] &= 0b10000001;
                        receivedTargetID[0] &= 0b10000001;
#if PRINTF_APP_NETWORK_CONNECT
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Received Host Unique ID: ",(unsigned int) xTaskGetTickCount());
                        for (unsigned int i = 0; i < 8; i++)
                        {
                          npf_snprintf(byteString, 3, "%02X", receivedSensorID[i]);
                          strcat(uart_buf, byteString);
                        }
                        strcat(uart_buf, ".\n");
                        huart2print(uart_buf, strlen(uart_buf));
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Received Target Unique ID: ",(unsigned int) xTaskGetTickCount());
                        for (unsigned int i = 0; i < 8; i++)
                        {
                          npf_snprintf(byteString, 3, "%02X", receivedTargetID[i]);
                          strcat(uart_buf, byteString);
                        }
                        strcat(uart_buf, ".\n");
                        huart2print(uart_buf, strlen(uart_buf));
#endif
                        if (IdenticalIDs(receivedTargetID, moduleTable[0].ID) && IdenticalIDs(receivedSensorID, moduleTable[1].ID))
                        { // message for me: Host is confirming that I'm added to the host module table
                  	      // adding sequence number received from the Host
                          exchangeIteration++;
                          exchangeTrial = 0;
                  	      moduleTable[1].sequence = (uint32_t) (((rxBuf[21] << 8 | rxBuf[20]) << 8 | rxBuf[19]) << 8 | rxBuf[18]);
                          // add TIME_DELAY_TO_START_RTC and 1s, + 10ms for inherent delay for the slave (xEpoch is in 1/100s)
                          xEpoch = xEpoch + 1010;
                          xSemaphoreTake(recEpochMutex, pdMS_TO_TICKS(400));
                          received_epoch = (time_t)(xEpoch / 100);
                          xSemaphoreGive(recEpochMutex);
                          recSubSeconds = xEpoch - (xEpoch / 100) * 100;
                          // notify app_hal_pps that the rtc needs to set
                          app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_SET_RTC);
                          // wait 1s HAL/RTOS time - time between receiving message and coming here
                          delayRtcStartup = 1010 - (xTaskGetTickCount() - timeStamp8) - recSubSeconds + 86;
                          vTaskDelay(delayRtcStartup);
                          // notify app_rtc that the rtc can be started:
                          app_rtc_notify(NOTIFICATION_FROM_APP_NETWORK_CONNECT_START_RTC);
                          // wait a bit so that the RTC has been started
                          vTaskDelay(100);
                          printExtraInfo = 1;
#if PRINTF_APP_NETWORK_CONNECT
                          waitToPrint();
                          npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] My host found!\r\n",(unsigned int) xTaskGetTickCount());
                          huart2print(uart_buf, strlen(uart_buf));
                          waitToPrint();
                          npf_snprintf(uart_buf, 200, "Number |   Full Unique ID   | type |  Started or connected  | Sequence | FEI (Hz) | RSSI (dBm) | SNR (dB)\r\n");
                          huart2print(uart_buf, strlen(uart_buf));
                          waitToPrint();
                          npf_snprintf(uart_buf, 200, "-------+--------------------+------+------------------------+----------+----------+------------+---------\r\n");
                          huart2print(uart_buf, strlen(uart_buf));
                          for (unsigned int j = 0; j < 2; j++)
                          {
                            waitToPrint();
                            npf_snprintf(uart_buf, 200, " %05u | 0x",(unsigned int)moduleTable[j].number);
                            for (unsigned int k = 0; k < 8; k++)
                            {
                              npf_snprintf(byteString, 3, "%02X", moduleTable[j].ID[k]);
                              strcat(uart_buf, byteString);
                            }
                            if (moduleTable[j].type)
                            {
                              strcat(uart_buf, " | Host | ");
                            }
                            else
                            {
                              strcat(uart_buf, " | Node | ");
                            }
                            huart2print(uart_buf, strlen(uart_buf));
                            waitToPrint();
                            rec_tmStructure = gmtime(&moduleTable[j].epochConnected);
                            npf_snprintf(uart_buf, 200, "%02d/%02d/%02d %02d:%02d:%02d.%02d | ",
                                rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
                                rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
                            huart2print(uart_buf, strlen(uart_buf));
                            waitToPrint();
                            npf_snprintf(uart_buf, 200, "%05u |  %07.2f |  %05.2f |  %05.2f\r\n", (unsigned int) moduleTable[1].sequence, moduleTable[j].hostfei, moduleTable[j].hostrssi, moduleTable[j].hostsnr);
                            huart2print(uart_buf, strlen(uart_buf));
                          }
#endif
                        }
                      }
                      else
                      { // checkSumCalc of confirmation message is not correct
#if PRINTF_APP_NETWORK_CONNECT
                        waitToPrint();
                        npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Check sum of Confirmation Message is not correct.\r\n",(unsigned int) xTaskGetTickCount());
                        huart2print(uart_buf, strlen(uart_buf));
#endif
                        //todo inform the host that the confirmation message received is not correct
                      }
                    } // end of first sanity check of Confirmation Message
                    else
                    {
  #if PRINTF_APP_NETWORK_CONNECT
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "%u [app_network_connect] Received Confirmation Message did not pass first sanity check.\r\n",(unsigned int) xTaskGetTickCount());
                      huart2print(uart_buf, strlen(uart_buf));
  #endif
                    }
                  } // end of notificationValue == NOTIFICATION_FROM_DIO1
                  else
                  {
#if PRINTF_APP_NETWORK_CONNECT
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Wrong notification value received while waiting for Confirmation Message.\r\n",(unsigned int) xTaskGetTickCount());
                    huart2print(uart_buf, strlen(uart_buf));
#endif
                  }
                } // end of Received message with no time-out, DIOx received for Confirmation Message
                else
                {
#if PRINTF_APP_NETWORK_CONNECT
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "%u ANTENNARSSITEST [app_network_connect] Time out occurred while waiting for Confirmation Message.\r\n",(unsigned int) xTaskGetTickCount());
                  huart2print(uart_buf, strlen(uart_buf));
#endif
                }
#endif // ANTENNARSSITEST
              }
              else
              { // message is not from my host. Check if this host is already existing in my module table
                for (unsigned int i = 2; i < MAXNRMODULES; i++) // moduleTable[0 and 1] does not need to be checked
                {
                  if (!moduleTable[i].number)
                  { // first empty position create a new record to store the new host, but let him know that I'm already connected with another host
                    moduleTable[i].number = moduleTable[i-1].number + 1U;
#if PRINTF_APP_NETWORK_CONNECT
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "%u [app_network_connect] Extra host found to add.\r\n",(unsigned int) xTaskGetTickCount());
                    huart2print(uart_buf, strlen(uart_buf));
#endif
                    i = MAXNRMODULES - 1; // jump out of the for..next loop
                    // todo ...
                    // end of new record
                  }
                  else
                  {
                    if (IdenticalIDs(receivedSensorID, moduleTable[i].ID))
                    { // module is already known, if not, check next record
#if PRINTF_APP_NETWORK_CONNECT
                      waitToPrint();
                      npf_snprintf(uart_buf, 200, "%u [app_network_connect] Message from known extra host.\r\n",(unsigned int) xTaskGetTickCount());
                      huart2print(uart_buf, strlen(uart_buf));
#endif
                      // store the position, epoch, healthy, etc information of the other known host or it is a specific command
                      // todo ...
                    }
                  }
                } // end of for loop to run through module table
              } // end of check of message from another host
            } // end of check of message from a host
            if (printExtraInfo)
            {
//              printExtraInfo = 0;
//              pcf2131_get_time(&rtc_time);
//              // Calculate Epoch:
//              rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
//              rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
//              rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
//              rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
//              rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
//              rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
//              rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
//              // calculate epoch, mktime will also calculate tm_wday and tm_yday.
//              // epoch = #s elapsed since 1/1/1970
//              // mktime consumes 5.46KB of FASH memory!!
//              rtc_epoch = mktime(&rtc_tmStructure);
//              // Node: print date/time info of received epoch and current running rtc
//              rec_tmStructure = gmtime(&received_epoch);
//              waitToPrint();
//              npf_snprintf(uart_buf, 200, "%u [app_network_connect] delay used to start the RTC was %ums.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) delayRtcStartup);
//              huart2print(uart_buf, strlen(uart_buf));
//              waitToPrint();
//              npf_snprintf(uart_buf, 200, "%u [app_network_connect] Host Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n", (unsigned int) xTaskGetTickCount(),
//                  rec_tmStructure->tm_mday, rec_tmStructure->tm_mon + 1, rec_tmStructure->tm_year + 1900,
//                   rec_tmStructure->tm_hour, rec_tmStructure->tm_min, rec_tmStructure->tm_sec, (unsigned int)recSubSeconds);
//              huart2print(uart_buf, strlen(uart_buf));
//              waitToPrint();
//              npf_snprintf(uart_buf, 200, "%u [app_network_connect]  RTC Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n", (unsigned int) xTaskGetTickCount(),
//                   rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds);
//              huart2print(uart_buf, strlen(uart_buf));
            }
          } // end of received message is genuine
          else
          {
#if PRINTF_APP_NETWORK_CONNECT
            waitToPrint();
            npf_snprintf(uart_buf, 200, "[network_connect]    Checksum done at %ums -> Checksum error, received message is not genuine or error during transmission.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
//            vTaskDelay(1000);
          }
        } // end of first sanity check of received message
        else
        {
#if PRINTF_APP_NETWORK_CONNECT
            waitToPrint();
            npf_snprintf(uart_buf, 200, "[network_connect] First check done at %ums -> Error, received message did not pass first sanity check.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      } // end of notificationValue == NOTIFICATION_FROM_DIO1
      else
      {
#if PRINTF_APP_NETWORK_CONNECT
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_network_connect] Wrong notification value received: 0x%02X.\r\n",
           (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif

      }
    } // end of received message with no time-out, DIOx received
    else
    {// time-out on Radio Rx, no DIOx received
#if PRINTF_APP_NETWORK_CONNECT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_network_connect] Nothing received during 11s...Retry.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
    //GoToStandby(0);

#if SUPERCAPUSED
    ScapThreadNotify(NOTIFICATION_LOAD_SCAP);     // first make sure that the supercap is loaded
//    xTaskNotifyStateClear(netwConThreadHandler);
//    notificationValue = 0;
//    while ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
//    { // waiting for a notification value from the app_supercap that the super capacitor is loaded
//      xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(10000));
//      if ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
//      {
//        xTaskNotifyStateClear(netwConThreadHandler);
//        notificationValue = 0;
//      }
//      else
//      {
//#if PRINTF_APP_NETWORK_CONNECT
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [network_connect] Super capacitor loaded. Ready to start.\r\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//    }
#endif

#if ANTENNARSSITEST
    vTaskDelayUntil(&xLastWakeTime, 2000U);
#else
#if PLANTSENSOR
    vTaskDelayUntil(&xLastWakeTimeNetCon, 3000U);
#else
    vTaskDelayUntil(&xLastWakeTimeNetCon, 10000U);
#endif // PLANTSENSOR
#endif // ANTENNARSSITEST
#endif // SENSOR_NODE
  }
}



#if SENSOR_HOST
void BroadCastMyID(void)
{
  uint32_t   notificationValue = 0;      // Used to identify where the notification is coming from.
//20240622 moved to start of new iteration of broadcast, to align the start of the broadcast better to GNSSPP10S
  // Set DIO interrupt request parameters
  txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
  txBuf[1] = 0x01;  //irqMask   7:0  enable TxDone interrupt
  txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
  txBuf[3] = 0x01;  //dio1Mask  7:0  map TXDone to DIO1
  txBuf[4] = 0x00;  //dio2Mask 15:8
  txBuf[5] = 0x00;  //dio2Mask  7:0
  txBuf[6] = 0x00;  //dio3Mask 15:8
  txBuf[7] = 0x00;  //dio3Mask  7:0
  SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); //0x8D
  //load transmit data
  txBuf[ 0] = STARTDELIMITER;           // "S" Start Delimiter
  txBuf[ 1] = moduleTable[0].ID[0];     // MyID LSB
  txBuf[ 2] = moduleTable[0].ID[1];
  txBuf[ 3] = moduleTable[0].ID[2];
  txBuf[ 4] = moduleTable[0].ID[3];
  txBuf[ 5] = moduleTable[0].ID[4];
  txBuf[ 6] = moduleTable[0].ID[5];
  txBuf[ 7] = moduleTable[0].ID[6];
  txBuf[ 8] = moduleTable[0].ID[7];     // MyID MSB

  // take Broadcast time stamp:

  // to know how long this process takes, take RTOS time stamp before and after reading the RTC:
  rtosTimeStampBCastStrt = xTaskGetTickCount();
  if (!pcf2131_is_running())
  { // only broadcast my ID if the clock is running,
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at start Broadcast, skip broadcast.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  else
  {
    pcf2131_get_time(&rtc_time);
    // Calculate Epoch:
    rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
    rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
    rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
    rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
    rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
    rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
    rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
    // calculate epoch, mktime will also calculate tm_wday and tm_yday.
    // epoch = #s elapsed since 1/1/1970
    // mktime consumes 5.46KB of FASH memory!!
    rtc_epoch = mktime(&rtc_tmStructure);
    xEpoch = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds - 50; // -50cs because the RTC runs 500ms before the GNSS reference time
    txBuf[ 9] = (uint8_t)(xEpoch & 0xFF); // My Epoch LSB
    xEpoch    = xEpoch >> 8;
    txBuf[10] = (uint8_t)(xEpoch & 0xFF); // My Epoch 2nd Byte
    xEpoch    = xEpoch >> 8;
    txBuf[11] = (uint8_t)(xEpoch & 0xFF); // My Epoch 3rd Byte
    xEpoch    = xEpoch >> 8;
    txBuf[12] = (uint8_t)(xEpoch & 0xFF); // My Epoch 4th Byte
    xEpoch    = xEpoch >> 8;
    txBuf[13] = (uint8_t)(xEpoch & 0xFF); // My Epoch 5th Byte
    xEpoch    = xEpoch >> 8;
    txBuf[14] = (uint8_t)(xEpoch & 0xFF); // My Epoch 6th Byte
    xEpoch    = xEpoch >> 8;
    txBuf[15] = (uint8_t)(xEpoch & 0xFF); // My Epoch 7th Byte
    xEpoch    = xEpoch >> 8;
    txBuf[16] = (uint8_t)(xEpoch & 0xFF); // My Epoch MSB
    txBuf[17] = HOSTISTHEREANYONE;        // "P" pairing command, "Hello, is there anyone?"
    CheckSumCalc = 0;
    for (uint8_t i = 0; i < 18; i++)
    {
      CheckSumCalc = (CheckSumCalc ^ txBuf[i]);
    }
    txBuf[18] = (uint8_t)CheckSumCalc;
    txBuf[19] = ENDDELIMITER;             // "E" Message End delimiter
    WriteTXBuffer(0, txBuf, 20);          // TX Buffer offset = 0, 20 byte payload
    xTaskNotifyStateClear(netwConThreadHandler);
    ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt
    rtosTimeStampBCastData = xTaskGetTickCount();
    SetTx();                              //transmit MyID with current settings for both Sensor host (master) and sensor node (slave)
    if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(5000)))
    {// No time-out
      if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
      { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
        rtosTimeStampBCastDone = timeStampInt;
        ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt //not working correctly?
#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
        // 20240622 moved following message from app_rtc_sync AFTER the broadcast message to avoid a delay of 7ms
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [app_rtc_sync] GNSSPP10S #%u, @%ums. RTOS time difference with previous GNSSPP10S = %ums. Total drift since RTOS synchronization = %dms.\r\n",
//            (unsigned int) xTaskGetTickCount(), (unsigned int) gnss_pps, (unsigned int) rtosTimeStampGNSSPPS, (unsigned int)rtosDriftToGNSS, (int) totalRtosDriftToGNSS);
//        huart2print(uart_buf, strlen(uart_buf));
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [network_connect] Broadcast started at %ums -> %dms after GNSSPPS #%u, RTC data collection took %ums. RTC Date: %d/%d/%d, UTC Time: %d:%d:%d.%d, RTC epoch: %lds.\r\n",
            (unsigned int) xTaskGetTickCount(), (unsigned int) rtosTimeStampBCastStrt, (int) (rtosTimeStampBCastStrt - rtosTimeStampGNSSPPS), (unsigned int) gnss_pps, (unsigned int) (rtosTimeStampBCastData - rtosTimeStampBCastStrt),
      	    rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds, (long)rtc_epoch);
        huart2print(uart_buf, strlen(uart_buf));
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [network_connect] Broadcast done at %ums -> %ums duration. Payload: ",
            (unsigned int) xTaskGetTickCount(), (unsigned int) rtosTimeStampBCastDone, (unsigned int) (rtosTimeStampBCastDone - rtosTimeStampBCastStrt));
        for (unsigned int i = 0; i < 20; i++)
        {
          npf_snprintf(byteString, 3, "%02X", txBuf[i]);
          strcat(uart_buf, byteString);
        }
        strcat(uart_buf, ". Waiting to be paired.\n");
        huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
      }
    }
    else
    {// time-out
#if PRINTF_APP_NETWORK_CONNECT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_network_connect] [BroadCastMyID] DIOx pin time-out occurred...(>5s).\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
  }
//  GoToStandby(0);
}

void ConfirmPairing(uint32_t ModuleNr)
{
  uint32_t notificationValue = 0;      // Used to identify where the notification is coming from.
  uint32_t sequenceNumber = moduleTable[ModuleNr].number;
  // Set DIO interrupt request parameters
  txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
  txBuf[1] = 0x01;  //irqMask   7:0  enable TxDone interrupt
  txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
  txBuf[3] = 0x01;  //dio1Mask  7:0  map TXDone to DIO1
  txBuf[4] = 0x00;  //dio2Mask 15:8
  txBuf[5] = 0x00;  //dio2Mask  7:0
  txBuf[6] = 0x00;  //dio3Mask 15:8
  txBuf[7] = 0x00;  //dio3Mask  7:0
  SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); //0x8D
  //load transmit data
  txBuf[ 0] = STARTDELIMITER;                  // "S" Start Delimiter
  txBuf[ 1] = moduleTable[0].ID[0];            // MyID LSB
  txBuf[ 2] = moduleTable[0].ID[1];
  txBuf[ 3] = moduleTable[0].ID[2];
  txBuf[ 4] = moduleTable[0].ID[3];
  txBuf[ 5] = moduleTable[0].ID[4];
  txBuf[ 6] = moduleTable[0].ID[5];
  txBuf[ 7] = moduleTable[0].ID[6];
  txBuf[ 8] = moduleTable[0].ID[7];            // MyID MSB
  txBuf[ 9] = moduleTable[ModuleNr].ID[0];     // Node ID LSB
  txBuf[10] = moduleTable[ModuleNr].ID[1];
  txBuf[11] = moduleTable[ModuleNr].ID[2];
  txBuf[12] = moduleTable[ModuleNr].ID[3];
  txBuf[13] = moduleTable[ModuleNr].ID[4];
  txBuf[14] = moduleTable[ModuleNr].ID[5];
  txBuf[15] = moduleTable[ModuleNr].ID[6];
  txBuf[16] = moduleTable[ModuleNr].ID[7];      // Node ID MSB
  txBuf[17] = CONFIRMPAIRING;                   // "C" Confirm pairing command, "You are my friend now!"
  txBuf[18] = (uint8_t)(sequenceNumber & 0xFF); // Sequence Number LSB
  sequenceNumber = sequenceNumber >> 8;
  txBuf[19] = (uint8_t)(sequenceNumber & 0xFF);
  sequenceNumber = sequenceNumber >> 8;
  txBuf[20] = (uint8_t)(sequenceNumber & 0xFF);
  sequenceNumber = sequenceNumber >> 8;
  txBuf[21] = (uint8_t)(sequenceNumber & 0xFF); // Sequence Number MSB
  txBuf[22] = 0xFA;                             // Spare
  txBuf[23] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 LSB
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[24] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 2nd Byte
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[25] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 3rd Byte
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[26] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 4th Byte
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[27] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 5th Byte
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[28] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 6th Byte
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[29] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 7th Byte
  xEpochLht5    = xEpochLht5 >> 8;
  txBuf[30] = (uint8_t)(xEpochLht5 & 0xFF);         // My xEpochLht5 MSB

  // take time stamp Lh(t6):
  while (!pcf2131_is_running())
  {
	//todo remove risk of endless loop
    pcf2131_start_clock();
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at reading Lh(t6).\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
    vTaskDelay(20);
#endif
  }

  pcf2131_get_time(&rtc_time);
  // Calculate Epoch:
  rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
  rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
  rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
  rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
  rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
  rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
  rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
  // calculate epoch, mktime will also calculate tm_wday and tm_yday.
  // epoch = #s elapsed since 1/1/1970
  // mktime consumes 5.46KB of FASH memory!!
  rtc_epoch = mktime(&rtc_tmStructure);
  xEpochLht6 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds - 50;

  txBuf[31] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 LSB
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[32] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 2nd Byte
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[33] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 3rd Byte
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[34] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 4th Byte
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[35] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 5th Byte
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[36] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 6th Byte
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[37] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 7th Byte
  xEpochLht6    = xEpochLht6 >> 8;
  txBuf[38] = (uint8_t)(xEpochLht6 & 0xFF);         // My xEpochLht6 MSB
  xEpochLht6 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds - 50;

  CheckSumCalc = 0;
  for (uint8_t i = 0; i < 39; i++)
  {
    CheckSumCalc = (CheckSumCalc ^ txBuf[i]);
  }
  txBuf[39] = (uint8_t)CheckSumCalc;             // Check Sum
  txBuf[40] = ENDDELIMITER;                      // "E" Message End delimiter
  WriteTXBuffer(0, txBuf, 41);                   // TX Buffer offset = 0, 32 byte payload
  xTaskNotifyStateClear(netwConThreadHandler);
  SetTx();                                       //transmit Confirmation message including synchronization information
  if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(5000)))
  {// No time-out
    if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
    { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
      ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt //not working correctly?
//#if PRINTF_APP_NETWORK_CONNECT
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_network_connect] TxDone.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
    }
  }
  else
  {// time-out
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_network_connect] [ConfirmPairing] DIOx pin time-out occurred...(>5s).\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
//#if !ANTENNARSSITEST
#if PRINTF_APP_NETWORK_CONNECT
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_network_connect] [ConfirmPairing] Confirmation done, payload: ",(unsigned int) xTaskGetTickCount());
  for (unsigned int i = 0; i < 41; i++)
  {
    npf_snprintf(byteString, 3, "%02X", txBuf[i]);
    strcat(uart_buf, byteString);
  }
  strcat(uart_buf, "\r\n");
  huart2print(uart_buf, strlen(uart_buf));
#endif
//#endif
}

void ConfirmDataReceived(uint8_t ModuleNr)
{

}



#else // SENSOR_NODE
void AnswerPairing(void)
{
  uint32_t   notificationValue = 0;      // Used to identify where the notification is coming from.
  // Set DIO interrupt request parameters
  txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
  txBuf[1] = 0x01;  //irqMask   7:0  enable TxDone interrupt
  txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
  txBuf[3] = 0x01;  //dio1Mask  7:0  map TXDone to DIO1
  txBuf[4] = 0x00;  //dio2Mask 15:8
  txBuf[5] = 0x00;  //dio2Mask  7:0
  txBuf[6] = 0x00;  //dio3Mask 15:8
  txBuf[7] = 0x00;  //dio3Mask  7:0
  SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); //0x8D
  //load transmit data
  txBuf[ 0] = STARTDELIMITER;       // "S" Start Delimiter
  txBuf[ 1] = moduleTable[0].ID[0]; // MyID LSB
  txBuf[ 2] = moduleTable[0].ID[1];
  txBuf[ 3] = moduleTable[0].ID[2];
  txBuf[ 4] = moduleTable[0].ID[3];
  txBuf[ 5] = moduleTable[0].ID[4];
  txBuf[ 6] = moduleTable[0].ID[5];
  txBuf[ 7] = moduleTable[0].ID[6];
  txBuf[ 8] = moduleTable[0].ID[7]; // MyID MSB
  txBuf[ 9] = moduleTable[1].ID[0]; // Host ID LSB
  txBuf[10] = moduleTable[1].ID[1];
  txBuf[11] = moduleTable[1].ID[2];
  txBuf[12] = moduleTable[1].ID[3];
  txBuf[13] = moduleTable[1].ID[4];
  txBuf[14] = moduleTable[1].ID[5];
  txBuf[15] = moduleTable[1].ID[6];
  txBuf[16] = moduleTable[1].ID[7]; // Host ID MSB
  txBuf[17] = rawEfeBuf[1];         // received ESTIMATED_FREQUENCY_ERROR MSB
  txBuf[18] = rawEfeBuf[2];
  txBuf[19] = rawEfeBuf[3];         // received ESTIMATED_FREQUENCY_ERROR LSB
  txBuf[20] = packetStatusBuf[1];   // rssiSync: Communication RSSI of packet received by the node. Actual signal power is -(rssiSync)/2 dBm
  txBuf[21] = packetStatusBuf[2];   // snr: estimation of SNR of last package received. In two's complement format x 4. Actual SNR is (snr)/4 dB
  txBuf[22] = NODEIAMHERE;          // "Y" pairing ANSWER command, "Yes, I'm here!"
  txBuf[23] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy LSB
  xEpoch    = xEpoch >> 8;
  txBuf[24] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy 2nd Byte
  xEpoch    = xEpoch >> 8;
  txBuf[25] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy 3rd Byte
  xEpoch    = xEpoch >> 8;
  txBuf[26] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy 4th Byte
  xEpoch    = xEpoch >> 8;
  txBuf[27] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy 5th Byte
  xEpoch    = xEpoch >> 8;
  txBuf[28] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy 6th Byte
  xEpoch    = xEpoch >> 8;
  txBuf[29] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy 7th Byte
  xEpoch    = xEpoch >> 8;
  txBuf[30] = (uint8_t)(xEpoch & 0xFF);         // My SyncEpochDummy MSB

  // take time stamp Ln(t1):

  while (!pcf2131_is_running())
  {
    pcf2131_start_clock();
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [network_connect] Clock is not running at reading Ln(t1).\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
    vTaskDelay(20);
#endif
  }


  pcf2131_get_time(&rtc_time);
  // Calculate Epoch:
  rtc_tmStructure.tm_year  = (uint16_t)(rtc_time.year - 1900); // Year - 1900
  rtc_tmStructure.tm_mon   = (uint8_t) (rtc_time.month -   1); // Month, where 0 = jan
  rtc_tmStructure.tm_mday  = (uint8_t) (rtc_time.dayOfMonth);  // Day of the month
  rtc_tmStructure.tm_hour  = (uint8_t)  rtc_time.hours;        // 24 hours format
  rtc_tmStructure.tm_min   = (uint8_t)  rtc_time.minutes;
  rtc_tmStructure.tm_sec   = (uint8_t)  rtc_time.seconds;      // because the clock will be started on the next PPS
  rtc_tmStructure.tm_isdst = 0;                                // Is DST on? 1 = yes, 0 = no, -1 = unknown
  // calculate epoch, mktime will also calculate tm_wday and tm_yday.
  // epoch = #s elapsed since 1/1/1970
  // mktime consumes 5.46KB of FASH memory!!
  rtc_epoch = mktime(&rtc_tmStructure);
  xEpochLnt1 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds;

  txBuf[31] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 LSB
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[32] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 2nd Byte
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[33] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 3rd Byte
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[34] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 4th Byte
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[35] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 5th Byte
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[36] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 6th Byte
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[37] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 7th Byte
  xEpochLnt1    = xEpochLnt1 >> 8;
  txBuf[38] = (uint8_t)(xEpochLnt1 & 0xFF);         // My xEpochLnt1 MSB
  xEpochLnt1 = ((uint64_t)(rtc_epoch)) * (uint64_t)100 + (uint64_t)rtc_time.subSeconds;

  CheckSumCalc = 0;
  for (uint8_t i = 0; i < 39; i++)
  {
    CheckSumCalc = (CheckSumCalc ^ txBuf[i]);
  }
  txBuf[39] = (uint8_t)CheckSumCalc;             // Check Sum
  txBuf[40] = ENDDELIMITER;                      // "E" Message End delimiter
  WriteTXBuffer(0, txBuf, 41);                   // TX Buffer offset = 0, 32 byte payload

  xTaskNotifyStateClear(netwConThreadHandler);
  SetTx(); //transmit MyID to my Host
  if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(3000)))
  {// No time-out
    if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
    { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
      timeStamp1 = startOfTransmit;
      ClearIrqStatus(IRQ_RADIO_ALL);             //clear the interrupt //not working correctly?
#if PRINTF_APP_NETWORK_CONNECT
      waitToPrint();
      npf_snprintf(uart_buf, 200, "[network_connect] AnswerPairing done at %ums -> Broadcast message answered, payload: ",(unsigned int) xTaskGetTickCount());
      for (unsigned int i = 0; i < 41; i++)
      {
        npf_snprintf(byteString, 3, "%02X", txBuf[i]);
        strcat(uart_buf, byteString);
      }
      strcat(uart_buf, ".\r\n");
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
  }
  else
  {// time-out
#if PRINTF_APP_NETWORK_CONNECT
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_network_connect] [AnswerPairing] DIOx pin time-out occurred...(>3s).\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
//  GoToStandby(0);
}

void DataToHost(void)
{

}

#endif

uint8_t IdenticalIDs(char *ID1, char *ID2)
{
  for (unsigned int i = 0; i < 8; i++)
  {
    if (ID1[i] != ID2[i])
    {
       return 0;
    }
  }
  return 1;
}

void app_netw_con_notify_fromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (netwConThreadHandler != NULL)
  {
    xTaskNotifyFromISR(netwConThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void NwConThreadNotify(uint32_t notValue)
{
  if (netwConThreadHandler != NULL)
  {
    xTaskNotify(netwConThreadHandler, notValue, eSetBits);
  }
}

