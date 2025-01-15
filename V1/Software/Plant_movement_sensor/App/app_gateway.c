/*
 * app_gateway.c
 *
 *  Created on: 17 Sep 2023
 *      Author: Sarah Goossens
 */

#include "main.h"
#include "app_gateway.h"
#include "usart.h"              // to declare huart2
#include <string.h>
#include "../Drivers/SX1280/SX1280.h"
#include "../Drivers/PCF2131/PCF2131.h"
#include "time.h"
//#include "app_rtc.h"
//#include "app_hal_pps.h"
#include "spi.h"
//#include "../../App/app_lorawan.h"
//#include "../../App/app_lorawan_core.h"
#include "../../App/app_lorawan_crypto.h"
#include "../../App/app_lorawan_defs.h"
#include "app_imu.h"
#include "lptim.h"
#include "app_MahonyAHRS.h"
#include "app_MadgwickAHRS.h"
#include "app_adc.h"

#include "app_imu.h"
//#include "../../Drivers/Invn/Devices/HostSerif.h"
#include "../../Drivers/Invn/Devices/Device.h"
#include "app_supercap.h"
#include "iwdg.h"
#include "idd_io_hal.h"
#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
//
//#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Defs.h"
//#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
//#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"

//#include "app_hal_pps.h"
//#include "app_rtc.h"
//#include "app_hal_pps.h"
//#include "app_led.h"

#define PRINTF_APP_GATEWAY 1

#define USE_CAL_ACC 0
#define USE_CAL_GYR 0
#define USE_CAL_MAG 0
#define USE_RV      1    /* requires COMPASS*/ //quat 9 axis
#define USE_ORI     0    /* requires COMPASS*/ //euler angles
#define USE_ACC_MAG 0	//use only acc and mag




//extern TaskHandle_t halPPSThreadHandler;
//extern TaskHandle_t rtcThreadHandler;
//extern TaskHandle_t halSyncThreadHandler;
//extern TaskHandle_t ledThreadHandler;

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

extern char                  uart_buf[200];
extern char                  byteString[3];
extern uint32_t              LoRaDevAddrInit;
static uint8_t               txBuf[40];
static uint8_t               rxBuf[52];
static uint8_t               payload[32];
	   uint16_t              payloadLength;
static uint8_t               encPayload[32];
//static uint8_t               decPayload[25];
       uint32_t              mic;
       uint32_t              LR1_countUp = 0x0000;
       uint32_t              LR1_countDown = 0x0000;
       uint8_t               devAddr[4];
       uint8_t               rxLength = 0;
       uint8_t               rxMacHeader = 0;
       uint8_t               rxDevAddr[4];
       uint8_t               rxFCtrl = 0;
       uint8_t 	             rxADR = 0;
       uint8_t 	             rxADRACKReq = 0;
       uint8_t               rxACK = 0;
       uint8_t               rxClassB = 0;
       uint8_t               rxFOptsLen = 0;
       uint8_t               rxFOpts[15];
       uint8_t               rxFPort = 0;
       uint8_t               rxFRMPayload[50];
       uint8_t               rxFRMPayloadLength = 0;
       uint32_t              rxMic;
       uint32_t              rxMicCheck;
extern int                   pitch;
extern int                   yaw;
extern int                   roll;
       int16_t 	             imuData[32];
       int16_t               quatArray[10];
extern float                 accX;
extern float                 accY;
extern float                 accZ;
extern float                 magX;
extern float                 magY;
extern float                 magZ;
extern float                 gyrX;
extern float                 gyrY;
extern float                 gyrZ;
extern float                 accBias[3];
extern float                 gyrBias[3];
extern float                 magBias[3];
extern inv_sensor_config_offset_t accOffset;
	   int16_t               am_quatW = 0; //quaternion values calculated with mahony filter from acc and mag values
	   int16_t               am_quatX = 0;
	   int16_t               am_quatY = 0;
	   int16_t               am_quatZ = 0;
extern int16_t               quatW;
extern int16_t	             quatX;
extern int16_t	             quatY;
extern int16_t	             quatZ;
extern uint8_t               pollIMU;
extern inv_device_icm20948_t device_icm20948;
extern RadioStatus_t         statusSX1280;
extern uint32_t              imuMeasurementNr;
extern uint16_t              batteryVoltageLevel;
extern uint16_t              batteryInVoltageLevel;
extern uint16_t              supercapVoltageLevel;
extern uint16_t              vCoreVoltageLevel;
extern uint16_t              vCoreTemperature;
extern TickType_t            ledFreq;
extern int 		             calibrationActive;
       int                   calibrationActiveOld = 0;
       int	                 numberOfMeas = 0;

     //  MahonyAHRS ahrs;   // Mahony filter object
       float                 q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Initial quaternion values

//extern uint16_t              batteryVoltageLevel;
//extern uint16_t              batteryInVoltageLevel;
//extern uint16_t              supercapVoltageLevel;
//extern uint16_t              vCoreVoltageLevel;
//extern uint16_t              vCoreTemperature;
//extern uint8_t               ubAdcGrpRegularUnitaryConvStatus;
//extern uint16_t              uhADCxConvertedData;
//extern uint64_t              startSuperCapOn;
//extern uint64_t              superCapOnTime;
//extern uint32_t              superCapAvailable;
extern ADC_HandleTypeDef     hadc4;
extern uint32_t              radioBusy;

extern uint8_t           pcf2131Buf[54];

extern uint32_t imuAvailable;

extern IWDG_HandleTypeDef hiwdg;
extern uint32_t printReadMemsIMURegContent;

extern uint8_t   dmpBiasFlash[100];

extern uint8_t   stopLoadingSuperCap;
extern uint64_t  timeOfEWDGI;
static inv_device_t * device; // Just a handy variable to keep the handle to device object




//extern lr1mac_states_t lr1mac_state;
//extern lr1_stack_mac_t  lr1_mac_obj;
//static rx_packet_type_t valid_rx_packet     = NO_MORE_VALID_RX_PACKET;
//static uint32_t    failsafe_timstamp_get( void );


//extern lr1mac_states_t lr1mac_state;
//extern lr1_stack_mac_t  lr1_mac_obj;

//static rx_packet_type_t valid_rx_packet     = NO_MORE_VALID_RX_PACKET;
//static user_rx_packet_type_t AvailableRxPacket    = NO_LORA_RXPACKET_AVAILABLE;
//static uint32_t    failsafe_timstamp_get( void );


//LoRaWAN settings for this device
// Define your own LoRaWAN credential
//not needed for ABP
//uint8_t user_dev_eui[8]  = {70, B3, D5, 7E, D0, 06, 09, 04};
//uint8_t user_join_eui[8] = { 0 };
//uint8_t user_app_key[16] = { 0 };

//my device
//uint8_t  LoRaMacNwkSKeyInit[] = {0x5D, 0x39, 0x6F, 0xF2, 0x7E, 0xA3, 0xAF, 0x43, 0xC4, 0x3A, 0x99, 0x41, 0xCD, 0x94, 0xCD, 0xDF};
//uint8_t  LoRaMacAppSKeyInit[] = {0x6D, 0x86, 0x11, 0xEE, 0x20, 0x13, 0x73, 0x61, 0xC1, 0x56, 0xBB, 0x4D, 0x15, 0x65, 0xFF, 0x92};
//uint8_t  LoRaMacAppKeyInit[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t  AppEuiInit[]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x09, 0x04};
//uint32_t LoRaDevAddrInit = 0x260B1B80; //from original file


//static lorawan_keys_t LoraWanKeys = {LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, 0x00000000, ABP_DEVICE};
//static lorawan_keys_t LoraWanKeys = {LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, LoRaDevAddrInit, ABP_DEVICE};
//static lorawan_keys_t LoraWanKeys = {LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, LoRaDevAddrInit, ABP_DEVICE};

//sensors Thomas





//osThreadId gatewayThreadHandler;
TaskHandle_t gatewayThreadHandler;

//static receive_win_t    receive_window_type = RECEIVE_NONE;

/* Private function prototypes -----------------------------------------------*/
//void send_message_to_gateway(lr1_stack_mac_t* lr1_mac);
//void receive_message_from_gateway(lr1_stack_mac_t* lr1_mac);

// startOfTransmit

void GatewayThreadInit()
{
//  osThreadDef(gatewayThread, gateway_thread, osPriorityAboveNormal, 0, 128);
//  gatewayThreadHandler = osThreadCreate(osThread(gatewayThread), NULL);
  if (xTaskCreate ((TaskFunction_t)GatewayThreadStart, "GatewayThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh, &gatewayThreadHandler) != pdPASS)
  {
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
}

void GatewayThreadStart(const void * params)
{
//  TickType_t xLastWakeTime = xTaskGetTickCount();
//
//
//  vTaskDelayUntil(&xLastWakeTime, 500U);

#if PRINTF_APP_GATEWAY
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_gateway] Started.\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif

#if myPlantSensor
  //my device
  uint8_t  LoRaMacNwkSKeyInit[] = {0x5D, 0x39, 0x6F, 0xF2, 0x7E, 0xA3, 0xAF, 0x43, 0xC4, 0x3A, 0x99, 0x41, 0xCD, 0x94, 0xCD, 0xDF};
  uint8_t  LoRaMacAppSKeyInit[] = {0x6D, 0x86, 0x11, 0xEE, 0x20, 0x13, 0x73, 0x61, 0xC1, 0x56, 0xBB, 0x4D, 0x15, 0x65, 0xFF, 0x92};
  //uint8_t  LoRaMacAppKeyInit[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  //uint8_t  AppEuiInit[]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x09, 0x04};
  //uint32_t LoRaDevAddrInit = 0x260B1B80; //from original file
#endif
#if plantSensor1
  uint8_t  LoRaMacNwkSKeyInit[] = {0x58, 0xD3, 0x8F, 0x45, 0x71, 0x57, 0x34, 0x8F, 0x56, 0xF2, 0x2B, 0x11, 0xA3, 0xC0, 0xD0, 0xE9}; //0x5D, 0x39, 0x6F, 0xF2, 0x7E, 0xA3, 0xAF, 0x43, 0xC4, 0x3A, 0x99, 0x41, 0xCD, 0x94, 0xCD, 0xDF};
  uint8_t  LoRaMacAppSKeyInit[] = {0xE5, 0xA4, 0x57, 0x52, 0x25, 0x7E, 0x47, 0x37, 0x5B, 0x9E, 0x7B, 0xEE, 0x28, 0x2E, 0x8A, 0xBB}; //0x6D, 0x86, 0x11, 0xEE, 0x20, 0x13, 0x73, 0x61, 0xC1, 0x56, 0xBB, 0x4D, 0x15, 0x65, 0xFF, 0x92};
  uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x30, 0xDD}; // 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x09, 0x04};
  //uint32_t LoRaDevAddrInit = 0x260BAA00;
#endif
#if plantSensor2
  uint8_t  LoRaMacNwkSKeyInit[] = {0x31, 0xB6, 0x5A, 0x22, 0xE1, 0xAF, 0x24, 0x50, 0x13, 0x40, 0x68, 0x4A, 0xB4, 0xA8, 0xA7, 0xF7}; //0x5D, 0x39, 0x6F, 0xF2, 0x7E, 0xA3, 0xAF, 0x43, 0xC4, 0x3A, 0x99, 0x41, 0xCD, 0x94, 0xCD, 0xDF};
  uint8_t  LoRaMacAppSKeyInit[] = {0x8E, 0x6F, 0x63, 0x28, 0xDF, 0xAC, 0xC7, 0x7D, 0x25, 0xB9, 0x3F, 0x40, 0x75, 0xD1, 0xE3, 0x3C}; //0x6D, 0x86, 0x11, 0xEE, 0x20, 0x13, 0x73, 0x61, 0xC1, 0x56, 0xBB, 0x4D, 0x15, 0x65, 0xFF, 0x92};
  uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x30, 0xDE}; //0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x09, 0x04};
  //uint32_t LoRaDevAddrInit = 0x260B83A5; //260B1B80; //from original file
#endif
#if plantSensor3
  uint8_t  LoRaMacNwkSKeyInit[] = {0x58, 0x5A, 0x76, 0xC7, 0x1B, 0x1F, 0x29, 0x88, 0x91, 0xB7, 0xED, 0x48, 0x90, 0xA2, 0x04, 0xFE}; //0x5D, 0x39, 0x6F, 0xF2, 0x7E, 0xA3, 0xAF, 0x43, 0xC4, 0x3A, 0x99, 0x41, 0xCD, 0x94, 0xCD, 0xDF};
  uint8_t  LoRaMacAppSKeyInit[] = {0x4C, 0x32, 0x35, 0x06, 0x8B, 0x5D, 0xCB, 0x6E, 0x73, 0x04, 0x7A, 0x54, 0x7D, 0x62, 0x99, 0xA8}; //0x6D, 0x86, 0x11, 0xEE, 0x20, 0x13, 0x73, 0x61, 0xC1, 0x56, 0xBB, 0x4D, 0x15, 0x65, 0xFF, 0x92};
  uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x30, 0xE0}; //0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x09, 0x04};
  //uint32_t LoRaDevAddrInit = 0x260BB657; //from original file
#endif
#if plantSensor4
  uint8_t  LoRaMacNwkSKeyInit[] = {0xE0, 0x38, 0x92, 0x07, 0xB9, 0xCA, 0x8B, 0xF9, 0xC1, 0xEA, 0x42, 0x11, 0x9C, 0xA0, 0x49, 0x97}; //0x5D, 0x39, 0x6F, 0xF2, 0x7E, 0xA3, 0xAF, 0x43, 0xC4, 0x3A, 0x99, 0x41, 0xCD, 0x94, 0xCD, 0xDF};
  uint8_t  LoRaMacAppSKeyInit[] = {0x54, 0xB1, 0x3C, 0x89, 0x1D, 0x94, 0x00, 0x36, 0x36, 0xD6, 0x09, 0x6D, 0x4D, 0x9D, 0xE8, 0x6D}; //0x6D, 0x86, 0x11, 0xEE, 0x20, 0x13, 0x73, 0x61, 0xC1, 0x56, 0xBB, 0x4D, 0x15, 0x65, 0xFF, 0x92};
  uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x30, 0xE1}; //0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x09, 0x04};
  //uint32_t LoRaDevAddrInit = 0x260BE244; //260B1B80; //from original file
#endif

  //set 1
  #if plantsensor_s1_01
    uint8_t  LoRaMacNwkSKeyInit[] = {0x5C, 0x8F, 0x01, 0xBC, 0x76, 0x04, 0xC2, 0x32, 0x82, 0xF3, 0x62, 0xF7, 0xB5, 0xB3, 0xAA, 0xCF};
    uint8_t  LoRaMacAppSKeyInit[] = {0x6B, 0xBD, 0xF0, 0x35, 0xB3, 0xCC, 0xA3, 0x30, 0x68, 0x50, 0xE2, 0x7A, 0x94, 0x2F, 0x84, 0x0B};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB1};
    //uint32_t LoRaDevAddrInit = 0x260B8764;
  #endif
  #if plantsensor_s1_02
    uint8_t  LoRaMacNwkSKeyInit[] = {0xD7, 0x30, 0xC9, 0xB1, 0x46, 0xF9, 0xB9, 0xBE, 0xE1, 0xE6, 0xC6, 0xAD, 0x2C, 0x7C, 0xCE, 0x2F};
    uint8_t  LoRaMacAppSKeyInit[] = {0x0D, 0xE7, 0x37, 0x41, 0x24, 0x42, 0x0A, 0x94, 0x0F, 0x1A, 0xAF, 0xFA, 0xE5, 0x35, 0x87, 0xF2};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB2};
    //uint32_t LoRaDevAddrInit = 0x260B7A8F;
  #endif
  #if plantsensor_s1_03
    uint8_t  LoRaMacNwkSKeyInit[] = {0x45, 0xBC, 0xDD, 0x6D, 0x26, 0x91, 0xB3, 0xE5, 0x59, 0x2C, 0x5C, 0x6E, 0x7C, 0x69, 0x93, 0xC5};
    uint8_t  LoRaMacAppSKeyInit[] = {0x59, 0xC0, 0x9C, 0x8A, 0x4C, 0x65, 0x10, 0xAE, 0x6A, 0x18, 0x0D, 0x75, 0xF8, 0x3D, 0xFF, 0xE8};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB4};
    //uint32_t LoRaDevAddrInit = 0x260BF8D3;
  #endif
  #if plantsensor_s1_04
    uint8_t  LoRaMacNwkSKeyInit[] = {0x75, 0xBC, 0x14, 0x18, 0x94, 0xB9, 0x77, 0xDF, 0x76, 0xDE, 0x02, 0xBA, 0xE2, 0xAE, 0x3C, 0x23};
    uint8_t  LoRaMacAppSKeyInit[] = {0xC5, 0x68, 0x15, 0x80, 0x79, 0x6E, 0x53, 0xE7, 0x61, 0x8A, 0x04, 0x70, 0x81, 0x10, 0xC8, 0x85};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB5};
    //uint32_t LoRaDevAddrInit = 0x260BD489;
  #endif
  #if plantsensor_s1_05
    uint8_t  LoRaMacNwkSKeyInit[] = {0x29, 0x82, 0xAC, 0x2E, 0xDC, 0x8B, 0x3B, 0xE7, 0x23, 0x54, 0x44, 0x85, 0x24, 0x31, 0x77, 0xD8};
    uint8_t  LoRaMacAppSKeyInit[] = {0x69, 0x94, 0xEF, 0xAB, 0x2B, 0xF0, 0xA6, 0x48, 0x9E, 0xDA, 0xA6, 0xAC, 0xA5, 0x09, 0x45, 0xB2};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB6};
    //uint32_t LoRaDevAddrInit = 0x260BA1BC;
  #endif
  #if plantsensor_s1_06
    uint8_t  LoRaMacNwkSKeyInit[] = {0x4D, 0xFC, 0xC5, 0x4C, 0x82, 0x90, 0x67, 0x16, 0x1E, 0x27, 0x21, 0xAC, 0x63, 0xFB, 0x5E, 0xF2};
    uint8_t  LoRaMacAppSKeyInit[] = {0xC0, 0xE1, 0x50, 0x10, 0xE3, 0xB6, 0xF5, 0x88, 0x0C, 0xEA, 0x7A, 0x6E, 0x26, 0xD6, 0xD7, 0x6D};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB7};
    //uint32_t LoRaDevAddrInit = 0x260B91D4;
  #endif
  #if plantsensor_s1_07
    uint8_t  LoRaMacNwkSKeyInit[] = {0x1F, 0xCF, 0x14, 0xA2, 0xAF, 0xD1, 0xE8, 0xAE, 0x04, 0x78, 0x07, 0x5A, 0xAC, 0x6E, 0x03, 0xC2};
    uint8_t  LoRaMacAppSKeyInit[] = {0xCC, 0x96, 0xCC, 0xBA, 0xCF, 0x98, 0xE0, 0xE3, 0xE4, 0xC4, 0xB0, 0x15, 0xD5, 0xB6, 0xF9, 0x5A};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB8};
    //uint32_t LoRaDevAddrInit = 0x260B4E21;
  #endif
  #if plantsensor_s1_08
    uint8_t  LoRaMacNwkSKeyInit[] = {0x8D, 0x5C, 0x7C, 0x5F, 0x3C, 0x9B, 0x9A, 0x42, 0xF8, 0x31, 0x47, 0xAD, 0x7B, 0x64, 0x09, 0x63};
    uint8_t  LoRaMacAppSKeyInit[] = {0x36, 0x6F, 0x28, 0x31, 0x4F, 0x7E, 0x9E, 0x4B, 0x3E, 0x24, 0x8C, 0x5E, 0x39, 0x93, 0xAF, 0x43};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xB9};
    //uint32_t LoRaDevAddrInit = 0x260B71CB;
  #endif
  #if plantsensor_s1_09
    uint8_t  LoRaMacNwkSKeyInit[] = {0x50, 0x19, 0x20, 0x25, 0x93, 0xE3, 0x0E, 0x83, 0x4C, 0x82, 0x0D, 0xFB, 0x18, 0xC5, 0x79, 0xC2};
    uint8_t  LoRaMacAppSKeyInit[] = {0xA9, 0x03, 0xD2, 0x5F, 0xF7, 0xE6, 0x6D, 0x70, 0x15, 0x80, 0xC1, 0x01, 0x63, 0x6C, 0xED, 0xD2};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xBA};
    //uint32_t LoRaDevAddrInit = 0x260BE4E9;
  #endif
  #if plantsensor_s1_10
    uint8_t  LoRaMacNwkSKeyInit[] = {0x62, 0x42, 0xD1, 0x56, 0xF7, 0xFE, 0xA6, 0x7E, 0xED, 0x39, 0x93, 0xBE, 0xE7, 0x78, 0x6A, 0x95};
    uint8_t  LoRaMacAppSKeyInit[] = {0xC2, 0xE2, 0x23, 0xB9, 0x31, 0x16, 0xDC, 0x5A, 0x22, 0x86, 0x29, 0x70, 0x21, 0x56, 0xC5, 0xAD};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xBB};
    //uint32_t LoRaDevAddrInit = 0x260BC643;
  #endif
  #if plantsensor_s1_11
    uint8_t  LoRaMacNwkSKeyInit[] = {0x1C, 0x83, 0x15, 0x7A, 0x73, 0x2F, 0x5A, 0x1A, 0x05, 0x1F, 0x32, 0x0E, 0x5A, 0x0B, 0xE8, 0xAD};
    uint8_t  LoRaMacAppSKeyInit[] = {0xA1, 0x0A, 0xBF, 0x95, 0x76, 0xD1, 0x46, 0xCB, 0xC2, 0x3C, 0xAC, 0x85, 0x08, 0x80, 0x8B, 0xDD};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xBC};
    //uint32_t LoRaDevAddrInit = 0x260BA322;
  #endif
  #if plantsensor_s1_12
    uint8_t  LoRaMacNwkSKeyInit[] = {0x17, 0x43, 0x17, 0x49, 0xDA, 0xA9, 0x6D, 0x18, 0xCA, 0xF1, 0x97, 0x2B, 0x20, 0xD8, 0x21, 0x10};
    uint8_t  LoRaMacAppSKeyInit[] = {0x52, 0xD0, 0x68, 0xCE, 0x1B, 0xDF, 0x2A, 0xF1, 0x73, 0x5E, 0xAE, 0x8D, 0x2C, 0x99, 0x95, 0x0C};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xBE};
    //uint32_t LoRaDevAddrInit = 0x260B514C;
  #endif
  #if plantsensor_s1_13
    uint8_t  LoRaMacNwkSKeyInit[] = {0x7C, 0xD3, 0xD1, 0xD8, 0x4F, 0x1E, 0x17, 0x9C, 0x22, 0xA6, 0xA3, 0x3F, 0x5F, 0x4A, 0x74, 0x4B};
    uint8_t  LoRaMacAppSKeyInit[] = {0x59, 0xEB, 0x5F, 0x02, 0x2E, 0x2F, 0x04, 0x16, 0x32, 0x01, 0x9B, 0xFF, 0xA1, 0x1D, 0x47, 0x37};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xBF};
    //uint32_t LoRaDevAddrInit = 0x260B6096;
  #endif
  #if plantsensor_s1_14
    uint8_t  LoRaMacNwkSKeyInit[] = {0xE3, 0xBE, 0x8C, 0x45, 0xCD, 0xF8, 0xA7, 0x94, 0xA5, 0xB1, 0xF5, 0xC0, 0x39, 0x8A, 0x49, 0x67};
    uint8_t  LoRaMacAppSKeyInit[] = {0xD9, 0x50, 0x03, 0xB4, 0x2E, 0xB5, 0xE6, 0x1F, 0xC5, 0x48, 0xC6, 0xBC, 0xFA, 0x60, 0x1C, 0xAD};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xC0};
    //uint32_t LoRaDevAddrInit = 0x260BF336;
  #endif
  #if plantsensor_s1_15
    uint8_t  LoRaMacNwkSKeyInit[] = {0xF7, 0xD4, 0xBC, 0xAA, 0xD6, 0xF2, 0xD2, 0xD2, 0x2E, 0xE5, 0xF5, 0x59, 0x71, 0xC8, 0xB3, 0x04};
    uint8_t  LoRaMacAppSKeyInit[] = {0xCC, 0x0A, 0xC8, 0xF2, 0x58, 0xA2, 0x21, 0xE6, 0xBE, 0x83, 0xD4, 0xA9, 0xF7, 0x2B, 0x1F, 0xB7};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA3, 0xC1};
    //uint32_t LoRaDevAddrInit = 0x260B440A;
  #endif
  //set 2
  #if plantsensor_s2_01
    uint8_t  LoRaMacNwkSKeyInit[] = {0xB3, 0xE3, 0x00, 0x73, 0x4B, 0xDF, 0xB8, 0xDB, 0xE5, 0x46, 0xBD, 0xD2, 0xA7, 0x82, 0xBC, 0xB5};
    uint8_t  LoRaMacAppSKeyInit[] = {0xAD, 0xB1, 0x56, 0x3D, 0x46, 0x32, 0x87, 0xC0, 0x2E, 0x75, 0xDC, 0xD6, 0xFD, 0x74, 0x73, 0x08};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x74};
    //uint32_t LoRaDevAddrInit = 0x260BD10E;
  #endif
  #if plantsensor_s2_02
    uint8_t  LoRaMacNwkSKeyInit[] = {0xBF, 0x8C, 0xF1, 0xAC, 0x82, 0xC7, 0xE9, 0x93, 0xB8, 0xC3, 0xCD, 0x93, 0xA6, 0x67, 0x91, 0xBE};
    uint8_t  LoRaMacAppSKeyInit[] = {0x92, 0x9E, 0x6B, 0x86, 0x9E, 0xA6, 0xBE, 0x13, 0x91, 0x6C, 0xB7, 0xFD, 0x21, 0x13, 0xC2, 0x7A};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x79};
    //uint32_t LoRaDevAddrInit = 0x260B23F3;
  #endif
  #if plantsensor_s2_03
    uint8_t  LoRaMacNwkSKeyInit[] = {0x47, 0xEE, 0x43, 0x4D, 0x7D, 0x15, 0x16, 0x91, 0x81, 0xC4, 0x18, 0xF4, 0xD8, 0xDE, 0xC3, 0x0D};
    uint8_t  LoRaMacAppSKeyInit[] = {0x47, 0xD1, 0x80, 0x33, 0x58, 0xE7, 0x51, 0x6E, 0xC6, 0x64, 0x1B, 0x8A, 0xE5, 0x35, 0x19, 0x58};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x7A};
    //uint32_t LoRaDevAddrInit = 0x260B4BB2;
  #endif
  #if plantsensor_s2_04
    uint8_t  LoRaMacNwkSKeyInit[] = {0x71, 0xB6, 0x09, 0x50, 0xEE, 0xA8, 0xDB, 0x49, 0x2A, 0x54, 0x46, 0xC4, 0x85, 0x01, 0x43, 0x3F};
    uint8_t  LoRaMacAppSKeyInit[] = {0xFC, 0x2B, 0x5E, 0xF2, 0xA9, 0x7A, 0xF9, 0xE1, 0xE3, 0x42, 0xDE, 0x8E, 0x03, 0xC7, 0xD9, 0x58};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x7B};
    //uint32_t LoRaDevAddrInit = 0x260B8C8A;
  #endif
  #if plantsensor_s2_05
    uint8_t  LoRaMacNwkSKeyInit[] = {0x58, 0x7F, 0xF1, 0x53, 0x9C, 0xA3, 0xED, 0xB9, 0x86, 0x8B, 0x86, 0x77, 0xA6, 0x75, 0xD3, 0x5B};
    uint8_t  LoRaMacAppSKeyInit[] = {0xA7, 0x2C, 0xAB, 0x84, 0x18, 0xAE, 0x7A, 0x41, 0xEB, 0x39, 0x4F, 0x18, 0xF9, 0x02, 0x9B, 0xBA};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x7C};
    //uint32_t LoRaDevAddrInit = 0x260B304B;
  #endif
  #if plantsensor_s2_06
    uint8_t  LoRaMacNwkSKeyInit[] = {0x7C, 0xBD, 0xC5, 0xAF, 0x47, 0xF9, 0xAF, 0x61, 0xDB, 0x8B, 0xA0, 0x0F, 0xF1, 0xA3, 0xF1, 0x9D};
    uint8_t  LoRaMacAppSKeyInit[] = {0x9B, 0xFF, 0xC4, 0x36, 0x74, 0x54, 0x03, 0xC9, 0x87, 0x98, 0x53, 0x66, 0xE3, 0xA5, 0x6C, 0x86};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x7D};
    //uint32_t LoRaDevAddrInit = 0x260BFE32;
  #endif
  #if plantsensor_s2_07
    uint8_t  LoRaMacNwkSKeyInit[] = {0x40, 0xE8, 0x8F, 0x9D, 0x35, 0x79, 0x3A, 0x20, 0x6D, 0x98, 0x37, 0x9F, 0x28, 0xC9, 0x2B, 0xA0};
    uint8_t  LoRaMacAppSKeyInit[] = {0x12, 0x34, 0xF9, 0x64, 0xEA, 0x20, 0x9E, 0x10, 0xD0, 0x42, 0x06, 0x5F, 0xE1, 0xD0, 0x54, 0x3C};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x7E};
    //uint32_t LoRaDevAddrInit = 0x260B4EA2;
  #endif
  #if plantsensor_s2_08
    uint8_t  LoRaMacNwkSKeyInit[] = {0xD1, 0x2A, 0x74, 0x35, 0x14, 0xB6, 0x7C, 0xC6, 0x98, 0x2C, 0x04, 0x96, 0xD2, 0xC0, 0x64, 0x8A};
    uint8_t  LoRaMacAppSKeyInit[] = {0xF6, 0x76, 0xFE, 0xC7, 0x1F, 0x6E, 0xA1, 0x30, 0x66, 0x28, 0xE1, 0x38, 0x1C, 0x7B, 0xC3, 0x60};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x7F};
    //uint32_t LoRaDevAddrInit = 0x260BE59C;
  #endif
  #if plantsensor_s2_09
    uint8_t  LoRaMacNwkSKeyInit[] = {0xCD, 0x7D, 0x9D, 0x3F, 0xEA, 0xF5, 0x94, 0x7D, 0xCD, 0x43, 0x0F, 0xA6, 0x1A, 0x23, 0x8A, 0x64};
    uint8_t  LoRaMacAppSKeyInit[] = {0x72, 0x2A, 0x21, 0xDC, 0x72, 0x58, 0x61, 0x6C, 0x3D, 0xFE, 0xF9, 0x32, 0xAA, 0x45, 0x67, 0xF2};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x80};
    //uint32_t LoRaDevAddrInit = 0x260B9044;
  #endif
  #if plantsensor_s2_10
    uint8_t  LoRaMacNwkSKeyInit[] = {0xE7, 0x09, 0xFA, 0xC9, 0xD8, 0xB8, 0xB4, 0x6C, 0x3F, 0xF1, 0xB0, 0x50, 0x25, 0x27, 0xF5, 0x78};
    uint8_t  LoRaMacAppSKeyInit[] = {0x69, 0xB4, 0xBB, 0x30, 0xE6, 0xDE, 0x9C, 0xCF, 0x89, 0x1C, 0x86, 0x25, 0xA4, 0xF9, 0x11, 0x87};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x81};
    //uint32_t LoRaDevAddrInit = 0x260B1BA8;
  #endif
  #if plantsensor_s2_11
    uint8_t  LoRaMacNwkSKeyInit[] = {0xA4, 0xAC, 0x2A, 0xA0, 0x87, 0xD5, 0x32, 0x04, 0x19, 0x41, 0x98, 0x38, 0xA0, 0xEC, 0x6A, 0x96};
    uint8_t  LoRaMacAppSKeyInit[] = {0xF6, 0x6D, 0x62, 0x03, 0x1D, 0x05, 0x74, 0x8A, 0x04, 0x84, 0x2E, 0x6F, 0xD7, 0xE7, 0xE7, 0xD3};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x82};
    //uint32_t LoRaDevAddrInit = 0x260B87FD;
  #endif
  #if plantsensor_s2_12
    uint8_t  LoRaMacNwkSKeyInit[] = {0x1A, 0x3F, 0x69, 0x81, 0xBD, 0xB3, 0xCE, 0xB3, 0xE1, 0xF2, 0x34, 0x3D, 0x38, 0xC8, 0xD3, 0x78};
    uint8_t  LoRaMacAppSKeyInit[] = {0x4D, 0x91, 0x39, 0x8D, 0x58, 0x2B, 0x55, 0x3C, 0xA6, 0x72, 0xB7, 0x8C, 0x90, 0x69, 0x69, 0xA9};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x83};
    //uint32_t LoRaDevAddrInit = 0x260BFF7F;
  #endif
  #if plantsensor_s2_13
    uint8_t  LoRaMacNwkSKeyInit[] = {0xB6, 0x2C, 0x9E, 0x13, 0x32, 0xEC, 0x05, 0xEE, 0x67, 0x76, 0xFD, 0x29, 0x50, 0x10, 0xA2, 0xD6};
    uint8_t  LoRaMacAppSKeyInit[] = {0x40, 0x08, 0x97, 0xB7, 0xE6, 0x95, 0xBD, 0x1B, 0x4F, 0x66, 0x55, 0xB7, 0x39, 0x3E, 0x24, 0xEF};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x84};
    //uint32_t LoRaDevAddrInit = 0x260B4B53;
  #endif
  #if plantsensor_s2_14
    uint8_t  LoRaMacNwkSKeyInit[] = {0x1F, 0x14, 0x2B, 0xA2, 0x8E, 0x6C, 0x60, 0xE8, 0x34, 0xF5, 0x82, 0xF3, 0x94, 0x63, 0xEE, 0xFE};
    uint8_t  LoRaMacAppSKeyInit[] = {0xA4, 0x1F, 0xEB, 0xB8, 0x4E, 0xA1, 0x02, 0x0B, 0xE8, 0xBB, 0x5B, 0x42, 0xAE, 0x96, 0xAF, 0x56};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x85};
    //uint32_t LoRaDevAddrInit = 0x260BCD20;
  #endif
  #if plantsensor_s2_15
    uint8_t  LoRaMacNwkSKeyInit[] = {0x1C, 0x73, 0xA1, 0x44, 0xCF, 0x97, 0xC1, 0xD2, 0xAD, 0x46, 0xF0, 0x97, 0xD7, 0x7C, 0xC4, 0x65};
    uint8_t  LoRaMacAppSKeyInit[] = {0xAD, 0xD8, 0xEB, 0xD7, 0x5C, 0x60, 0xD2, 0x17, 0x8C, 0x9F, 0x63, 0x8B, 0xD0, 0x6C, 0x0C, 0x4C};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x86};
    //uint32_t LoRaDevAddrInit = 0x260B93AA;
  #endif
  //set 3
  #if plantsensor_s3_01
    uint8_t  LoRaMacNwkSKeyInit[] = {0x74, 0xC2, 0x55, 0x73, 0x86, 0x36, 0x68, 0x2E, 0x8A, 0x77, 0x4F, 0x7B, 0xD6, 0xC6, 0x01, 0x06};
    uint8_t  LoRaMacAppSKeyInit[] = {0x68, 0x6B, 0x41, 0xE7, 0x60, 0xAC, 0x8A, 0x79, 0x11, 0x7F, 0x8C, 0xEF, 0x20, 0xC3, 0xD9, 0x5D};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x98};
    //uint32_t LoRaDevAddrInit = 0x260B97D2;
  #endif
  #if plantsensor_s3_02
    uint8_t  LoRaMacNwkSKeyInit[] = {0xE7, 0x26, 0xE0, 0x6A, 0x09, 0xFA, 0xC0, 0xEA, 0x8B, 0x8B, 0xFD, 0xA6, 0x70, 0x45, 0xB1, 0xA9};
    uint8_t  LoRaMacAppSKeyInit[] = {0x64, 0x3E, 0x49, 0xA8, 0x68, 0xE8, 0xFA, 0x83, 0xAD, 0xC1, 0x00, 0x46, 0xD0, 0x9E, 0x63, 0xAB};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x9A};
    //uint32_t LoRaDevAddrInit = 0x260BCFA9;
  #endif
  #if plantsensor_s3_03
    uint8_t  LoRaMacNwkSKeyInit[] = {0x14, 0xA5, 0xD5, 0x54, 0x74, 0xB2, 0x87, 0x6F, 0x37, 0x3F, 0x28, 0xD9, 0x54, 0xD1, 0x15, 0x26};
    uint8_t  LoRaMacAppSKeyInit[] = {0x5F, 0x39, 0x87, 0xBC, 0xBC, 0x97, 0x6C, 0xEC, 0xDE, 0xB7, 0x7C, 0xFB, 0xEC, 0x42, 0xAB, 0x6A};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x9B};
    //uint32_t LoRaDevAddrInit = 0x260BA56D;
  #endif
  #if plantsensor_s3_04
    uint8_t  LoRaMacNwkSKeyInit[] = {0x46, 0x40, 0x3B, 0x05, 0x96, 0xD8, 0x5C, 0xCA, 0x97, 0x37, 0x17, 0x14, 0xAA, 0x37, 0xA3, 0xC9};
    uint8_t  LoRaMacAppSKeyInit[] = {0x75, 0xC7, 0xC9, 0xE3, 0x7D, 0xFF, 0xC5, 0x05, 0xB7, 0xAD, 0xBB, 0x8A, 0x79, 0x9E, 0x2F, 0x23};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x9C};
    //uint32_t LoRaDevAddrInit = 0x260B4CE1;
  #endif
  #if plantsensor_s3_05
    uint8_t  LoRaMacNwkSKeyInit[] = {0xB7, 0xA3, 0xB6, 0xCB, 0x9A, 0xDA, 0xB8, 0x06, 0xD1, 0xDB, 0xD9, 0x02, 0xDA, 0x07, 0x76, 0x8F};
    uint8_t  LoRaMacAppSKeyInit[] = {0xF1, 0x62, 0x18, 0xC9, 0x7B, 0x86, 0xCB, 0x67, 0x85, 0xE3, 0x23, 0xCA, 0x31, 0x14, 0xD8, 0x25};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x9D};
    //uint32_t LoRaDevAddrInit = 0x260B89B3;
  #endif
  #if plantsensor_s3_06
    uint8_t  LoRaMacNwkSKeyInit[] = {0xE3, 0x8C, 0xAD, 0xC1, 0xD3, 0x36, 0x0E, 0xFD, 0x5A, 0x60, 0xFB, 0x97, 0xE5, 0x88, 0x0B, 0x3E};
    uint8_t  LoRaMacAppSKeyInit[] = {0x17, 0x09, 0x1B, 0x66, 0x98, 0x7C, 0xCE, 0x91, 0xE7, 0x16, 0x73, 0x09, 0xA6, 0xB1, 0x01, 0x3B};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x9E};
    //uint32_t LoRaDevAddrInit = 0x260BB25D;
  #endif
  #if plantsensor_s3_07
    uint8_t  LoRaMacNwkSKeyInit[] = {0xBD, 0xDD, 0x7B, 0x65, 0x68, 0x78, 0xC3, 0x21, 0x2D, 0x57, 0x4C, 0xAC, 0x40, 0xB7, 0x06, 0xCA};
    uint8_t  LoRaMacAppSKeyInit[] = {0xC6, 0xAF, 0x0A, 0x0E, 0x58, 0x6F, 0x22, 0x23, 0xC1, 0xC4, 0x89, 0x38, 0x58, 0xCD, 0x9D, 0xDA};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0x9F};
    //uint32_t LoRaDevAddrInit = 0x260B89C4;
  #endif
  #if plantsensor_s3_08
    uint8_t  LoRaMacNwkSKeyInit[] = {0x15, 0x51, 0xA7, 0xB7, 0x05, 0xE9, 0x68, 0x78, 0xEE, 0xEC, 0xC1, 0x0C, 0x67, 0x35, 0xCF, 0xA2};
    uint8_t  LoRaMacAppSKeyInit[] = {0x2F, 0xD9, 0xB8, 0xF7, 0xC4, 0xFA, 0x6D, 0x6C, 0x2E, 0x97, 0xB3, 0x67, 0xB8, 0xD6, 0xEF, 0x2E};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA0};
    //uint32_t LoRaDevAddrInit = 0x260B276E;
  #endif
  #if plantsensor_s3_09
    uint8_t  LoRaMacNwkSKeyInit[] = {0xE8, 0x79, 0x94, 0x4F, 0x46, 0x57, 0x4A, 0x52, 0xC6, 0x19, 0xCC, 0xB4, 0xB5, 0x39, 0xDB, 0xCA};
    uint8_t  LoRaMacAppSKeyInit[] = {0x3A, 0x6A, 0xBE, 0x54, 0xBC, 0x05, 0x3C, 0xE2, 0x77, 0x24, 0xF6, 0x9D, 0x27, 0xC6, 0xED, 0x62};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA2};
    //uint32_t LoRaDevAddrInit = 0x260B8495;
  #endif
  #if plantsensor_s3_10
    uint8_t  LoRaMacNwkSKeyInit[] = {0x2E, 0x5B, 0x20, 0x9F, 0xD0, 0xF2, 0x05, 0x60, 0x77, 0x3A, 0xC7, 0xB0, 0xAB, 0xE0, 0xAE, 0x51};
    uint8_t  LoRaMacAppSKeyInit[] = {0xBD, 0x65, 0xAB, 0x97, 0x8B, 0xD8, 0x02, 0x87, 0x9E, 0x0C, 0xEB, 0x5C, 0x91, 0xBA, 0x74, 0x83};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA3};
    //uint32_t LoRaDevAddrInit = 0x260BAB7E;
  #endif
  #if plantsensor_s3_11
    uint8_t  LoRaMacNwkSKeyInit[] = {0xF2, 0x76, 0xB2, 0x22, 0x4B, 0xF1, 0xDB, 0x36, 0x73, 0x01, 0x6F, 0x61, 0x43, 0x5F, 0x8D, 0xBB};
    uint8_t  LoRaMacAppSKeyInit[] = {0x8C, 0x8C, 0xC8, 0xB0, 0xCE, 0xCF, 0x56, 0x5B, 0x0F, 0x0B, 0xF4, 0x08, 0x83, 0x5E, 0xB0, 0x30};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA4};
    //uint32_t LoRaDevAddrInit = 0x260B4F71;
  #endif
  #if plantsensor_s3_12
    uint8_t  LoRaMacNwkSKeyInit[] = {0x23, 0x0B, 0xDB, 0x88, 0x90, 0xBC, 0xCA, 0xE8, 0xC8, 0x0C, 0x63, 0x50, 0xCA, 0x0B, 0x4E, 0x49};
    uint8_t  LoRaMacAppSKeyInit[] = {0x9F, 0x3E, 0xB5, 0x1A, 0xDB, 0xC3, 0xCC, 0xC0, 0x20, 0xA9, 0x64, 0x1A, 0xCD, 0x0B, 0x45, 0xB7};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA5};
    //uint32_t LoRaDevAddrInit = 0x260B81BF;
  #endif
  #if plantsensor_s3_13
    uint8_t  LoRaMacNwkSKeyInit[] = {0xD2, 0x5C, 0xC4, 0x43, 0x1A, 0xF9, 0x0B, 0x48, 0x6E, 0xBD, 0x7A, 0x5C, 0xE8, 0x98, 0x41, 0xDD};
    uint8_t  LoRaMacAppSKeyInit[] = {0x9C, 0x4F, 0x55, 0xE5, 0xB7, 0x70, 0xA7, 0x35, 0x75, 0x66, 0x56, 0xF8, 0xCD, 0x31, 0xD4, 0x6D};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA6};
    //uint32_t LoRaDevAddrInit = 0x260BA4AD;
  #endif
  #if plantsensor_s3_14
    uint8_t  LoRaMacNwkSKeyInit[] = {0x27, 0xCE, 0xB3, 0xE2, 0x34, 0xD0, 0x72, 0xBD, 0xA6, 0x92, 0x34, 0xA0, 0x42, 0x59, 0xB1, 0x2E};
    uint8_t  LoRaMacAppSKeyInit[] = {0x0A, 0x52, 0xF2, 0xAA, 0x4E, 0x70, 0xA5, 0x60, 0xCB, 0xFD, 0x4C, 0xA4, 0x55, 0xE6, 0xE9, 0x22};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA7};
    //uint32_t LoRaDevAddrInit = 0x260B7CBA;
  #endif
  #if plantsensor_s3_15
    uint8_t  LoRaMacNwkSKeyInit[] = {0xD7, 0x14, 0x80, 0xCE, 0xE2, 0x33, 0xD7, 0x2B, 0x60, 0xF4, 0x04, 0xEA, 0x2F, 0x0F, 0x65, 0xF4};
    uint8_t  LoRaMacAppSKeyInit[] = {0xE5, 0x16, 0x48, 0x44, 0x7C, 0x6D, 0x6A, 0x6B, 0x76, 0xB7, 0xBF, 0xB2, 0x21, 0x51, 0x0B, 0x10};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA8};
    //uint32_t LoRaDevAddrInit = 0x260B0F5D;
  #endif
  //set 4
  #if plantsensor_s4_01
    uint8_t  LoRaMacNwkSKeyInit[] = {0x00, 0x64, 0xFD, 0x6E, 0x0F, 0xFA, 0x3D, 0xD1, 0x4D, 0x20, 0x47, 0x8E, 0x1A, 0xF0, 0x7C, 0x92};
    uint8_t  LoRaMacAppSKeyInit[] = {0x03, 0x7F, 0xF4, 0x9F, 0xF2, 0x1C, 0x2F, 0xDA, 0x44, 0xDD, 0xB9, 0xC3, 0x75, 0x26, 0x75, 0xAC};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xA9};
    //uint32_t LoRaDevAddrInit = 0x260B3C1B;
  #endif
  #if plantsensor_s4_02
    uint8_t  LoRaMacNwkSKeyInit[] = {0x8C, 0x51, 0xE9, 0x54, 0x90, 0xF1, 0x0B, 0xB6, 0x7B, 0x80, 0xE9, 0xDA, 0xF7, 0xC2, 0xF2, 0x45};
    uint8_t  LoRaMacAppSKeyInit[] = {0xC2, 0x36, 0x69, 0xD3, 0x00, 0x33, 0x7C, 0x12, 0x8B, 0x62, 0x36, 0xB8, 0x5B, 0x8F, 0x75, 0x9E};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xAA};
    //uint32_t LoRaDevAddrInit = 0x260BE04C;
  #endif
  #if plantsensor_s4_03
    uint8_t  LoRaMacNwkSKeyInit[] = {0x74, 0x3F, 0x92, 0xAB, 0xF0, 0x0E, 0x78, 0x6F, 0xFF, 0xA4, 0x31, 0x9B, 0xDD, 0x31, 0x32, 0x74};
    uint8_t  LoRaMacAppSKeyInit[] = {0x73, 0x6E, 0xCF, 0x93, 0x92, 0x25, 0x64, 0xA2, 0x78, 0x16, 0x4A, 0x31, 0x4E, 0xBE, 0x4E, 0x4E};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xAB};
    //uint32_t LoRaDevAddrInit = 0x260B8032;
  #endif
  #if plantsensor_s4_04
    uint8_t  LoRaMacNwkSKeyInit[] = {0x2D, 0x12, 0x3F, 0x59, 0x67, 0xDA, 0x5C, 0xF9, 0x04, 0x02, 0x79, 0x33, 0x83, 0x53, 0x37, 0x9B};
    uint8_t  LoRaMacAppSKeyInit[] = {0x81, 0xD4, 0x61, 0xC6, 0x12, 0x7D, 0x16, 0x99, 0x0E, 0xA1, 0x64, 0xA5, 0x14, 0xC6, 0x0C, 0x50};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xAC};
    //uint32_t LoRaDevAddrInit = 0x260BA2BC;
  #endif
  #if plantsensor_s4_05
    uint8_t  LoRaMacNwkSKeyInit[] = {0xA1, 0x8B, 0x66, 0x0B, 0x27, 0x93, 0x1F, 0x8E, 0x15, 0x0B, 0x78, 0x43, 0x04, 0x31, 0x75, 0xAA};
    uint8_t  LoRaMacAppSKeyInit[] = {0xEA, 0x28, 0x9C, 0x3D, 0xA1, 0x41, 0xB1, 0x9B, 0x91, 0x13, 0xEB, 0x8F, 0x4E, 0x67, 0x1B, 0x35};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xAD};
    //uint32_t LoRaDevAddrInit = 0x260B2810;
  #endif
  #if plantsensor_s4_06
    uint8_t  LoRaMacNwkSKeyInit[] = {0x6D, 0xA0, 0xC3, 0xCC, 0x10, 0x53, 0x21, 0x07, 0x16, 0x07, 0x23, 0x88, 0x23, 0x33, 0x4A, 0x8D};
    uint8_t  LoRaMacAppSKeyInit[] = {0x86, 0x9E, 0x61, 0x25, 0xB5, 0x92, 0x53, 0x1A, 0xD9, 0x2A, 0xA6, 0x81, 0xA5, 0xA5, 0x40, 0xE1};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xAE};
    //uint32_t LoRaDevAddrInit = 0x260B3104;
  #endif
  #if plantsensor_s4_07
    uint8_t  LoRaMacNwkSKeyInit[] = {0x9B, 0x46, 0x8C, 0x54, 0xB9, 0x1F, 0xE5, 0x98, 0x09, 0x89, 0x0F, 0x4B, 0x32, 0xF1, 0x44, 0x5C};
    uint8_t  LoRaMacAppSKeyInit[] = {0x63, 0xDC, 0x28, 0x32, 0x03, 0x28, 0x3F, 0x63, 0xC1, 0xFD, 0x02, 0x84, 0x64, 0xE7, 0xFA, 0xB6};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xAF};
    //uint32_t LoRaDevAddrInit = 0x260B77FB;
  #endif
  #if plantsensor_s4_08
    uint8_t  LoRaMacNwkSKeyInit[] = {0x3D, 0xCC, 0x42, 0x27, 0x5E, 0xC9, 0xF9, 0xE6, 0x37, 0x97, 0xA1, 0xAE, 0x92, 0x62, 0xE2, 0x1D};
    uint8_t  LoRaMacAppSKeyInit[] = {0x60, 0xD1, 0x9A, 0xEE, 0x7E, 0x61, 0xD3, 0xCC, 0x86, 0xEE, 0x4B, 0x95, 0x95, 0xC7, 0x33, 0x37};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB0};
    //uint32_t LoRaDevAddrInit = 0x260BE17B;
  #endif
  #if plantsensor_s4_09
    uint8_t  LoRaMacNwkSKeyInit[] = {0x24, 0x1D, 0x82, 0xC2, 0xCF, 0x0C, 0xF6, 0x63, 0x9B, 0x40, 0x83, 0xB1, 0x8F, 0x36, 0x87, 0x0C};
    uint8_t  LoRaMacAppSKeyInit[] = {0xA8, 0xB2, 0xC6, 0x02, 0x39, 0x45, 0xA1, 0x9C, 0x9E, 0x0F, 0x83, 0xCC, 0xF7, 0x56, 0x59, 0x7E};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB1};
    //uint32_t LoRaDevAddrInit = 0x260BD596;
  #endif
  #if plantsensor_s4_10
    uint8_t  LoRaMacNwkSKeyInit[] = {0x72, 0x83, 0x31, 0x04, 0xBB, 0xBA, 0xFF, 0xF5, 0x76, 0xB6, 0xDF, 0x19, 0x91, 0x8F, 0x2A, 0x9C};
    uint8_t  LoRaMacAppSKeyInit[] = {0xE6, 0x62, 0x18, 0x8A, 0xF4, 0x1E, 0x24, 0xB7, 0xDB, 0x81, 0xAF, 0x84, 0x67, 0x66, 0x4A, 0x0E};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB2};
    //uint32_t LoRaDevAddrInit = 0x260B316C;
  #endif
  #if plantsensor_s4_11
    uint8_t  LoRaMacNwkSKeyInit[] = {0x4F, 0x0D, 0xD8, 0x23, 0x20, 0x6D, 0x1F, 0x2D, 0xC7, 0xEB, 0x7A, 0xB0, 0x14, 0x6C, 0x54, 0xFC};
    uint8_t  LoRaMacAppSKeyInit[] = {0x7E, 0x8C, 0x07, 0x18, 0xF4, 0x96, 0x85, 0x24, 0x4D, 0x74, 0x49, 0xE5, 0xC7, 0x13, 0x4D, 0x78};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB3};
    //uint32_t LoRaDevAddrInit = 0x260B2E39;
  #endif
  #if plantsensor_s4_12
    uint8_t  LoRaMacNwkSKeyInit[] = {0xDC, 0xB1, 0x4A, 0xA3, 0x3A, 0xC0, 0xB1, 0x59, 0xE3, 0xA6, 0xB8, 0xE8, 0xC7, 0xDF, 0x06, 0xDE};
    uint8_t  LoRaMacAppSKeyInit[] = {0xAF, 0x18, 0xAF, 0x2B, 0xFD, 0x2E, 0x59, 0x4C, 0x24, 0x37, 0xE3, 0x6E, 0x63, 0xA9, 0x19, 0x1D};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB4};
    //uint32_t LoRaDevAddrInit = 0x260B70B8;
  #endif
  #if plantsensor_s4_13
    uint8_t  LoRaMacNwkSKeyInit[] = {0x37, 0x54, 0xA5, 0xA5, 0xE2, 0x10, 0xB9, 0xEE, 0xDD, 0x2A, 0xD8, 0xB5, 0xB8, 0x6C, 0xD3, 0x5D};
    uint8_t  LoRaMacAppSKeyInit[] = {0x65, 0xA0, 0xAC, 0x19, 0x55, 0x73, 0x79, 0x60, 0xA0, 0x23, 0x2F, 0xD8, 0x4C, 0xCE, 0x49, 0xD4};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB5};
    //uint32_t LoRaDevAddrInit = 0x260B4A94;
  #endif
  #if plantsensor_s4_14
    uint8_t  LoRaMacNwkSKeyInit[] = {0x5F, 0xED, 0x6A, 0x50, 0x0D, 0x7B, 0xF6, 0xB7, 0x02, 0x97, 0x25, 0x7D, 0x95, 0xC9, 0xD0, 0xFB};
    uint8_t  LoRaMacAppSKeyInit[] = {0x6F, 0xA5, 0x5F, 0x08, 0x2C, 0x62, 0x09, 0xEF, 0xE7, 0x41, 0xFD, 0x79, 0x4A, 0x32, 0x82, 0xD8};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB6};
    //uint32_t LoRaDevAddrInit = 0x260B8AD1;
  #endif
  #if plantsensor_s4_15
    uint8_t  LoRaMacNwkSKeyInit[] = {0x2A, 0x1B, 0xF0, 0x03, 0x1B, 0xB8, 0xF6, 0xBD, 0xB7, 0xD0, 0x1F, 0xC3, 0xD2, 0x91, 0xE6, 0xFD};
    uint8_t  LoRaMacAppSKeyInit[] = {0xDB, 0x63, 0xD5, 0x39, 0xB2, 0xB7, 0xFF, 0xBB, 0xA8, 0xC2, 0x63, 0x2D, 0xD8, 0x8A, 0xB1, 0x97};
    uint8_t  DevEuiInit[]         = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xA4, 0xB7};
    //uint32_t LoRaDevAddrInit = 0x260BFC3F;
  #endif



  //uint32_t   notificationValue = 0;                   // Used to identify where the notification is coming from.
  //xLastWakeTime     = xTaskGetTickCount();

  //***************************************************************************
  // LoRaWAN send one packet
  //***************************************************************************
 // lr1mac_states_t send_status;
  //static lr1_stack_mac_t  lr1_mac_obj         		= { 0 };
  //static user_rx_packet_type_t AvailableRxPacket    = NO_LORA_RXPACKET_AVAILABLE;
  //uint8_t fPort = PORTNWK; //PORTNWK = MAC Commands
//  lr1mac_core_init(&LoraWanKeys); //needed
//  lr1mac_core_dr_strategy_set( STATIC_ADR_MODE ); //data rate is changed by gateway //needed
  //MAC commands:
  //0x02: LinkCheckReq
//  uint8_t dataToSend = 0x02; //needed

  //send_status = lr1mac_core_payload_send( 104, &dataToSend , 1, UNCONF_DATA_UP, xTaskGetTickCount() + 200 ); //wordt het lr1_mac object gemaakt

  //uint32_t   notificationValue = 0;                   // Used to identify where the notification is coming from.
 //  TickType_t xLastWakeTime     = xTaskGetTickCount();
  // uint32_t   delayRtcStartup   = 0;
  // uint8_t    printExtraInfo    = 0;
  // uint8_t    spreadingFactor   = 0;
  // uint8_t    bandwidth         = 0;
  // uint8_t    codingRate        = 0;
  // uint8_t    bleChannelNr      = 0;
  // uint8_t    exchangeTrial     = 0;   // the number of host-node trial communications. Each trial, the exchangeIteration number is copied. If after 5 trials the
                                       // exchangeIteration number is not changed, then it means that the communication will not work and we go to the next settings

  //lr1mac_states_t lr1mac_core_process( AvailableRxPacket );

  //denk dat deze functie in RTOS taak moet komen
  //void lr1_stack_mac_update( lr1_stack_mac_t* lr1_mac )
   //send_status = LWPSTATE_IDLE;

   //74657374

   //example maximum payload of 22:
//   payload[ 0] = 0x01;
//   payload[ 1] = 0x02;
//   payload[ 2] = 0x03;
//   payload[ 3] = 0x04;
//   payload[ 4] = 0x05;
//   payload[ 5] = 0x06;
//   payload[ 6] = 0x07;
//   payload[ 7] = 0x08;
//   payload[ 8] = 0x09;
//   payload[ 9] = 0x10;
//   payload[10] = 0x11;
//   payload[11] = 0x12;
//   payload[12] = 0x13;
//   payload[13] = 0x14;
//   payload[14] = 0x15;
//   payload[15] = 0x16;
//   payload[16] = 0x17;
//   payload[17] = 0x18;
//   payload[18] = 0x19;
//   payload[19] = 0x20;
//   payload[20] = 0x21;
//   payload[21] = 0x22;
//
//   payloadLength = 22;

   //example with only 4 bytes:
//   payload[ 0] = 0x01;
//   payload[ 1] = 0x02;
//   payload[ 2] = 0x03;
//   payload[ 3] = 0x04;

   //74657374 = 'test'
//   payload[ 0] = 0x74;
//   payload[ 1] = 0x65;
//   payload[ 2] = 0x73;
//   payload[ 3] = 0x74;



  devAddr[ 0] = (LoRaDevAddrInit >> 24) & 0xFF;
  devAddr[ 1] = (LoRaDevAddrInit >> 16) & 0xFF;
  devAddr[ 2] = (LoRaDevAddrInit >> 8) & 0xFF;
  devAddr[ 3] = LoRaDevAddrInit & 0xFF;

  uint8_t    buf[5];
  uint8_t    rxbuf[5];

  uint32_t offTime        = 0;
  uint32_t offTimeLptimer = 0;
  uint64_t totalCycleTime = 0;
  uint64_t timeStampIMUCycleStart = 0;
  int      j              = 0;

  uint32_t   notificationValue = 0;      // Used to identify where the notification is coming from.
  uint8_t    spreadingFactor   = LORA_SF12;
  uint8_t    bandwidth         = LORA_BW_0800;
  uint8_t    codingRate        = LORA_CR_LI_4_8;
  uint8_t    bleChannelNr      = 2;

  uint32_t   pwrcr1            = 0;

  PCF2131_time_t rtc_time;

  uint32_t   regVal            = 0; // contains a register value

  uint32_t refreshIWDG = 1;

//  ScapThreadNotify(NOTIFICATION_LOAD_SCAP); // first make sure that the supercap is loaded before starting the radio:
//  uint32_t whileIterations = 0; // to prevent lock into while loop
//  xTaskNotifyStateClear(gatewayThreadHandler);
//  notificationValue = 0;
//  while ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
//  { // waiting for a notification value from the app_supercap that the super capacitor is loaded
//    xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(30000));
//    if ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) == NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [gateway] Super capacitor loaded. Ready to start.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//      if (!notificationValue)
//      {
//#if PRINTF_APP_GATEWAY
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [gateway] No notification from supercap in the last 30s.\r\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      else
//      {
//#if PRINTF_APP_GATEWAY
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [gateway] Wrong notification value received to load supercap: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      xTaskNotifyStateClear(gatewayThreadHandler);
//      notificationValue = 0;
//    }
//    if (whileIterations++ > 10)
//    {
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [gateway] Error: Jump out of while loop waiting for a notification from supercap.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//      notificationValue = NOTIFICATION_FROM_SCAP_SUPERCAP_READY;
//    }
//  }
//
//  SX1280HalReset(); // Reset the radio
//  //SX1280WaitOnBusy();
//#if PRINTF_APP_GATEWAY
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [SX1280.c] LoRa radio reset done.\r\n", (unsigned int) xTaskGetTickCount());
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  statusSX1280 = SX1280GetStatus(); // wake up radio:
//#if PRINTF_APP_GATEWAY
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [SX1280.c] LoRa radio wake-up done.\r\n", (unsigned int) xTaskGetTickCount());
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  buf[0] = 0x00; // 0x00 LDO mode - consumes more power or 0x01 DC-DC mode - consumes less power
//  SX1280HalReadCommand(RADIO_SET_REGULATORMODE, buf, 1);
//  statusSX1280 = SX1280GetStatus();
//#if PRINTF_APP_GATEWAY
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [SX1280.c] RADIO_SET_TXPARAMS send: 0x%02X, 0x%02X.\r\n",(unsigned int) xTaskGetTickCount(),buf[0],buf[1]);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  volatile uint8_t  silicon_version;
//  SX1280HalReadCommand(RADIO_GET_SILICON_VERSION, buf, 1); //returns 0x47 for new Silicon, 0x67 for original
//  silicon_version = buf[0];
//  statusSX1280 = SX1280GetStatus();
//#if PRINTF_APP_GATEWAY
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [SX1280.c] Silicon version of SX1280: 0x%02X, ",(unsigned int) xTaskGetTickCount(),(unsigned int)silicon_version);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
//  switch (silicon_version)
//  {
//  case 0x47: // new silicon
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "new silicon.\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    break;
//  case 0x67: // original silicon
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "original silicon.\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    break;
//  default:
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "unknown silicon version.\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//  }
//
////  SX1280Init();
//  GoToStandby(0);
//  vTaskDelay(200U);
//  SX1280SetSleep();
////  vTaskDelay(1000U);

  for(;;)
  { // Infinite loop
    timeStampIMUCycleStart = xTaskGetTickCount();
    //totalCycleTime = 0;
    imuMeasurementNr = 0;
    if (imuAvailable)
    {
#if !IMUTEST
      ICM_20948_sleepModeEnable(0, &device_icm20948); // wake-up IMU
#endif
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] IMU wake-up done.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      vTaskDelay(100U);
      pollIMU = 1; // now IMU can poll for new data
#if USE_RV

      // put all other tasks in suspend modus
//      vTaskSuspend(ledThreadHandler);
//      vTaskSuspend(halPPSThreadHandler);
//      vTaskSuspend(rtcThreadHandler);
//      vTaskSuspend(halSyncThreadHandler);

//      while (imuMeasurementNr < 120)
//      { // to make sure that minimum 120 IMU measurements are done
//          vTaskDelay(100U);
//      }

//      vTaskResume(halSyncThreadHandler);
//      vTaskResume(rtcThreadHandler);
//      vTaskResume(halPPSThreadHandler);
//      vTaskResume(ledThreadHandler);

      // Refreh the Watchdog
#if TESTIWDG
      if (refreshIWDG < 4)  // Only for test purposes!!!
      {
#endif
        HAL_IWDG_Refresh(&hiwdg);
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] IWDG refresh #%u given.\r\n", (unsigned int) xTaskGetTickCount(),  (unsigned int) refreshIWDG);
        huart2print(uart_buf, strlen(uart_buf));
        refreshIWDG++;
#if TESTIWDG
      }
      else
      {
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] No IWDG Refresh given, shut down activated at last refresh, EWI given at %u.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) timeOfEWDGI);
          huart2print(uart_buf, strlen(uart_buf));
      }
#endif

// alternative implementation. here the app_imu is suspended:

      while (imuMeasurementNr < 120)
      { // to make sure that minimum 120 IMU measurements are done
          xTaskNotifyStateClear(gatewayThreadHandler);
          if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(200)))
          {// No time-out
            if ((notificationValue & NOTIFICATION_FROM_IMU) == NOTIFICATION_FROM_IMU)
            { // Interrupt from INT1_ICM20948_Pin (see main HAL_GPIO_EXTI_Rising_Callback())
              PollImuDevice();
              //inv_device_poll(device); // Poll device for data
            }
            else
            {
#if PRINTF_APP_GATEWAY
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_gateway] Wrong notification value from IMU!\r\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
    //          npf_snprintf(uart_buf, 200, ".\r\n");
#endif
            }
          }
          else
          {
#if PRINTF_APP_GATEWAY
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_gateway] Time-out (>200ms) on interrupt from INT1_ICM20948_Pin!\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
            PollImuDevice();
            //inv_device_poll(device); // Poll device for data
#endif

          }

        //vTaskDelay(100U);
      }


//      typedef struct inv_sensor_config_offset {
//      	float offset[3];
//      } inv_sensor_config_offset_t;



//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_gateway] Accelerometer Bias x = %f, y = %f, z = %f.\r\n",(unsigned int) xTaskGetTickCount(), (float)accBias[0], (float)accBias[1], (float)accBias[2]);
//      huart2print(uart_buf, strlen(uart_buf));
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_gateway]     Gyrometer Bias x = %f, y = %f, z = %f.\r\n",(unsigned int) xTaskGetTickCount(), (float)gyrBias[0], (float)gyrBias[1], (float)gyrBias[2]);
//      huart2print(uart_buf, strlen(uart_buf));
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_gateway]  Magnetometer Bias x = %f, y = %f, z = %f.\r\n",(unsigned int) xTaskGetTickCount(), (float)magBias[0], (float)magBias[1], (float)magBias[2]);
//      huart2print(uart_buf, strlen(uart_buf));
//#endif


      // reading out directly the bias registers:

      uint8_t  userBank            = 0;
      uint32_t dmpRegister         = 0;
      uint8_t  dmpBias[4]          = {0};
      uint8_t  dmpRegRead          = 0;
      uint32_t dmpBiasUint32_t[12] = {0};
      uint32_t dmpBiasShifted[12]  = {0};

      select_userbank(ub_0);
      dmpRegister = ACCEL_BIAS_X;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[0] = dmpBias[0];
      dmpBiasFlash[1] = dmpBias[1];
      dmpBiasFlash[2] = dmpBias[2];
      dmpBiasFlash[3] = dmpBias[3];
      dmpBiasUint32_t[0] = (uint32_t)((int32_t)dmpBiasFlash[0] << 24) | ((int32_t)dmpBiasFlash[1] << 16) | ((int32_t)dmpBiasFlash[2] << 8) | ((int32_t)dmpBiasFlash[3]);
      dmpBiasShifted[0] = (dmpBiasUint32_t[0] >> 9);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = ACCEL_BIAS_Y;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[4] = dmpBias[0];
      dmpBiasFlash[5] = dmpBias[1];
      dmpBiasFlash[6] = dmpBias[2];
      dmpBiasFlash[7] = dmpBias[3];
      dmpBiasUint32_t[1] = (uint32_t)((int32_t)dmpBiasFlash[4] << 24) | ((int32_t)dmpBiasFlash[5] << 16) | ((int32_t)dmpBiasFlash[6] << 8) | ((int32_t)dmpBiasFlash[7]);
      dmpBiasShifted[1] = (dmpBiasUint32_t[1] >> 9);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = ACCEL_BIAS_Z;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[8] = dmpBias[0];
      dmpBiasFlash[9] = dmpBias[1];
      dmpBiasFlash[10] = dmpBias[2];
      dmpBiasFlash[11] = dmpBias[3];
      dmpBiasUint32_t[2] = (uint32_t)((int32_t)dmpBiasFlash[8] << 24) | ((int32_t)dmpBiasFlash[9] << 16) | ((int32_t)dmpBiasFlash[10] << 8) | ((int32_t)dmpBiasFlash[11]);
      dmpBiasShifted[2] = (dmpBiasUint32_t[2] >> 9);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = ACCEL_ACCURACY;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 2, dmpBias);
      dmpBiasFlash[12] = dmpBias[0];
      dmpBiasFlash[13] = dmpBias[1];
      dmpBiasUint32_t[3] = (uint32_t)((int32_t)dmpBiasFlash[12] << 8) | ((int32_t)dmpBiasFlash[13]);
      vTaskDelay(100U);

      select_userbank(ub_0);
      dmpRegister = CPASS_BIAS_X;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[14] = dmpBias[0];
      dmpBiasFlash[15] = dmpBias[1];
      dmpBiasFlash[16] = dmpBias[2];
      dmpBiasFlash[17] = dmpBias[3];
      dmpBiasUint32_t[4] = (uint32_t)((int32_t)dmpBiasFlash[14] << 24) | ((int32_t)dmpBiasFlash[15] << 16) | ((int32_t)dmpBiasFlash[16] << 8) | ((int32_t)dmpBiasFlash[17]);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = CPASS_BIAS_Y;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[18] = dmpBias[0];
      dmpBiasFlash[19] = dmpBias[1];
      dmpBiasFlash[20] = dmpBias[2];
      dmpBiasFlash[21] = dmpBias[3];
      dmpBiasUint32_t[5] = (uint32_t)((int32_t)dmpBiasFlash[18] << 24) | ((int32_t)dmpBiasFlash[19] << 16) | ((int32_t)dmpBiasFlash[20] << 8) | ((int32_t)dmpBiasFlash[21]);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = CPASS_BIAS_Z;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[22] = dmpBias[0];
      dmpBiasFlash[23] = dmpBias[1];
      dmpBiasFlash[24] = dmpBias[2];
      dmpBiasFlash[25] = dmpBias[3];
      dmpBiasUint32_t[6] = (uint32_t)((int32_t)dmpBiasFlash[22] << 24) | ((int32_t)dmpBiasFlash[23] << 16) | ((int32_t)dmpBiasFlash[24] << 8) | ((int32_t)dmpBiasFlash[25]);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = CPASS_ACCURACY;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 2, dmpBias);
      dmpBiasFlash[26] = dmpBias[0];
      dmpBiasFlash[27] = dmpBias[1];
      dmpBiasUint32_t[7] = (uint32_t)((int32_t)dmpBiasFlash[26] << 8) | ((int32_t)dmpBiasFlash[27]);
      vTaskDelay(100U);

      select_userbank(ub_0);
      dmpRegister = GYRO_BIAS_X;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[28] = dmpBias[0];
      dmpBiasFlash[29] = dmpBias[1];
      dmpBiasFlash[30] = dmpBias[2];
      dmpBiasFlash[31] = dmpBias[3];
      dmpBiasUint32_t[8] = (uint32_t)((int32_t)dmpBiasFlash[28] << 24) | ((int32_t)dmpBiasFlash[29] << 16) | ((int32_t)dmpBiasFlash[30] << 8) | ((int32_t)dmpBiasFlash[31]);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = GYRO_BIAS_Y;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[32] = dmpBias[0];
      dmpBiasFlash[33] = dmpBias[1];
      dmpBiasFlash[34] = dmpBias[2];
      dmpBiasFlash[35] = dmpBias[3];
      dmpBiasUint32_t[9] = (uint32_t)((int32_t)dmpBiasFlash[32] << 24) | ((int32_t)dmpBiasFlash[33] << 16) | ((int32_t)dmpBiasFlash[34] << 8) | ((int32_t)dmpBiasFlash[35]);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = GYRO_BIAS_Z;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 4, dmpBias);
      dmpBiasFlash[36] = dmpBias[0];
      dmpBiasFlash[37] = dmpBias[1];
      dmpBiasFlash[38] = dmpBias[2];
      dmpBiasFlash[39] = dmpBias[3];
      dmpBiasUint32_t[10] = (uint32_t)((int32_t)dmpBiasFlash[36] << 24) | ((int32_t)dmpBiasFlash[37] << 16) | ((int32_t)dmpBiasFlash[38] << 8) | ((int32_t)dmpBiasFlash[39]);
      vTaskDelay(100U);
      select_userbank(ub_0);
      dmpRegister = GYRO_ACCURACY;
      userBank = (dmpRegister >> 8);
      dmpRegRead = dmpRegister & 0xFF;
      spi_master_write_register((uint8_t)REG_MEM_BANK_SEL, 1, &userBank);
      spi_master_write_register((uint8_t)REG_MEM_START_ADDR, 1, &dmpRegRead);
      spi_master_read_register(REG_MEM_R_W, 2, dmpBias);
      dmpBiasFlash[40] = dmpBias[0];
      dmpBiasFlash[41] = dmpBias[1];
      dmpBiasUint32_t[11] = (uint32_t)((int32_t)dmpBiasFlash[40] << 8) | ((int32_t)dmpBiasFlash[41]);

      waitToPrint();
      npf_snprintf(uart_buf, 200, "\r\n%u [app_gateway] Accelerometer BIAS_X dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u, SHIFT --> %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[0], (unsigned int)dmpBiasFlash[1], (unsigned int)dmpBiasFlash[2], (unsigned int)dmpBiasFlash[3], (unsigned int)dmpBiasUint32_t[0], (unsigned int)dmpBiasShifted[0]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Accelerometer BIAS_Y dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u, SHIFT --> %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[4], (unsigned int)dmpBiasFlash[5], (unsigned int)dmpBiasFlash[6], (unsigned int)dmpBiasFlash[7], (unsigned int)dmpBiasUint32_t[1], (unsigned int)dmpBiasShifted[1]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Accelerometer BIAS_Z dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u, SHIFT --> %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[8], (unsigned int)dmpBiasFlash[9], (unsigned int)dmpBiasFlash[10], (unsigned int)dmpBiasFlash[11], (unsigned int)dmpBiasUint32_t[2], (unsigned int)dmpBiasShifted[2]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Accelerometer ACCURACY dmpAcc[0] = %u, dmpAcc[1] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[12], (unsigned int)dmpBiasFlash[13], (unsigned int)dmpBiasUint32_t[3]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Magnetometer BIAS_X dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[14], (unsigned int)dmpBiasFlash[15], (unsigned int)dmpBiasFlash[16], (unsigned int)dmpBiasFlash[17], (unsigned int)dmpBiasUint32_t[4]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Magnetometer BIAS_Y dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[18], (unsigned int)dmpBiasFlash[19], (unsigned int)dmpBiasFlash[20], (unsigned int)dmpBiasFlash[21], (unsigned int)dmpBiasUint32_t[5]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Magnetometer BIAS_Z dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[22], (unsigned int)dmpBiasFlash[23], (unsigned int)dmpBiasFlash[24], (unsigned int)dmpBiasFlash[25], (unsigned int)dmpBiasUint32_t[6]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Magnetometer ACCURACY dmpAcc[0] = %u, dmpAcc[1] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[26], (unsigned int)dmpBiasFlash[27], (unsigned int)dmpBiasUint32_t[7]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Gyroscope BIAS_X dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[28], (unsigned int)dmpBiasFlash[29], (unsigned int)dmpBiasFlash[30], (unsigned int)dmpBiasFlash[31], (unsigned int)dmpBiasUint32_t[8]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Gyroscope BIAS_Y dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[32], (unsigned int)dmpBiasFlash[33], (unsigned int)dmpBiasFlash[34], (unsigned int)dmpBiasFlash[35], (unsigned int)dmpBiasUint32_t[9]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Gyroscope BIAS_Z dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[36], (unsigned int)dmpBiasFlash[37], (unsigned int)dmpBiasFlash[38], (unsigned int)dmpBiasFlash[39], (unsigned int)dmpBiasUint32_t[10]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Gyroscope ACCURACY dmpAcc[0] = %u, dmpAcc[1] = %u, --> INT32: %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[40], (unsigned int)dmpBiasFlash[41], (unsigned int)dmpBiasUint32_t[11]);
      huart2print(uart_buf, strlen(uart_buf));

      device = inv_device_icm20948_get_base(&device_icm20948); // Simply get generic device handle from icm20948 Device
      uint32_t accOffset[3] = {0,0,0};
      uint32_t magOffset[3] = {0,0,0};
      uint32_t gyrOffset[3] = {0,0,0};

//      printReadMemsIMURegContent = 1;
      inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_OFFSET, &accOffset, sizeof(accOffset));
      inv_device_get_sensor_config(device, INV_SENSOR_TYPE_MAGNETOMETER,  INV_SENSOR_CONFIG_OFFSET, &magOffset, sizeof(magOffset));
      inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,     INV_SENSOR_CONFIG_OFFSET, &gyrOffset, sizeof(gyrOffset));
//      printReadMemsIMURegContent = 0;

#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "\r\n%u [app_gateway] Accelerometer offsets x = %u, y = %u, z = %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)accOffset[0], (unsigned int)accOffset[1], (unsigned int)accOffset[2]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway]  Magnetometer offsets x = %u, y = %u, z = %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)magOffset[0], (unsigned int)magOffset[1], (unsigned int)magOffset[2]);
      huart2print(uart_buf, strlen(uart_buf));
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway]   Gyroscope   offsets x = %u, y = %u, z = %u.\r\n",
    		  (unsigned int) xTaskGetTickCount(), (unsigned int)gyrOffset[0], (unsigned int)gyrOffset[1], (unsigned int)gyrOffset[2]);
      huart2print(uart_buf, strlen(uart_buf));
#endif


      dmpBiasFlash[42] = 0xDA;
      dmpBiasFlash[43] = 0xDA;
      // now save these values in FLASH:

      FLASH_EraseInitTypeDef flashEraseData;
      flashEraseData.TypeErase = FLASH_TYPEERASE_PAGES;
      flashEraseData.Page = 127;
      flashEraseData.NbPages = 1;
      uint32_t flashErasePageError = 0;

#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Start to flash DMP values.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif

      // write value(s) to this location:
      HAL_FLASH_Unlock();
      HAL_FLASHEx_Erase(&flashEraseData, &flashErasePageError);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FE000, (uint32_t) &dmpBiasFlash[0]); // QUADWORD = 16 bytes
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FE010, (uint32_t) &dmpBiasFlash[16]);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, 0x080FE020, (uint32_t) &dmpBiasFlash[32]);
      HAL_FLASH_Lock();

#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Flash done.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif

#if PRINTF_APP_GATEWAY
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_lgateway] Data stored to Flash: 0x",(unsigned int) xTaskGetTickCount());
  for (unsigned int i = 0; i < 48; i++)
  {
    npf_snprintf(byteString, 3, "%02X", dmpBiasFlash[i]);
    strcat(uart_buf, byteString);
  }
  strcat(uart_buf, ". \n");
  huart2print(uart_buf, strlen(uart_buf));
#endif



#endif
#if USE_ACC_MAG
      vTaskDelay(1000U);
#endif
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "\r\n%u [app_gateway] Put IMU to sleep.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      pollIMU = 0;
      vTaskDelay(100U);
#if !IMUTEST
      ICM_20948_sleepModeEnable(1, &device_icm20948); // put IMU to sleep again
#endif
      vTaskDelay(100U);
    }
    else
    {// create dummy data as there is no IMU on board
    	quatW = 1;
    	quatX = 2;
    	quatY = 3;
    	quatZ = 4;
    }
    //
    // now data is available to transmit to the Lora Gateway
    //
#if USE_ORI
    yaw /= 10;
    pitch /= 10;
    roll /= 10;
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Lora Packet Payload: yaw = %d, pitch = %d, roll = %d.\r\n",(unsigned int) xTaskGetTickCount(), yaw, pitch, roll);
    huart2print(uart_buf, strlen(uart_buf));
#endif
    payload[ 0] = (yaw >> 8) & 0xFF;
    payload[ 1] = yaw & 0xFF;
    payload[ 2] = (pitch >> 8) & 0xFF;
    payload[ 3] = pitch & 0xFF;
    payload[ 4] = (roll >> 8) & 0xFF;
    payload[ 5] = roll & 0xFF;
    payloadLength = 6;
#endif

#if USE_ACC_MAG
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Measurement: %d Time: %u, float format acc: x = %f, y = %f, z = %f, mag: x = %f, y = %f, z = %f.\r\n", (unsigned int) xTaskGetTickCount(), numberOfMeas, (unsigned int) totalCycleTime, accX, accY, accZ, magX, magY, magZ);
    huart2print(uart_buf, strlen(uart_buf));
#endif

    // Update filter with sensor data
    //MahonyAHRSupdate(0.0, 0.0, 0.0, accX, accY, accZ, magX, magY, magZ);
    MadgwickAHRSupdate(gyrX, gyrY, gyrZ, accX, accY, accZ, magX, magY, magZ);

    am_quatW = (int16_t) q[0];
    am_quatX = (int16_t) q[1];
    am_quatY = (int16_t) q[2];
    am_quatZ = (int16_t) q[3];

#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Measurement: %d Time: %u, int16_t format quat from acc and mag: w = %d, x = %d, y = %d, z = %d.\r\n", (unsigned int) xTaskGetTickCount(), numberOfMeas, (unsigned int) totalCycleTime, am_quatW, am_quatX, am_quatY, am_quatZ);
    huart2print(uart_buf, strlen(uart_buf));
#endif

#endif
#if USE_RV || USE_ACC_MAG


#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Mean value of IMU quaternions measurement: w = %d, x = %d, y = %d, z = %d.\r\n", (unsigned int) xTaskGetTickCount(), quatW, quatX, quatY, quatZ);
    huart2print(uart_buf, strlen(uart_buf));
#endif

    quatArray[ 0] = (((uint16_t) (totalCycleTime / 100)) >> 8) & 0xFF; //time 0x012C is dummy time exact 300 seconds
    quatArray[ 1] =  ((uint16_t) (totalCycleTime / 100)) & 0xFF;
    quatArray[ 2] = (quatW >> 8) & 0xFF;
    quatArray[ 3] =  quatW & 0xFF;
    quatArray[ 4] = (quatX >> 8) & 0xFF;
    quatArray[ 5] =  quatX & 0xFF;
    quatArray[ 6] = (quatY >> 8) & 0xFF;
    quatArray[ 7] =  quatY & 0xFF;
    quatArray[ 8] = (quatZ >> 8) & 0xFF;
    quatArray[ 9] =  quatZ & 0xFF;

    j = numberOfMeas * 10; //Equals 0 if first measurement, equals 4 if second measurement, equals 8 if third measurement (numberOfMeas starts at 0)
    for (int i = 0; i < 10; i++)
    {
      imuData[i+j] = quatArray[i];
    }
    numberOfMeas++;
    payloadLength = numberOfMeas * 10 + 2; // for every measurement 2 bytes time info + 8 bytes quat data plus 2 bytes for battery info
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] quatArray: 0x",(unsigned int) xTaskGetTickCount());
    for (unsigned int i = 0; i < 10; i++)
    {
      npf_snprintf(byteString, 3, "%02X", quatArray[i]);
      strcat(uart_buf, byteString);
    }
    strcat(uart_buf, ". \n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] imuData: 0x",(unsigned int) xTaskGetTickCount());
    for (unsigned int i = 0; i < payloadLength-2; i++)
    {
      npf_snprintf(byteString, 3, "%02X", imuData[i]);
      strcat(uart_buf, byteString);
    }
    strcat(uart_buf, ". \n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
#endif // USE_RV


//#if !IMUTEST // Switch off Tx process for IMU test purposes

    if (numberOfMeas == 3)
    {
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] 3rd IMU measurement set done, start Tx process.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif

      //numberOfMeas = 0;
      for (int i = 0; i < payloadLength - 2 ; i++)
      {
        payload[i] = imuData[i]; //todo: maybe we can already do this at instruction where dataImu is filled??
      }

      payload[payloadLength - 2] = (batteryInVoltageLevel >> 8) & 0xFF; //0x01; // battery info: 0x168 is dummy data for 3.60 V, on gateway divide by 100
      payload[payloadLength - 1] =  batteryInVoltageLevel       & 0xFF; //0x68; // batteryInVoltageLevel is in mV!! Divide by 1000

      notificationValue = 0; // Used to identify where the notification is coming from.
      spreadingFactor   = LORA_SF12;
      bandwidth         = LORA_BW_0800;
      codingRate        = LORA_CR_LI_4_8;
      bleChannelNr      = 2;

      if (!stopLoadingSuperCap) // when EWI, the supercap should not be loaded anymore
//      if (refreshIWDG < 4)  // Only for test purposes!
      {
#if SUPERCAPUSED
        ScapThreadNotify(NOTIFICATION_LOAD_SCAP); // first make sure that the supercap is loaded before starting the radio:
        uint32_t whileIterations = 0; // to prevent lock into while loop
        xTaskNotifyStateClear(gatewayThreadHandler);
        notificationValue = 0;
        while ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) != NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
        { // waiting for a notification value from the app_supercap that the super capacitor is loaded
          xTaskNotifyWait( 0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(30000));
          if ((notificationValue & NOTIFICATION_FROM_SCAP_SUPERCAP_READY) == NOTIFICATION_FROM_SCAP_SUPERCAP_READY)
          {
#if PRINTF_APP_GATEWAY
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [gateway] Super capacitor loaded. Ready to start Radio.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
          }
          else
          {
            if (!notificationValue)
            {
#if PRINTF_APP_GATEWAY
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [gateway] No notification from supercap in the last 30s.\r\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
#endif
            }
            else
            {
#if PRINTF_APP_GATEWAY
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [gateway] Wrong notification value received to load supercap: 0x%08X.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
              huart2print(uart_buf, strlen(uart_buf));
#endif
            }
            xTaskNotifyStateClear(gatewayThreadHandler);
            notificationValue = 0;
          }
          if (whileIterations++ > 10)
          {
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [gateway] Error: Jump out of while loop waiting for a notification from supercap.\r\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
            notificationValue = NOTIFICATION_FROM_SCAP_SUPERCAP_READY;
          }
        }

// 20241016 temporarily switched off
        //
        // switch on the super capacitor for 2 extra seconds
        //
        HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_SET);   // Supercap on
        ledFreq = 2000; // led off time 2000ms, on time 200ms
        vTaskDelay(2000);
        HAL_GPIO_WritePin(SCAP_ON_GPIO_Port, SCAP_ON_Pin, GPIO_PIN_RESET);   // Supercap off
        ledFreq = 0; // led off

#endif // SUPERCAPUSED
      }
      else
      {
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [gateway] No Supercap loading process.\r\n", (unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
      }
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [gateway] Super capacitor voltage level = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
          (unsigned int) xTaskGetTickCount(), (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
          (unsigned int) batteryInVoltageLevel, (unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
      huart2print(uart_buf, strlen(uart_buf));
#endif

#if SENDGATEWAYMESSAGE // Switch off Tx process for test purposes
      SX1280Init();

      SetTypicalRegisterSettings(LORA, 13 + payloadLength, spreadingFactor, bandwidth, codingRate, bleChannelNr);
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Typical register settings were set.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      // Set DIO interrupt request parameters
      ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt
      txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
      txBuf[1] = 0x01;  //irqMask   7:0  enable TxDone interrupt
      txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
      txBuf[3] = 0x01;  //dio1Mask  7:0  map TXDone to DIO1
      txBuf[4] = 0x00;  //dio2Mask 15:8
      txBuf[5] = 0x00;  //dio2Mask  7:0
      txBuf[6] = 0x00;  //dio3Mask 15:8
      txBuf[7] = 0x00;  //dio3Mask  7:0
      SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); //0x8D
      ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] DIO IRQ params set.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif

      //load transmit data
//    if ((LR1_countUp & 0x000000FF) == 2 || (LR1_countUp & 0x000000FF) == 3 )
//    {
//      txBuf[ 0] = 0x80; //confirmed uplink
//    }
//    else
//    {
      txBuf[ 0] = 0x40; //unconfirmed uplink
//    }
      txBuf[ 1] = devAddr[3];
      txBuf[ 2] = devAddr[2];
      txBuf[ 3] = devAddr[1];
      txBuf[ 4] = devAddr[0];
      txBuf[ 5] = 0x00; // ADR is off (0x00) //0x80; //frame control: ADR = 1, rest = 0
      if ((txBuf[5] & 0b10000000) >> 7 == 1)
      {
#if PRINTF_APP_GATEWAY
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] Adaptive Data Rate is enabled.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }
      //txBuf[ 6] = 0x02; // example = FCnt
      //txBuf[ 7] = 0x00; //= FCnt
      txBuf[ 6] =  LR1_countUp & 0x000000FF;       //0x02 = FCnt
      txBuf[ 7] = (LR1_countUp & 0x0000FF00) >> 8; //0x00 = FCnt
      txBuf[ 8] = 0x02; //fport: 1 = one measurement, 2 = three measurements
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Decrypted payload: 0x",(unsigned int) xTaskGetTickCount());
      for (unsigned int i = 0; i < payloadLength; i++)
      {
        npf_snprintf(byteString, 3, "%02X", payload[i]);
        strcat(uart_buf, byteString);
      }
      strcat(uart_buf, ". \n");
      huart2print(uart_buf, strlen(uart_buf));
      npf_snprintf(uart_buf, 200, ".\r\n");
#endif
      lora_crypto_payload_encrypt(payload, payloadLength, LoRaMacAppSKeyInit, LoRaDevAddrInit, UP_LINK, LR1_countUp, encPayload );
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] [SendLoRaWANpacket] encrypted payload: 0x",(unsigned int) xTaskGetTickCount());
      for (unsigned int i = 0; i < payloadLength; i++)
      {
        npf_snprintf(byteString, 3, "%02X", encPayload[i]);
        strcat(uart_buf, byteString);
      }
      strcat(uart_buf, ". \n");
      huart2print(uart_buf, strlen(uart_buf));
      npf_snprintf(uart_buf, 200, ".\r\n");
#endif
//    // check decryption:
//    payload[ 0] = encPayload[0];
//    payload[ 1] = encPayload[1];
//    payload[ 2] = encPayload[2];
//    payload[ 3] = encPayload[3];
//    lora_crypto_payload_decrypt(encPayload, payloadLength, LoRaMacAppSKeyInit, LoRaDevAddrInit, UP_LINK, LR1_countUp, decPayload);
//    //payload_decrypt(encPayload, payloadLength, LoRaMacAppSKeyInit, LoRaDevAddrInit, UP_LINK, LR1_count, decPayload);
//#if PRINTF_APP_GATEWAY
//     waitToPrint();
//     npf_snprintf(uart_buf, 200, "%u [app_gateway] [SendLoRaWANpacket] Check decrypted payload: 0x",(unsigned int) xTaskGetTickCount());
//     for (unsigned int i = 0; i < payloadLength; i++)
//     {
//       npf_snprintf(byteString, 3, "%02X", decPayload[i]);
//       strcat(uart_buf, byteString);
//     }
//     strcat(uart_buf, ". \n");
//     huart2print(uart_buf, strlen(uart_buf));
//     npf_snprintf(uart_buf, 200, ".\r\n");
//#endif
      for (int i = 0; i < payloadLength; i++)
      {
        txBuf[9 + i] = encPayload[i];
      }
      compute_mic(&txBuf[ 0], 9 + payloadLength, LoRaMacNwkSKeyInit, LoRaDevAddrInit, UP_LINK, LR1_countUp, &mic);

      txBuf[ 9 + payloadLength] =  mic         & 0xFF; //0x2B;
      txBuf[10 + payloadLength] = (mic >>  8)  & 0xFF; //0x11;
      txBuf[11 + payloadLength] = (mic >> 16)  & 0xFF; //0xFF;
      txBuf[12 + payloadLength] = (mic >> 24)  & 0xFF; //0x0D;
      WriteTXBuffer(0, txBuf, 13 + payloadLength);          // TX Buffer offset = 0, 20 byte payload
      radioBusy = 1;
      xTaskNotifyStateClear(gatewayThreadHandler);
      ClearIrqStatus(IRQ_RADIO_ALL); // clear the interrupt
#if PRINTF_APP_GATEWAY
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] Ready to transmit.\r\n", (unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif

      SetTx();                       // transmit message to gateway

      notificationValue = 0;
      if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2000)))
      {// No time-out
        if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
        { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
          ClearIrqStatus(IRQ_RADIO_ALL); // clear the interrupt
          HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
          vTaskDelay(50U);
          HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
#if PRINTF_APP_GATEWAY
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] [SendLoRaWANpacket] SetTX done for channel %d. Lora packet payload: ",(unsigned int) xTaskGetTickCount(), bleChannelNr);
          for (unsigned int i = 0; i < 13 + payloadLength; i++)
          {
            npf_snprintf(byteString, 3, "%02X", txBuf[i]);
            strcat(uart_buf, byteString);
          }
          strcat(uart_buf, ". \n");
          huart2print(uart_buf, strlen(uart_buf));
#endif
#if PRINTF_APP_GATEWAY
		  waitToPrint();
		  npf_snprintf(uart_buf, 200, "%u [gateway] Super capacitor voltage level after transmit = %umV, BATT = %umV, BATTIN = %umV, Core = %umV, Core temp = %u°C.\r\n",
			(unsigned int) xTaskGetTickCount(), (unsigned int) supercapVoltageLevel, (unsigned int) batteryVoltageLevel,
			(unsigned int) batteryInVoltageLevel, (unsigned int) vCoreVoltageLevel, (unsigned int) vCoreTemperature);
		  huart2print(uart_buf, strlen(uart_buf));
#endif
        }
        else
        {
#if PRINTF_APP_GATEWAY
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] Wrong notification value received during transmit: %02X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      }
      else
      {// time-out
#if PRINTF_APP_GATEWAY
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] DIOx pin time-out occurred...(>2s).\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }

#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Just after notification.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif

      radioBusy = 0;
      bleChannelNr = 2;

      // Listen after sending the broadcasted pairing message to see if someone wants to pair.
      SetTypicalRegisterSettings(LORA, 15, spreadingFactor, bandwidth, codingRate, bleChannelNr);
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Typical radio settings done.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif

      txBuf[0] = 0x08;
      txBuf[1] = LORA_PACKET_EXPLICIT; // HeaderType explicit
      txBuf[2] = payloadLength;        // PayloadLength
      txBuf[3] = LORA_CRC_ON;          // CRC enabled
      txBuf[4] = LORA_IQ_INVERTED;     // InvertIQ/chirp invert //was normal //important difference with TX
      txBuf[5] = 0x00;                 // not used
      txBuf[6] = 0x00;                 // not used
      SX1280HalWriteCommand( RADIO_SET_PACKETPARAMS, txBuf, 7 );
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Packet settings done.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      // Set DIO interrupt request parameters
      txBuf[0] = 0x40;  //irqMask  15:8  enable RxTxTimeout
      txBuf[1] = 0x02;  //irqMask   7:0  enable RxDone interrupt
      txBuf[2] = 0x40;  //dio1Mask 15:8  map RxTxTimeout to DIO1
      txBuf[3] = 0x02;  //dio1Mask  7:0  map RXDone to DIO1
      txBuf[4] = 0x00;  //dio2Mask 15:8
      txBuf[5] = 0x00;  //dio2Mask  7:0
      txBuf[6] = 0x00;  //dio3Mask 15:8
      txBuf[7] = 0x00;  //dio3Mask  7:0
      SX1280HalWriteCommand( RADIO_SET_DIOIRQPARAMS, txBuf, 8 ); //0x8D
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] DIO settings done.\r\n", (unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      ClearIrqStatus(IRQ_RADIO_ALL); //clear the interrupt

#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gateway] Listening for down link message of server.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
      notificationValue = 0;
      radioBusy = 1;
      xTaskNotifyStateClear(gatewayThreadHandler);

      SetRx(); // End device listening for downlink of server

//#if PRINTF_APP_GATEWAY
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [app_network_connect] Rx done.\r\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//

      if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, pdMS_TO_TICKS(2000)))  // Listen maximum 2s.
      {// Received message with no time-out, DIOx received
        if ((notificationValue & NOTIFICATION_FROM_DIO1) == NOTIFICATION_FROM_DIO1)
        { // Interrupt from DIOx_Pin (see main HAL_GPIO_EXTI_Callback())
          ClearIrqStatus(IRQ_RADIO_ALL);//clear the interrupt
#if PRINTF_APP_GATEWAY
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] Notification from DIO1 received - RxDone.\r\n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
#endif

//
           ReadRXBuffer(0, rxBuf, 15);

         //uint8_t rxBuf[] = {0x60, 0x80, 0x1B, 0x0B, 0x26, 0x23, 0xE6, 0x00, 0x08, 0x05, 0x06, 0xB4, 0x2D, 0xCC, 0xCA};

#if PRINTF_APP_GATEWAY
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] Payload received: ",(unsigned int) xTaskGetTickCount());
          for (unsigned int i = 0; i < 15; i++)
          {
            npf_snprintf(byteString, 3, "%02X", rxBuf[i]);
            strcat(uart_buf, byteString);
          }
          strcat(uart_buf, ".\n");
          huart2print(uart_buf, strlen(uart_buf));
#endif
          rxLength = 15;

          rxMacHeader = rxBuf[0];

          rxDevAddr[0] = rxBuf[4];
          rxDevAddr[1] = rxBuf[3];
          rxDevAddr[2] = rxBuf[2];
          rxDevAddr[3] = rxBuf[1];

          rxFCtrl = rxBuf[5];
          rxADR = (uint8_t) (rxFCtrl & 0b10000000) >> 7;
          rxADRACKReq = (rxFCtrl & 0b01000000) >> 6;
          rxACK = (rxFCtrl & 0b00100000) >> 5;
          rxClassB = (rxFCtrl & 0b00010000) >> 4;
          rxFOptsLen = rxFCtrl & 0x0F;

          if (rxFOptsLen == 0) //only data
          {
            LR1_countDown |= rxBuf[6];
            LR1_countDown |= (rxBuf[7] << 8);

        	//uint8_t rxFOpts[15]; //to do: check if correct!!

            for (int i = 0; i < 15; i++)
            {
              rxFOpts[i] = 0;
            }

            rxFPort = rxBuf[8];

            rxFRMPayloadLength = rxLength - (9 + 4);
            for (int i = 0; i < rxLength - 13; i++)
            {
              rxFRMPayload[i] = rxBuf[9 + i];
            }

            rxMic |= (rxBuf[rxLength - 4] << 24);
            rxMic |= (rxBuf[rxLength - 3] << 16);
            rxMic |= (rxBuf[rxLength - 2] << 8);
            rxMic |= rxBuf[rxLength - 1];
          }
          else //only MAC commands
          {
        	//LR1_countDown = (LR1_countDown & 0xFFFFFF00) | rxBuf[6];
        	//LR1_countDown = (LR1_countDown & 0xFFFF00FF) | (rxBuf[7] << 8);
            LR1_countDown |= rxBuf[6];
            LR1_countDown |= (rxBuf[7] << 8);

			//uint8_t rxFOpts[15];

            for (int i = 0; i < rxFOptsLen; i++)
            {
              rxFOpts[i] = rxBuf[8 + i];
            }

            rxFPort = 0; //if sending data and MAC commands, FPort is not equal to zero

            if (rxFOptsLen + 8 < rxLength - 4) //this means the packets consists of data and MAC commands
            {
              rxFPort = rxBuf[8 + rxFOptsLen];
              rxFRMPayloadLength = rxLength - (9 + rxFOptsLen + 4); //8 bytes with standard info, 4 bytes for MIC
              for (int i = 0; i < rxFRMPayloadLength; i++)
              {
                rxFRMPayload[i] = rxBuf[9 + rxFOptsLen + i];
              }
            }
            rxMic |= (rxBuf[rxLength - 4] << 24);
            rxMic |= (rxBuf[rxLength - 3] << 16);
            rxMic |= (rxBuf[rxLength - 2] << 8);
            rxMic |= rxBuf[rxLength - 1];
          }

#if PRINTF_APP_GATEWAY
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] Received payload: \n",(unsigned int) xTaskGetTickCount());
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] Length of received payload = %d\n",(unsigned int) xTaskGetTickCount(), rxLength);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] MAC Header = %02X \n",(unsigned int) xTaskGetTickCount(), rxMacHeader);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] Device addres = ",(unsigned int) xTaskGetTickCount());
          for (unsigned int i = 0; i < 4; i++)
          {
            npf_snprintf(byteString, 3, "%02X", rxDevAddr[i]);
            strcat(uart_buf, byteString);
          }
          strcat(uart_buf, "\n");
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] FCtrl = %02X\n",(unsigned int) xTaskGetTickCount(), rxFCtrl);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxADR = %d \n",(unsigned int) xTaskGetTickCount(), rxADR);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxADRACKReq = %d \n",(unsigned int) xTaskGetTickCount(), rxADRACKReq);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxACK = %d \n",(unsigned int) xTaskGetTickCount(), rxACK);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxClassB = %d \n",(unsigned int) xTaskGetTickCount(), rxClassB);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxFOptsLen = %02X \n",(unsigned int) xTaskGetTickCount(), rxFOptsLen);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          //npf_snprintf(uart_buf, 100, "%u [app_network_connect] FCntDown = %02lX %02lX = %lu \n",(unsigned int) xTaskGetTickCount(), ((LR1_countDown >> 8) & 0xFF), (LR1_countDown & 0xFF), LR1_countDown);
          npf_snprintf(uart_buf, 200, "%u [app_gateway] FCntDown = %04lX = %lu \n",(unsigned int) xTaskGetTickCount(), LR1_countDown, LR1_countDown);

          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxFOpts = ",(unsigned int) xTaskGetTickCount());
          for (unsigned int i = 0; i < rxFOptsLen; i++)
          {
            npf_snprintf(byteString, 3, "%02X", rxFOpts[i]);
            strcat(uart_buf, byteString);
          }
          strcat(uart_buf, "\n");
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxFPort = %d \n",(unsigned int) xTaskGetTickCount(), rxFPort);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxFRMPayloadLength = %d \n",(unsigned int) xTaskGetTickCount(), rxFRMPayloadLength);
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxFRMPayload = ",(unsigned int) xTaskGetTickCount());
          for (unsigned int i = 0; i < rxFRMPayloadLength; i++)
          {
            npf_snprintf(byteString, 3, "%02X", rxFRMPayload[i]);
            strcat(uart_buf, byteString);
          }
          strcat(uart_buf, "\n");
          huart2print(uart_buf, strlen(uart_buf));
          waitToPrint();
          npf_snprintf(uart_buf, 200, "%u [app_gateway] rxMIC = %08lX \n",(unsigned int) xTaskGetTickCount(), rxMic);
//        for (unsigned int i = 0; i < 4; i++)
//        {
//          npf_snprintf(byteString, 3, "%02lX", rxMic[i]);
//          strcat(uart_buf, byteString);
//        }
//        strcat(uart_buf, "\n");
          huart2print(uart_buf, strlen(uart_buf));
#endif

          //check received packet:
          if (devAddr[0] == rxDevAddr[0] && devAddr[1] == rxDevAddr[1] && devAddr[2] == rxDevAddr[2] && devAddr[3] == rxDevAddr[3])
          {
#if PRINTF_APP_GATEWAY
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_gateway] Device address is correct.\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
            if (rxMacHeader == 0b01100000 || rxMacHeader == 0b10100000)
            {
#if PRINTF_APP_GATEWAY
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_gateway] The received packet is a downlink.\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
#endif
              compute_mic(&rxBuf[ 0], rxLength - 4, LoRaMacNwkSKeyInit, LoRaDevAddrInit, DOWN_LINK, LR1_countDown, &mic);
              rxMicCheck = ((mic & 0xFF) << 24) | ((mic & 0xFF00) << 8) | ((mic & 0xFF0000) >> 8) | ((mic >> 24) & 0xFF); //todo: not clear why I have to do this???
              if (rxMic == rxMicCheck)
              {
#if PRINTF_APP_GATEWAY
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [app_gateway] Mic is correct.\n",(unsigned int) xTaskGetTickCount());
                huart2print(uart_buf, strlen(uart_buf));
#endif

#if PRINTF_APP_GATEWAY
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [app_gateway] Genuine message was received.\n",(unsigned int) xTaskGetTickCount());
                huart2print(uart_buf, strlen(uart_buf));
#endif
              }
              else
              {
#if PRINTF_APP_GATEWAY
                waitToPrint();
                npf_snprintf(uart_buf, 200, "%u [app_gateway] Mic is not correct. Received = %08lX | Calculated = %08lX\n",(unsigned int) xTaskGetTickCount(), rxMic, rxMicCheck);
//                    for (unsigned int i = 0; i < 4; i++)
//                    {
//                      npf_snprintf(byteString, 3, "%02lX", rxMic[i]);
//                      strcat(uart_buf, byteString);
//                    }
//                    strcat(uart_buf, " Calculated = ");
//                    for (unsigned int i = 0; i < 4; i++)
//                    {
//                      npf_snprintf(byteString, 3, "%02lX", rxMicCheck[i]);
//                      strcat(uart_buf, byteString);
//                    }
//                    strcat(uart_buf, ".\n");
                huart2print(uart_buf, strlen(uart_buf));
#endif
              }
            }
            else
            {
#if PRINTF_APP_GATEWAY
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_gateway] The received packet is not a downlink.\n",(unsigned int) xTaskGetTickCount());
              huart2print(uart_buf, strlen(uart_buf));
#endif
            }
          }
          else
          {
#if PRINTF_APP_GATEWAY
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_gateway] Device address not correct.\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
          }
          if (rxACK == 1)
          {
#if PRINTF_APP_GATEWAY
            waitToPrint();
            npf_snprintf(uart_buf, 200, "%u [app_gateway] An acknowledge from the server was received.\n",(unsigned int) xTaskGetTickCount());
            huart2print(uart_buf, strlen(uart_buf));
#endif
          }
        }
        else
        {
#if PRINTF_APP_GATEWAY
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] Wrong notification value received during receive: %02X.\r\n", (unsigned int) xTaskGetTickCount(), (unsigned int) notificationValue);
        huart2print(uart_buf, strlen(uart_buf));
#endif
        }
      } // End of Received message with no time-out, DIOx received
      else
      {
#if PRINTF_APP_GATEWAY
        waitToPrint();
        npf_snprintf(uart_buf, 200, "%u [app_gateway] Nothing received from server.\r\n",(unsigned int) xTaskGetTickCount());
        huart2print(uart_buf, strlen(uart_buf));
#endif
      }

      radioBusy = 0;
      LR1_countUp++;
      //
      // Put Radio to sleep:
      //
      GoToStandby(0);
      vTaskDelay(200U);
      SX1280SetSleep();
  //      vTaskDelay(200U);
#endif // !SENDGATEWAYMESSAGE
    } // end of if (numberOfMeas == 3), line 1534

//#endif // !IMUTEST

    // 1 time unit in the LPTIMER is 8ms
    // if we want data every minute, we need to wait 60000 - totalCycleTime / 8



    // show the status of PWR registers:
//    regVal = READ_REG(PWR->CR1);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1      = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    // check Bit 12: R1RSB1: SRAM1 retention in Standby mode
//    if ((regVal & PWR_CR1_R1RSB1_Msk) == PWR_CR1_R1RSB1_Msk)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_R1RSB1 SRAM1 content retained in Standby mode.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_R1RSB1 SRAM1 content not retained in Standby mode, change R1RSB1.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      regVal |= PWR_CR1_R1RSB1_Msk; // SRAM1 content retained in Standby mode enabled (set to 1)
//      WRITE_REG(PWR->CR1, regVal);
//    }
//    // check Bit 9: RADIORSB: 2.4 GHz RADIO SRAMs (RXTXRAM and Sequence RAM) and Sleep clock retention in Standby mode.
//    if ((regVal & PWR_CR1_RADIORSB_Msk) == PWR_CR1_RADIORSB_Msk)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_RADIORSB1 RADIO SRAMs content retained in Standby mode.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_RADIORSB RADIO SRAMs content not retained in Standby mode, change RADIORSB.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      regVal |= PWR_CR1_RADIORSB_Msk; // RADIO SRAMs content retained in Standby mode enabled (set to 1)
//      WRITE_REG(PWR->CR1, regVal);
//    }
//    // check Bit 7: ULPMEN: BOR0 ultra-low-power mode.
//    if ((regVal & PWR_CR1_ULPMEN_Msk) == PWR_CR1_ULPMEN_Msk)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_ULPMEN BOR0 operating in discontinuous (ultra-low-power) mode in Stop 1 and Standby modes.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_ULPMEN BOR0 operating in continuous (normal) mode in all operating modes.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
////      regVal |= PWR_CR1_ULPMEN_Msk; // Set BOR0 operating in discontinuous (ultra-low-power) mode (set to 1)
////      WRITE_REG(PWR->CR1, regVal);
//    }
//    // check Bit 5: R2RSB1: SRAM2 retention in Standby mode
//    if ((regVal & PWR_CR1_R2RSB1_Msk) == PWR_CR1_R2RSB1_Msk)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_R2RSB1 SRAM2 content retained in Standby mode.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_R2RSB1 SRAM2 content not retained in Standby mode, change R1RSB1.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      regVal |= PWR_CR1_R2RSB1_Msk; // SRAM2 content retained in Standby mode enabled (set to 1)
//      WRITE_REG(PWR->CR1, regVal);
//    }
//    // check Bit 2:0: LPMS[2:0]: Low-power mode selection.
//    if ((!(regVal & PWR_CR1_LPMS_0)) && (!(regVal & PWR_CR1_LPMS_1)) && (!(regVal & PWR_CR1_LPMS_2)))
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_LPMS[2:0] Stop 0 mode.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//      if ((regVal & PWR_CR1_LPMS_0) && (!(regVal & PWR_CR1_LPMS_1)) && (!(regVal & PWR_CR1_LPMS_2)))
//      {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_LPMS[2:0] Stop 1 mode.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      else
//      {
//        if ((!(regVal & PWR_CR1_LPMS_1)) && (regVal & PWR_CR1_LPMS_2))
//        {
//#if PRINTF_APP_GATEWAY
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1_LPMS[2:0] Standby mode.\r\n");
//          huart2print(uart_buf, strlen(uart_buf));
//#endif
//        }
//      }
//    }
////    regVal |= PWR_CR1_LPMS_2; // Set Standby mode (set to 1)
////    WRITE_REG(PWR->CR1, regVal);
//    regVal = READ_REG(PWR->CR1);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR1      = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//
//
//
//
//
//    regVal = READ_REG(PWR->CR2);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR2      = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(PWR->CR3);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->CR3      = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(PWR->VOSR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->VOSR     = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(PWR->SVMCR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->SVMCR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//
//
//
//    regVal = READ_REG(PWR->WUCR1); // Wake Up and Interrupt pins
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR1    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    // check Bit 2: WUPEN3: Wake up and interrupt pin WKUP3
//    if ((regVal & PWR_WUCR1_WUPEN3_Msk) == PWR_WUCR1_WUPEN3_Msk)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR1_WUPEN3 is enabled.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR1_WUPEN3 is disabled, enable it.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      regVal |= PWR_WUCR1_WUPEN3_Msk; // Wake-up and interrupt pin WKUP3 enabled
//      WRITE_REG(PWR->WUCR1, regVal);
//      regVal = READ_REG(PWR->WUCR1);
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR1    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    regVal = READ_REG(PWR->WUCR2); // detection level of Wake up pins
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR2    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    // check Bit 2: WUPEN3: Wake up and interrupt pin WKUP3
//    if ((regVal & PWR_WUCR2_WUPP3_Msk) == PWR_WUCR2_WUPP3_Msk)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR2_WUPP3 is enabled.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR2_WUPP3 is disabled, enable it.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      regVal |= PWR_WUCR2_WUPP3_Msk; // Detection on low level (falling edge)
//      WRITE_REG(PWR->WUCR2, regVal);
//      regVal = READ_REG(PWR->WUCR2);
//  #if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR2    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//      huart2print(uart_buf, strlen(uart_buf));
//  #endif
//    }
//    regVal = READ_REG(PWR->WUCR3);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR3    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    // check Bit 4 and 5: WUSEL3[1:0]: WKUP3 Selection
//    regVal &= PWR_WUCR3_WUSEL3_Msk;
//    regVal = regVal >> 4U;
//    if (regVal == 1)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR3_WUSEL3 is defined as WKUP3_0.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//    }
//    else
//    {
//      if (regVal == 2)
//      {
//#if PRINTF_APP_GATEWAY
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR3_WUSEL3 is defined as WKUP3_1.\r\n");
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//      }
//      else
//      {
//        if (regVal == 0)
//        {
//#if PRINTF_APP_GATEWAY
//          waitToPrint();
//          npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR3_WUSEL3 is not defined.\r\n");
//          huart2print(uart_buf, strlen(uart_buf));
//#endif
//        }
//      }
//    }
//    WRITE_REG(PWR->WUCR3, PWR_WUCR3_WUSEL3_1);
//    regVal = READ_REG(PWR->WUCR3);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUCR3    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(PWR->SVMSR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->SVMSR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(PWR->WUSR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->WUSR     = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(PWR->RADIOSCR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->RADIOSCR = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    regVal = READ_REG(PWR->IORETENRA);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRA= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal |= 0xFFFF;
//    WRITE_REG(PWR->IORETENRA, regVal);
//    regVal = READ_REG(PWR->IORETENRA);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRA= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    regVal = READ_REG(PWR->IORETENRB);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRB= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal |= 0xFFFF;
//    WRITE_REG(PWR->IORETENRB, regVal);
//    regVal = READ_REG(PWR->IORETENRB);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRB= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    regVal = READ_REG(PWR->IORETENRC);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRC= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal |= 0xE000;
//    WRITE_REG(PWR->IORETENRC, regVal);
//    regVal = READ_REG(PWR->IORETENRC);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRC= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    regVal = READ_REG(PWR->IORETENRH);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRH= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal |= 0x0008;
//    WRITE_REG(PWR->IORETENRH, regVal);
//    regVal = READ_REG(PWR->IORETENRH);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] PWR->IORETENRH= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif


    // show status of GPIO Port B registers: user LED is on PB13
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB register settings before switching off LED:\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    regVal = READ_REG(GPIOB->MODER);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->MODER  = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->OTYPER);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->OTYPER = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->OSPEEDR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->OSPEEDR= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->ODR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->ODR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->BSRR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->BSRR   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->BRR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->BRR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif


     //
    // Make sure the LED is off:
    //
//    USER_LED_GPIO_Port->BSRR = (uint32_t)USER_LED_Pin;
//    vTaskDelay(10U);
//    USER_LED_GPIO_Port->BSRR = (uint32_t)USER_LED_Pin;
//    vTaskDelay(10U);
//    USER_LED_GPIO_Port->BSRR = (uint32_t)USER_LED_Pin;

    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
//    vTaskDelay(10U);
//    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);


//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB register settings after switching off LED:\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    regVal = READ_REG(GPIOB->MODER);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->MODER  = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->OTYPER);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->OTYPER = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->OSPEEDR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->OSPEEDR= 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->ODR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->ODR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->BSRR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->BSRR   = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
//    regVal = READ_REG(GPIOB->BRR);
//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Gateway] GPIOB->BRR    = 0x%08X 0b" UINT32_T_TO_BINARY_PATTERN "\r\n",(unsigned int) regVal, UINT32_T_TO_BINARY(regVal));
//    huart2print(uart_buf, strlen(uart_buf));
//#endif


      //
      // switch off ADC:
      //
//      HAL_ADC_MspDeInit(&hadc4);
    //
    // Make sure the radio sleeps:
    //
//    GoToStandby(0);
//    SX1280SetSleep();
//    vTaskDelay(100U);



    totalCycleTime = xTaskGetTickCount() - timeStampIMUCycleStart;  // calculate cycle time
#if SLEEPTIME1MINUTE
    //for test purposes, 1 minute:
    offTime = 61000 - totalCycleTime;                               // calculate remaining waiting time for 1 minute cycle (+ 1s extra per minute)
#else
    offTime = 305000 - totalCycleTime;                              // calculate remaining waiting time for 5 minutes cycle (+ 1s extra per minute)
#endif
    offTimeLptimer = offTime /8;
// LPTIM: 8ms per period unit
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Total cycle time = %ums, Put CPU to sleep for %ums.\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) totalCycleTime, (unsigned int) offTime);
    huart2print(uart_buf, strlen(uart_buf));
#endif

//    //check status of LED:
//
//    regVal = READ_REG(GPIOB->ODR);
//    regVal &= GPIO_ODR_OD13_Msk;
//    if (!regVal)
//    {
//#if PRINTF_APP_GATEWAY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[Gateway] LED is ON, switch off LED\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
//    }

//20240401 put to comment, needs to be reviewed
//    __HAL_TIM_SET_AUTORELOAD(&hlptim1, 1250);
//    __HAL_TIM_SET_COUNTER(&hlptim1, 1250);
    LPTIM1->ARR = offTimeLptimer;
    hlptim1.Init.Period = offTimeLptimer;
    HAL_LPTIM_Init(&hlptim1);

//#if PRINTF_APP_GATEWAY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_gateway] LPTIM1 REGISTERS: ARR = %u, CCMR1 = %u, CCR1 = %u, CNT = %u, RCR = %u.\r\n",(unsigned int) xTaskGetTickCount(),
//        (unsigned int) LPTIM1->ARR, (unsigned int) LPTIM1->CCMR1, (unsigned int) LPTIM1->CCR1, (unsigned int) LPTIM1->CNT, (unsigned int) LPTIM1->RCR);
//    huart2print(uart_buf, strlen(uart_buf));
//#endif


    // ONLY FOR TESTING: switch off 1V8 to check current rating in sleep mode without sensors:
    // HAL_GPIO_WritePin(VSW1V8_On_GPIO_Port,   VSW1V8_On_Pin,   GPIO_PIN_RESET);   // switch 1V8 for IMU, BME280 and VEML6035 off

//#if PCF2131USED
//    pcf2131_get_time(&rtc_time);
//    offTime = (uint32_t) rtc_time.seconds + 30; // the second alarm compares the actual value of the seconds register, so if we want to have 30s sleep time, we need to add that to the current second value
//    if (offTime > 59)
//    {
//      offTime -= 60;
//    }
//    pcf2131SecondAlarm(offTime);
//    pcf2131_get_status(pcf2131Buf, 54, 1);
//#endif

    //
    // start the Low Power Timer first time to generate a wake-up interrupt in 10s
    //

    HAL_LPTIM_Counter_Start_IT(&hlptim1);
    __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARROK);

    //
    // Put the STM32 in standby mode with retention until the wake-up interrupt is received
    //
//    HAL_PWR_EnterSTANDBYMode();

    //
    //check status of LED:
    //
    calibrationActiveOld = calibrationActive;
    calibrationActive = 0;
    ledFreq = 0; // led off
    regVal = READ_REG(GPIOB->ODR);
    regVal &= GPIO_ODR_OD13_Msk;
    if (!regVal)
    {
#if PRINTF_APP_GATEWAY
      waitToPrint();
      npf_snprintf(uart_buf, 200, "[Gateway] LED is ON, switch off LED\r\n");
      huart2print(uart_buf, strlen(uart_buf));
#endif
      HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    }
    //
    // Put the STM32 in low power mode 0 until the wake-up interrupt is received
    //
//20240810
#if !KEEPCPUAWAKE
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_LPTIM_Counter_Stop_IT(&hlptim1);
#endif
    //20240520 After wake up, the UART starts up at a speed of 1000000 bps!!
    //todo
    calibrationActive = calibrationActiveOld;
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Awake after measurement %d of a complete cycle.\r\n",(unsigned int) xTaskGetTickCount(), numberOfMeas);
    huart2print(uart_buf, strlen(uart_buf));
#endif

#if PCF2131USED
    // take actual time of RTC to check:
    pcf2131_get_time(&rtc_time);
#if PRINTF_APP_GATEWAY
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gateway] Current RTC Date: %d/%d/%d, UTC Time: %d:%d:%d.%d.\r\n", (unsigned int) xTaskGetTickCount(),
         rtc_time.dayOfMonth, rtc_time.month, rtc_time.year, rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.subSeconds);
    huart2print(uart_buf, strlen(uart_buf));
#endif
#endif



//    //
//    // Switch on ADC:
//    //
//    HAL_ADC_MspInit(&hadc4);
    totalCycleTime += offTime;

    if (numberOfMeas == 3)
    {
      numberOfMeas = 0;
    }
  }
}


void GatewayNotifyFromISR(uint32_t notValue)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  if (gatewayThreadHandler != NULL)
  {
    xTaskNotifyFromISR(gatewayThreadHandler, notValue, eSetBits, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GatewayNotify(uint32_t notValue)
{
  if (gatewayThreadHandler != NULL)
  {
    xTaskNotify(gatewayThreadHandler, notValue, eSetBits);
  }
}

