/*
 * SX1280.c
 *
 *  Created on: Jul 31, 2022
 *      Author: Sarah Goossens
 */

#include "SX1280.h"
#if  PCF2131I2C
#include "i2c.h"
#else
#include "spi.h"
#endif
#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
//#include "../../Callback/timer_callback.h"
#include "rangingcorrection.h"
#include "cmsis_os2.h"
//#include "../../App/app_rtc.h"
#include "../../App/app_supercap.h"

#define PRINTF_SX1280 1
#define PRINTF_SX1280_RADIO_BUSY 1

extern char uart_buf[100];
extern const uint32_t Channelf[];
extern RadioLoRaBandwidths_t bandwidth;

TickTime_t timeout;
uint8_t current_ranging_channel;
uint8_t time_to_change_channels;
extern uint16_t YourID, MyID;
extern uint8_t spreading_factor;

extern uint64_t startOfTransmit;

extern osThreadId_t initThreadHandler;
extern RadioStatus_t         statusSX1280;

// moved out of SX1280.h:
////Ranging raw factor                 SF5     SF6     SF7     SF8     SF9     SF10
//const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
//const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
//const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
//const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
//const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
//const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };

HAL_StatusTypeDef HAL_Status;


/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE 0xFF //
static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

void SX1280HalReset(void)
{
  uint8_t stat = 0;
  while (!stat)
  {
    HAL_GPIO_WritePin(RESET_SX1280_GPIO_Port, RESET_SX1280_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(RESET_SX1280_GPIO_Port, RESET_SX1280_Pin, GPIO_PIN_SET);
    vTaskDelay(200);
	SX1280HalReadCommand(RADIO_GET_STATUS, (uint8_t *) &stat, 1);
    vTaskDelay(200);
	SX1280HalReadCommand(RADIO_GET_STATUS, (uint8_t *) &stat, 1);
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [SX1280] [HalReset] Lora Radio status returns 0, try again to reset radio.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
    HAL_GPIO_WritePin(VSW1V8_On_GPIO_Port,   VSW1V8_On_Pin,   GPIO_PIN_SET);   // switch 1V8 for IMU, BME280 and VEML6035 on
    vTaskDelay(200);
  }
}

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void SX1280WaitOnBusy(void)
{
  uint32_t whileIterations = 0; // to prevent lock into while loop
  uint32_t onlyOnce        = 1;
  while(HAL_GPIO_ReadPin(BUSY_SX1280_GPIO_Port, BUSY_SX1280_Pin) == 1)
  {
    if (onlyOnce)
    {
      onlyOnce = 0;
//#if PRINTF_SX1280_RADIO_BUSY
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [SX1280.c] [SX1280WaitOnBusy] Radio busy.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
    }
    vTaskDelay(100);
    if(whileIterations++ > 200)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [SX1280.c] [SX1280WaitOnBusy] Error: Jump out of while loop (Radio busy for > 20s).\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      return;
    }
  }
  //todo 20240413 implement SX1280WaitOnBusy() with EXTI callback:
//  uint32_t   notificationValue = 0;                     // Used to identify where the notification is coming from.
//
//  xTaskNotifyStateClear(initThreadHandler);
//  if (xTaskNotifyWait(0x00, 0xffffffff, &notificationValue, 10000))
//  {
//    if ((notificationValue & NOTIFICATION_FROM_RADIO_AVAILABLE) == NOTIFICATION_FROM_RADIO_AVAILABLE)
//    { // Interrupt from RADIO_BUSY_Pin (see main HAL_GPIO_EXTI_Callback()) that the Radio is ready
//#if PRINTF_APP_INIT
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_init] [initThread] Radio available notification received after reset of radio.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//
//    }
//  }
}

RadioStatus_t SX1280GetStatus(void)
{
  uint8_t stat = 0;
  RadioStatus_t status;
  SX1280HalReadCommand(RADIO_GET_STATUS, (uint8_t *) &stat, 1);
  status.Value = stat;

#if PRINTF_SX1280
  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [SX1280] [GetStatus] Lora Radio status = %u: CmdStatus = %u, ChipMode = %u.\r\n",
//       (unsigned int) xTaskGetTickCount(), (unsigned int) status.Value, (unsigned int) status.Fields.CmdStatus, (unsigned int) status.Fields.ChipMode);
  npf_snprintf(uart_buf, 200, "%u [SX1280] [GetStatus] Lora Radio status = %u; ",(unsigned int) xTaskGetTickCount(), (unsigned int) status.Value);
  if (status.Fields.CpuBusy)
  {
    strcat(uart_buf, "CpuBusy, ");
  }
  if (status.Fields.DmaBusy)
  {
    strcat(uart_buf, "DmaBusy, ");
  }
  strcat(uart_buf, "CmdStatus: ");
  switch (status.Fields.CmdStatus)
  {
    case 0x00:
      strcat(uart_buf, "0");
      break;
    case 0x01:
      strcat(uart_buf, "Cmd successfully processed");
      break;
    case 0x02:
      strcat(uart_buf, "Data is available to host");
      break;
    case 0x03:
      strcat(uart_buf, "Command time-out");
      break;
    case 0x04:
      strcat(uart_buf, "Command processing error");
      break;
    case 0x05:
      strcat(uart_buf, "Failure to execute command");
      break;
    case 0x06:
      strcat(uart_buf, "Command Tx done");
  }
  strcat(uart_buf, ", ChipMode: ");
  switch (status.Fields.ChipMode)
  {
    case 0x00:
      strcat(uart_buf, "0");
      break;
    case 0x01:
      strcat(uart_buf, "1");
      break;
    case 0x02:
      strcat(uart_buf, "STDBY_RC");
      break;
    case 0x03:
      strcat(uart_buf, "STDBY_XOSC");
      break;
    case 0x04:
      strcat(uart_buf, "FS");
      break;
    case 0x05:
      strcat(uart_buf, "Rx");
      break;
    case 0x06:
      strcat(uart_buf, "Tx");
  }
  strcat(uart_buf, ".\r\n");
  huart2print(uart_buf, strlen(uart_buf));
#endif
//  uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
//  uint8_t DmaBusy   : 1;  //!< Flag for DMA busy
//  uint8_t CmdStatus : 3;  //!< Command status
//  uint8_t ChipMode  : 3;  //!< Chip mode
  return status;
}

void SetRfFrequency( uint32_t frequency )
{
  uint8_t buf[3];
  uint32_t freq = 0;
  freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
  buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
  buf[1] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
  buf[2] = ( uint8_t )( freq & 0xFF );
  SX1280HalWriteCommand( RADIO_SET_RFFREQUENCY, buf, 3 );
}

void SX1280SetSleep(void)
{
	uint8_t buf[10];
	buf[0] = 3; //20240420 sleep config data retention (both RAM and Buffer)
//    uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
//                    ( sleepConfig.InstructionRamRetention << 2 ) |
//                    ( sleepConfig.DataBufferRetention << 1 ) |
//                    ( sleepConfig.DataRamRetention );
//
//    OperatingMode = MODE_SLEEP;
    SX1280HalWriteCommand(RADIO_SET_SLEEP, buf, 1 ); // 0x84
//#if PRINTF_SX1280_RADIO_BUSY
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [SX1280.c] [SX1280SetSleep] Radio in sleep mode.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
//#endif

}

void GoToStandby(uint8_t mode)//0-STDBY_RC  1-STDBY_XOSC
{
  uint8_t stat = 0;
  uint8_t buf[10];
  if(mode == 0)
  {
    buf[0] = STDBY_RC; //0x00
  }
  else
  {
    buf[0] = STDBY_XOSC;//0x01
  }
  SX1280HalWriteCommand(RADIO_SET_STANDBY, buf, 1 ); // 0x80
  //check status
  SX1280HalReadCommand(RADIO_GET_STATUS, (uint8_t *) &stat, 1);
  SX1280WaitOnBusy();
#if PRINTF_SX1280_RADIO_BUSY
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] [GoToStandby] Radio in standby mode.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif

  //status.Value = stat;
  //status:
  //010x xxxx - 2 STDBY_RX
  //011x xxxx - 3 STCBY_XOSC
  //100x xxxx - 4 FS active  - here most of the time...
  //101x xxxx - 5 RX
  //110x xxxx - 6 TX
  //xxx0 01xx - 1 Command successfully processed
  //xxx0 10xx - 2 data ready to be read
  //xxx0 11xx - 3 Command timeout
  //xxx1 00xx - 4 Command processing error
  //xxx1 01xx - 5 Failure to execute command
  //xxx1 10xx - 6 Packet TX complete
}

void SetTypicalRegisterSettings(uint8_t packetType, uint8_t payloadLength, uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t bleChannelNr)
{
  uint8_t buf[7];
  uint8_t tempbuf[1];
  ////////////////////
  // Step 1: Go to standby_RC mode
  GoToStandby(0);//go to standby mode: 0 = STDBY_RC  1 = STDBY_XOSC
  buf[0] = 0x00;  //TX buffer base Address
  buf[1] = 0x00;  //RX buffer base Address
  SX1280HalWriteCommand(RADIO_SET_BUFFERBASEADDRESS, buf, 2);
  ////////////////////
  // Step 2: Set packet type
  buf[0] = packetType;  //0-GFSK, 1-LORA, 2-RANGING 3-FLRC
  SX1280HalWriteCommand( RADIO_SET_PACKETTYPE, buf, 1 );
  ////////////////////
  // Step 3: Set the modulation parameters
  buf[0] = spreadingFactor; // Spreading factor
  buf[1] = bandwidth;       // Bandwidth
  buf[2] = codingRate;      // CodingRate, ex LORA_CR_4_5
  SX1280HalWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, 3 );
  // Datasheet Note p131 - after SetModulationParams command:
  // If the Spreading Factor selected is SF5 or SF6, it is required to use WriteRegister( 0x925, 0x1E )
  if(buf[0] == LORA_SF5 || buf[0] == LORA_SF6){tempbuf[0] = 0x1E;}
  // If the Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
  if(buf[0] == LORA_SF7 || buf[0] == LORA_SF8){tempbuf[0] = 0x37;}
  // If the Spreading Factor is SF9, SF10, SF11 or SF12, then the command WriteRegister( 0x925, 0x32 ) must be used
  if(buf[0] == LORA_SF9 || buf[0] == LORA_SF10 || buf[0] == LORA_SF11 || buf[0] == LORA_SF12){tempbuf[0] = 0x32;}
  WriteRegister_16bAddress(0x925, tempbuf, 1);
  // Datasheet Note p131 - after SetModulationParams command:
  // In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C
  buf[0] = 0x01; // LORA_SF_10
  WriteRegister_16bAddress(0x93C, buf, 1);
  ////////////////////
  // Step 4: Set the packet parameters
#if PLANTSENSOR
  buf[0] = 0x08;   // changed to 0x08 for TTN (was 0xC)
#else
  buf[0] = 0x0C;                 // PreambleLength, normally 0x0C
#endif
  buf[1] = LORA_PACKET_EXPLICIT; // HeaderType explicit
  buf[2] = payloadLength;        // PayloadLength
  buf[3] = LORA_CRC_ON;          // CRC enabled
#if PLANTSENSOR
  buf[4] = LORA_IQ_NORMAL;       // InvertIQ/chirp invert //was normal
#else
  buf[4] = LORA_IQ_INVERTED;       // InvertIQ/chirp invert //was normal
#endif
  buf[5] = 0x00;                 // not used
  buf[6] = 0x00;                 // not used
  SX1280HalWriteCommand( RADIO_SET_PACKETPARAMS, buf, 7 );
#if PRINTF_SX1280
   waitToPrint();
   npf_snprintf(uart_buf, 200, "%u [SX1280] [SetTypicalRegisterSettings] packet parameters set.\r\n",(unsigned int) xTaskGetTickCount());
   huart2print(uart_buf, strlen(uart_buf));
#endif
  ////////////////////
  // Step 5: Set the RF frequency to be used
  SetRfFrequency(Channelf[bleChannelNr]); //set TX freq to first in hop list
#if PRINTF_SX1280
   waitToPrint();
   npf_snprintf(uart_buf, 200, "%u [SX1280] [SetTypicalRegisterSettings] Tx frequency set.\r\n",(unsigned int) xTaskGetTickCount());
   huart2print(uart_buf, strlen(uart_buf));
#endif
  ////////////////////
  // Step 6: Set the Tx parameters - power and TX ramp time
  buf[0] = 31;  // PoutMAX (dB) = -18 + 31 = +13dBm (+12.5 max) transmit power 31 = 0x1F
                  // Pout    (dB) = -18 + 18 =   0dBm transmit power = 0x12
                  // PoutMIN (dB) = -18 +  0 = -18dBm transmit power
  buf[1] = 0xE0;  // 20uS = 0xEO ramp time - best for less out of band noise = lowest ramp time
                  // 16us = 0xC0
                  // 12us = 0xA0
                  // 10us = 0x80
                  //  8us = 0x60
                  //  6us = 0x40
                  //  4us = 0x20
                  //  2us = 0x00
  SX1280HalWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );  // 0x8E
}

void WriteRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size )
{
  uint16_t halSize  = size + 3;
  SX1280WaitOnBusy();
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  halTxBuffer[0] = RADIO_WRITE_REGISTER; //0x18 command for writing a block of bytes in a data memory space
  halTxBuffer[1] = (address & 0xFF00) >> 8 ;
  halTxBuffer[2] =  address & 0x00FF;
  memcpy(halTxBuffer + 3, (uint8_t *)buffer, size);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, halSize, 100);
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  //if( address != RADIO_SET_SLEEP )
  SX1280WaitOnBusy();
}

void ReadRegister_16bAddress( uint16_t address, uint8_t *buffer, uint16_t size )
{
  //uint8_t status;
  uint16_t halSize  = size + 3;
  halTxBuffer[0] = RADIO_READ_REGISTER;// send 0x19
  halTxBuffer[1] = ( address & 0xFF00 ) >> 8 ;//send 00 C0
  halTxBuffer[2] = address & 0x00FF;
  //halTxBuffer[3] = 0x00;// send 00
  for(uint16_t index = 0; index < size; index++)
  {
    halTxBuffer[3+index] = 0x00;
  }
  SX1280WaitOnBusy();
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, halRxBuffer, halSize, 100) != HAL_OK)
  {
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[SX1280] [ReadRegister_16bAddress] SPI RADIO_GET_STATUS issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  else
  {
    memcpy(buffer, halRxBuffer + 3, size); //was halRxBuffer +1 --> wrong!!
  }
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
  //return status;
}

void SX1280HalReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
  uint16_t halSize = 2 + size;
  halTxBuffer[0] = command;
  halTxBuffer[1] = 0x00;
  for(uint16_t index = 0; index < size; index++)
  {
    halTxBuffer[2+index] = 0x00;
  }
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, halRxBuffer, halSize, 100) != HAL_OK)
  {
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[SX1280] [SX1280HalReadCommand] SPI RADIO_GET_STATUS issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  else
  {
    memcpy(buffer, halRxBuffer + 2, size);
  }
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
}

void SetRx(void) //enter receive mode
{
//  halTxBuffer[0]  = RADIO_CLR_IRQSTATUS; // 0x97
//  halTxBuffer[1]  = 0xFF;
//  halTxBuffer[2]  = 0xFF;
//  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, 3, 100);
//  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
//  timeout.Step    = RADIO_TICK_SIZE_0015_US; // 0x00 : 15.625uS
//  timeout.NbSteps = 0xFFFF;                  // continuous mode
  halTxBuffer[0]  = RADIO_SET_RX;            // 0x82
  halTxBuffer[1]  = RADIO_TICK_SIZE_0015_US; // 0x00 : 15.625uS
//  halTxBuffer[2]  = (uint8_t)((timeout.NbSteps >> 8) & 0x00FF);
//  halTxBuffer[3]  = (uint8_t)( timeout.NbSteps       & 0x00FF);
  halTxBuffer[2]  = 0xFF; // continuous mode
  halTxBuffer[3]  = 0xFF;
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, 4, 100);
//  HAL_SPI_Transmit(&hspi1, (uint8_t *)0x8200FFFF, 4, 100);
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
}

void SetTx(void)
{
//  halTxBuffer[0]  = RADIO_CLR_IRQSTATUS;
//  halTxBuffer[1]  = 0xFF;
//  halTxBuffer[2]  = 0xFF;
//  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, 3, 100);
//  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
//  timeout.Step    = RADIO_TICK_SIZE_0015_US; //15.625uS
//  timeout.NbSteps = 0xFFFF;
  halTxBuffer[0]  = RADIO_SET_TX;
//  halTxBuffer[1]  = timeout.Step;
//  halTxBuffer[2]  = (uint8_t) ((timeout.NbSteps >> 8) & 0x00FF);
//  halTxBuffer[3]  = (uint8_t) ( timeout.NbSteps       & 0x00FF);
  halTxBuffer[1]  = RADIO_TICK_SIZE_0015_US; // 0x00 : 15.625uS
  halTxBuffer[2]  = 0xFF; // continuous mode
  halTxBuffer[3]  = 0xFF;
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  // Start of transmit - this is the closest place to take a timestamp for t5 (in case of Host and t1 in case of Node)
  startOfTransmit = xTaskGetTickCount();
  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, 4, 100);
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
}

void ClearIrqStatus( uint16_t irq )
{
  uint8_t buf[2];
  buf[0] = (uint8_t) (((uint16_t)irq >> 8) & 0x00FF);
  buf[1] = (uint8_t) ( (uint16_t)irq       & 0x00FF);
  SX1280HalWriteCommand(RADIO_CLR_IRQSTATUS, buf, 2);
}


void SX1280HalWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
  uint16_t halSize  = size + 1;
//20240806  if((command != RADIO_SET_SLEEP))// && (command != RADIO_SET_STANDBY))
//  {
//    SX1280WaitOnBusy();
//  }
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  halTxBuffer[0] = command;
  memcpy(halTxBuffer + 1, (uint8_t *)buffer, size);
  // Start of transmit - this is the closest place to take a timestamp for t5 (in case of Host and t1 in case of Node)
  startOfTransmit = xTaskGetTickCount();
  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, halSize, 100);
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  if((command != RADIO_SET_SLEEP))// && (command != RADIO_SET_STANDBY))
  {
    SX1280WaitOnBusy();
  }
}

void SX1280HalWriteReadCommand(RadioCommands_t command, uint8_t *txbuffer, uint16_t size, uint8_t *rxbuffer)
{
  uint16_t halSize  = size + 1;
  SX1280WaitOnBusy();
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  halTxBuffer[0] = command;
  memcpy(halTxBuffer + 1, (uint8_t *)txbuffer, size);
  HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, rxbuffer, halSize, 100);
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  if(command != RADIO_SET_SLEEP)
  {
    SX1280WaitOnBusy();
  }
}

void WriteTXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size )
{
  uint16_t halSize = 2 + size;
  SX1280WaitOnBusy();
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  halTxBuffer[0] = RADIO_WRITE_BUFFER;
  halTxBuffer[1] = offset;
  memcpy(halTxBuffer + 2, (uint8_t *)buffer, size);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)halTxBuffer, halSize, 100);
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
}

void ReadRXBuffer(uint8_t offset, uint8_t *buffer, uint16_t size )
{
  uint16_t halSize = 3 + size;
  halTxBuffer[0] = RADIO_READ_BUFFER;
  halTxBuffer[1] = offset;
  halTxBuffer[2] = 0; //write NOP, read status
  for(uint16_t index = 0; index < size; index++)
  {
    halTxBuffer[3 + index] = 0x00;
  }
  SX1280WaitOnBusy();
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi1, halTxBuffer, halRxBuffer, halSize, 400) != HAL_OK)
  {
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[SX1280] [ReadRXBuffer] SPI RADIO_GET_STATUS issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  else
  {
    memcpy(buffer, halRxBuffer + 3, size);
  }
  HAL_GPIO_WritePin(CS_SX1280_GPIO_Port, CS_SX1280_Pin, GPIO_PIN_SET);
  SX1280WaitOnBusy();
}


//S
//Write 32-bit data to actual EEPROM memory
//**********************************************************************************
//void EepromWrite( uint32_t addr, uint32_t data)
//{
//  uint32_t Address = 0;
//  /* Unlock the DATA EEPROM Flash to enable the flash control register access *************/
//  HAL_FLASHEx_DATAEEPROM_Unlock();
//  /* Erase the DATA EEPROM Flash area word by word (area defined by length) ***********/
////#if PRINTF_SX1280
////  waitToPrint();
////  npf_snprintf(uart_buf, 200, "[SX1280] [EepromWrite] DATA EEPROM unlock done.\r\n");
////  huart2print(uart_buf, strlen(uart_buf));
////#endif
//  Address = addr;
////  int32_t i = length;
//  int32_t i = 12;
//  int8_t erase_OK = 1;
//  int8_t program_OK = 1;
//  //int8_t memoryProgramStatus = 0;
//    if (HAL_FLASHEx_DATAEEPROM_Erase(Address) == HAL_OK)
//    {
////      *(__IO uint32_t *) Address = 0x00000000U;
////      Address = Address + 4;
////      i = i-4;
//    }
//    else
//    {
//      /* Error occurred while erasing DATA EEPROM Flash memory. */
//#if PRINTF_SX1280
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[SX1280] [EepromWrite] DATA EEPROM Erase error!\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
//      erase_OK = 0;
//    }
//  /* Program the DATA EEPROM Flash area byte by byte (area defined by length) ***********/
//  if (erase_OK)
//  {
//	Address = addr;
//	i = 0;
////	while (i < length)
////	while (i < 10)
////	{
//	  if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, Address, data) == HAL_OK)
//	  {
////		*(__IO uint8_t *)Address = (uint8_t) state_buffer[i];
//		Address++;
//	    i++;
//	  }
//	  else
//	  {
//	    /* Error occurred while writing DATA EEPROM Flash memory. */
//#if PRINTF_SX1280
//	    waitToPrint();
//        npf_snprintf(uart_buf, 200, "[SX1280] [EepromWrite] DATA EEPROM Write error!\r\n");
//        huart2print(uart_buf, strlen(uart_buf));
//#endif
//	    program_OK = 0;
//	  }
////	}
//  }
//  /* Lock the DATA EEPROM Flash to disable the flash control register access (recommended
//     to protect the FLASH memory against possible unwanted operation) *********/
//  HAL_FLASHEx_DATAEEPROM_Lock();
//  /* Check if the programmed data is OK
//      memoryProgramStatus = 0: data programmed correctly
//      memoryProgramStatus != 0: number of words not programmed correctly ******/
//  //TO DO
////  if (program_OK)
////  {
////	Address = DATA_EEPROM_BASE;
////	i = 0;
////	while (i < length)
////	{
////	  if (state_buffer[i] != *(__IO uint8_t *)(Address))
////	  {
////		memoryProgramStatus++;
////	  }
////	  Address++;
////	  i++;
////	}
////  }
//  /*Check if verification is OK*/
////  if (memoryProgramStatus != 0)
////  {
////	uart_buf_len = sprintf(uart_buf, "DATA EEPROM Verification error!\r\n");
////	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
////  }
////  else
////  {
////	uart_buf_len = sprintf(uart_buf, "Library state (%d bytes) correctly saved to DATA EEPROM.\r\n",length);
////    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
////  }
//}

double calc_y1(double x, int32_t rssi, int8_t chipwhip)//0-chip  1-whip
{
	volatile double y;

	if(chipwhip==1)
	{
		//WHIP ANTENNA
		if(rssi>-57)//SF6 - near targets
		{
			if(x<=-128)//less than 10ft
				y = -125.5105 - 3.018208*x - 0.02405039*x*x - 0.00008100349*x*x*x - 9.929779e-8*x*x*x*x;
			else // >10ft
				y = 9.359128 + 0.07299215*x + 0.0002869848*x*x - 0.000001927275*x*x*x + 4.244874e-9*x*x*x*x - 3.064832e-12*x*x*x*x*x;
		}
			else //SF10 - far targets (>50ft)
				y = 5.753966 + 0.07179256*x + 3.41401e-8*x*x - 1.146837e-13*x*x*x;
	}

	if(chipwhip==0)
	{
		//CHIP ANTENNA
		if(rssi>-80)//SF6 - near targets
		{
			if(x<=-142)//less than 10ft
				y = 25.62718 + 0.1427834*x + 0.0002415783*x*x + 7.792585e-8*x*x*x;
			else // >10ft
				y = 19.87852 + 0.06585959*x - 0.00001669755*x*x + 1.879339e-8*x*x*x;
		}
			else //SF10 - far targets (>50ft)
				y = 35.86462 + 0.07399442*x - 1.032333e-9*x*x;
	}

	return y;
}

int32_t complement2( const uint32_t num, const uint8_t bitCnt )
{
  int32_t retVal = ( int32_t )num;
  if( num >= 2<<( bitCnt - 2 ) )
  {
	retVal -= 2<<( bitCnt - 1 );
  }
  return retVal;
}

double SX1280GetRangingCorrectionPerSfBwGain( const uint8_t sf, const RadioLoRaBandwidths_t bw, const int8_t gain){
    uint8_t sf_index, bw_index;

    switch(sf){
//        case 5:
//            sf_index = 0;
//            break;
//        case 6:
//            sf_index = 1;
//            break;
//        case 7:
//            sf_index = 2;
//            break;
//        case 8:
//            sf_index = 3;
//            break;
//        case 9:
//            sf_index = 4;
//            break;
//        case 10:
//            sf_index = 5;
//            break;
//		case 11:
//			sf_index = 6;
//			break;
//		case 12:
//            sf_index = 7;
//            break;

	   case LORA_SF5:
			sf_index = 0;
			break;
		case LORA_SF6:
			sf_index = 1;
			break;
		case LORA_SF7:
			sf_index = 2;
			break;
		case LORA_SF8:
			sf_index = 3;
			break;
		case LORA_SF9:
			sf_index = 4;
			break;
		case LORA_SF10:
			sf_index = 5;
			break;
		case LORA_SF11:
			sf_index = 6;
			break;
		case LORA_SF12:
			sf_index = 7;
			break;
    }
    switch(bw){
        case LORA_BW_0400:
            bw_index = 0;
            break;
        case LORA_BW_0800:
            bw_index = 1;
            break;
        case LORA_BW_1600:
            bw_index = 2;
            break;
    }

    double correction = RangingCorrectionPerSfBwGain[sf_index][bw_index][gain];
    //uart_buf_len = sprintf(uart_buf, "SF: %d, BW: %d, gain: %d, Correction: %f.\r\n", sf_index, bw_index, gain, correction);
    //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
    return correction;
}


uint8_t SX1280GetRangingPowerDeltaThresholdIndicator( void )
{
        uint8_t buf[2];
	GoToStandby(1);//0-STDBY_RC  1-STDBY_XOSC

	//Enable clock in LoRa memory
	// Freeze Ranging Result
	ReadRegister_16bAddress(0x097F, buf, 2); //was 1
	buf[0] = ( buf[1] | (1<<1) ); //was ( buf[0] | (1<<1) )
	WriteRegister_16bAddress(0x097F, buf, 1); //Set to preserve the ranging result for reading

	//Set Ranging Result Type
	//Ranging result configuration.
	ReadRegister_16bAddress(0x0924, buf, 2);
	buf[0] = (buf[1] & 0xCF); //default to Raw result
	WriteRegister_16bAddress(0x0924, buf, 1);

	ReadRegister_16bAddress(0x0964, buf, 2);

//	uart_buf_len = sprintf(uart_buf, "Buffer na uitlezen register 0x0964 = 0x%0X, 0x%0X, 0x%0X \r\n", buf[0], buf[1], buf[2]);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	return buf[1];
}

int32_t SX1280GetLoRaBandwidth( )
{
    int32_t bwValue = 0;
    bandwidth = LORA_BW_1600;

    switch(bandwidth)
    {
        case LORA_BW_0200:
            bwValue = 203125;
            break;

        case LORA_BW_0400:
            bwValue = 406250;
            break;

        case LORA_BW_0800:
            bwValue = 812500;
            break;

        case LORA_BW_1600:
            bwValue = 1625000;
            break;

        default:
            bwValue = 0;
    }

//    uart_buf_len = sprintf(uart_buf, "bwValue = %ld\r\n", bwValue);
//    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
    return bwValue;
}

double SX1280ComputeRangingCorrectionPolynome(const uint8_t sf, const RadioLoRaBandwidths_t bw, const double median)
{
    uint8_t sf_index, bw_index;

    switch(sf){
        case 5:
            sf_index = 0;
            break;
        case 6:
            sf_index = 1;
            break;
        case 7:
            sf_index = 2;
            break;
        case 8:
            sf_index = 3;
            break;
        case 9:
            sf_index = 4;
            break;
        case 10:
            sf_index = 5;
            break;
		case 11:
			sf_index = 6;
			break;
		case 12:
            sf_index = 7;
            break;
    }

//		uart_buf_len = sprintf(uart_buf, "sf_index = %d\n\r", sf_index);
//		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);


    switch(bw){
        case LORA_BW_0400:
            bw_index = 0;
            break;
        case LORA_BW_0800:
            bw_index = 1;
            break;
        case LORA_BW_1600:
            bw_index = 2;
            break;
    }


//	uart_buf_len = sprintf(uart_buf, "bw_index = %d\n\r", bw_index);
//	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

    const RangingCorrectionPolynomes_t *polynome = RangingCorrectionPolynomesPerSfBw[sf_index][bw_index];
    double correctedValue = 0.0;
    double correctionCoeff = 0;
    for(uint8_t order = 0; order < polynome->order; order++){
        correctionCoeff = polynome->coefficients[order] * pow(median, polynome->order - order - 1);
        correctedValue += correctionCoeff;
    }
    return correctedValue;
}

void SX1280Init(void)
{
  uint8_t  buf[5];
  uint8_t  rxbuf[5];
  //SX1280WaitOnBusy();
  SX1280HalReset(); // Reset the radio
  SX1280WaitOnBusy();
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] LoRa radio reset done.\r\n", (unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  statusSX1280 = SX1280GetStatus(); // wake up radio:
  SX1280WaitOnBusy();
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] LoRa radio wake-up done.\r\n", (unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  buf[0] = 0x00; // 0x00 LDO mode - consumes more power or 0x01 DC-DC mode - consumes less power
  SX1280HalReadCommand(RADIO_SET_REGULATORMODE, buf, 1);
  SX1280WaitOnBusy();
  statusSX1280 = SX1280GetStatus();
  SX1280WaitOnBusy();
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] RADIO_SET_TXPARAMS send: 0x%02X, 0x%02X.\r\n",(unsigned int) xTaskGetTickCount(),buf[0],buf[1]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  volatile uint8_t  silicon_version;
  SX1280HalReadCommand(RADIO_GET_SILICON_VERSION, buf, 1); //returns 0x47 for new Silicon, 0x67 for original
  SX1280WaitOnBusy();
  silicon_version = buf[0];
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] Silicon version of SX1280: 0x%02X, ",(unsigned int) xTaskGetTickCount(),(unsigned int)silicon_version);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  switch (silicon_version)
  {
  case 0x47: // new silicon
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "new silicon.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
    break;
  case 0x67: // original silicon
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "original silicon.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
    break;
  default:
#if PRINTF_SX1280
    waitToPrint();
    npf_snprintf(uart_buf, 200, "unknown silicon version.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
// set sync word:
  ReadRegister_16bAddress(0x0944, buf, 3);
  // buf[0] is the status byte, buf[1] is content of Register 0x944 and buf[2] is content of Register 0x945
#if PLANTSENSOR
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 100, "%u [SX1280] [initThread] Sync word at reset time: 0x%02X%02X.\r\n",(unsigned int) xTaskGetTickCount(), buf[1], buf[2]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  buf[0] = buf[1];
  buf[1] = buf[2];
  buf[0] = ( buf[0] & ~0xF0 ) + ( 0x21 & 0xF0 );
  buf[1] = ( buf[1] & ~0xF0 ) + ( ( 0x21 & 0x0F ) << 4 );
//  buf[0] = 0x21;
//  buf[1] = 0x00;
  WriteRegister_16bAddress(0x944, buf, 2);
  ReadRegister_16bAddress(0x0944, rxbuf, 3);
  // buf[0] is the status byte, buf[1] is content of Register 0x944 and buf[1] is content of Register 0x945
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 100, "%u [SX1280] [initThread] Sync word set to: 0x%02X%02X.\r\n",(unsigned int) xTaskGetTickCount(), rxbuf[1], rxbuf[2]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#else
#if LISTENTOPLANTSENSOR
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 100, "%u [SX1280] [initThread] Sync word at reset time: 0x%02X%02X.\r\n",(unsigned int) xTaskGetTickCount(), buf[1], buf[2]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
  buf[0] = buf[1];
  buf[1] = buf[2];
  buf[0] = ( buf[0] & ~0xF0 ) + ( 0x21 & 0xF0 );
  buf[1] = ( buf[1] & ~0xF0 ) + ( ( 0x21 & 0x0F ) << 4 );
//  buf[0] = 0x21;
//  buf[1] = 0x00;
  WriteRegister_16bAddress(0x944, buf, 2);
  ReadRegister_16bAddress(0x0944, rxbuf, 3);
  // buf[0] is the status byte, buf[1] is content of Register 0x944 and buf[1] is content of Register 0x945
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 100, "%u [SX1280] [initThread] To listen to a plantsensor, sync word set to: 0x%02X%02X.\r\n",(unsigned int) xTaskGetTickCount(), rxbuf[1], rxbuf[2]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#else
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 100, "%u [SX1280] [initThread] Sync word at reset time: 0x%02X%02X, and will not be changed.\r\n",(unsigned int) xTaskGetTickCount(), buf[1], buf[2]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
#endif
#endif
  //radio read register (not in software of mbed)
  buf[0] = 0x0A;
  buf[1] = 0x14;
  buf[2] = 0X00;
  buf[3] = 0x00;
  SX1280HalWriteReadCommand(RADIO_READ_REGISTER, buf, 4, rxbuf);
  statusSX1280 = SX1280GetStatus();
#if PRINTF_SX1280
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] [initThread] RADIO_READ_REGISTER send: 0x%02X, 0x%02X.\r\n",(unsigned int) xTaskGetTickCount(),buf[0], buf[1]);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [SX1280] [initThread] RADIO_READ_REGISTER received: 0x%02X, 0x%02X.\r\n",(unsigned int) xTaskGetTickCount(),rxbuf[0], rxbuf[1]);
  huart2print(uart_buf, strlen(uart_buf));
#endif
}
