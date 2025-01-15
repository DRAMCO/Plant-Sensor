/*
 * An I2C and SPI driver for the NXP PCF2131 RTC
 *
 * Copyright 2013 Til-Technologies
 * Author: Renaud Cerrato <r.cerrato@til-technologies.fr>
 * Copyright 2020 NXP
 *
 * based on the other drivers in this same directory.
 *
 * Base file is a linux driver for PCF2131.
 * Adapted for use with STM32 by Sarah Goossens
 * Created on Jan 7, 2023
 *
 * PCF2131.c
 *
 */

#include "PCF2131.h"
#include "main.h"
#include <string.h>
#include "spi.h"
#include <time.h>
#include "../../App/app_rtc.h"

#define PRINTF_PCF2131 1
#define PRINTF_PCF2131_GET_STATUS 1


extern char              uart_buf[200];
extern uint32_t          rtc_pps;
extern uint32_t          gnss_pps;
extern uint64_t          timeStampInt;
extern int8_t            subSecondsCorrection;
extern int8_t            msCorrection;
extern uint64_t          rtosTimeStampStopRTC;
extern uint64_t          rtosTimeStampStartRTC;
extern float             tickSpeedToReference;

//extern SemaphoreHandle_t timeStampIntMutex;

uint8_t bTransferRequest = 0;
uint8_t pcf2131Buf[54];
uint8_t byteBuf[1];
uint8_t pcf2131TxBuffer[17];

void pcf2131_disable_poro()
{
  pcf2131_get_status(pcf2131Buf,16,0);
  pcf2131Buf[PCF2131_REG_CTRL1] &= ~PCF2131_BIT_CTRL1_POR_OVRD;
  byteBuf[0] = pcf2131Buf[PCF2131_REG_CTRL1];
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_CTRL1;
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_disable_poro] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  pcf2131_get_status(pcf2131Buf,16,0);
  if (!((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_POR_OVRD) == PCF2131_BIT_CTRL1_POR_OVRD))
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [pcf2131.c] [pcf2131_disable_poro] Power On Reset Override (PORO) Disabled.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
}

uint32_t pcf2131SecondAlarm(uint32_t setVal)
{ //     setVal =  0: Disable Second Alarm
  // 0 < setVal < 60: Enable Second Alarm, with the value given (1 to 59)
  //     setVal = 60: return only status of Second Alarm
  //    Return Value: 0 if Second Alarm is Disabled
  //                  value between 1 and 59 if Second Alarm is Enabled
  // Step 1: read SEC_ALARM REGISTER
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_A_MASK1, 1, byteBuf, 1, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_SEC_ALARM | 0x80); // add 0x80 to register for read, see product DS page 53
  pcf2131TxBuffer[1] = 0x00;
  pcf2131TxBuffer[2] = 0x00;
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  byteBuf[0] = pcf2131Buf[1]; // first byte is FF, shift 1 position, see also "log RTC implementation 20240225.doc"
#endif
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[PCF2131] SEC_ALARM = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[1], BYTE_TO_BINARY(pcf2131Buf[1]));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  // Step 2: Enable/Disable Second Alarm and set SEC_ALARM REGISTER
  if (setVal)
  {// Enable Second Alarm
    //byteBuf[0] &= ~PCF2131_AE_S; // Register SEC_ALARM (0x0E) Bit 7 - Enable Second Alarm (Set to 0)
    if (setVal < 60)
    {
      byteBuf[0] = bin2bcd((uint8_t) setVal); // Register SEC_ALARM (0x0E) Bit 7 - will always be Enabled (set to 0) by filling in setVal
    }
    else
    {
      return (uint32_t) byteBuf[0];
    }
  }
  else
  {// Disable Second Alarm
    byteBuf[0] |=  PCF2131_AE_S; // Register SEC_ALARM (0x0E) Bit 7 - Disable Second Alarm (Set to 1)
  }
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_SEC_ALARM, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  pcf2131TxBuffer[0] = PCF2131_SEC_ALARM; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131SecondAlarm] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
#endif
  // Step 3: Enable AIEA: Alarm Interrupt Enable in Interrupt B Mask1 (RTC_NINTB. This pin is configured as Wake Up pin on STM32WBA52:
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_B_MASK1, 1, byteBuf, 1, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  pcf2131TxBuffer[0] = (PCF2131_INT_B_MASK1 | 0x80); // add 0x80 to register for read, see product DS page 53
  pcf2131TxBuffer[1] = 0x00;
  pcf2131TxBuffer[2] = 0x00;
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  byteBuf[0] = pcf2131Buf[1]; // first byte is FF, shift 1 position, see also "log RTC implementation 20240225.doc"
#endif
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[PCF2131] INT_B_MASK1 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[1], BYTE_TO_BINARY(pcf2131Buf[1]));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  byteBuf[0] &= ~PCF2131_INT_B_MASK1_AIE; // Register INT_A_MASK1 (0x31) Bit 2 - Enable Alarm Interrupt (Set to 0)
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_A_MASK1, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  pcf2131TxBuffer[0] = PCF2131_INT_B_MASK1; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131SecondAlarm] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
#endif
  //todo also option to switch off AIEA
  // Step 4: Adapt CTRL2 register that an interrupt will be generated from the alarm flag
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL2, 1, byteBuf, 1, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  pcf2131TxBuffer[0] = (PCF2131_REG_CTRL2 | 0x80); // add 0x80 to register for read, see product DS page 53
  pcf2131TxBuffer[1] = 0x00;
  pcf2131TxBuffer[2] = 0x00;
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  byteBuf[0] = pcf2131Buf[1]; // first byte is FF, shift 1 position, see also "log RTC implementation 20240225.doc"
#endif
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "[PCF2131] REG_CTRL2 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[1], BYTE_TO_BINARY(pcf2131Buf[1]));
  huart2print(uart_buf, strlen(uart_buf));
#endif
  byteBuf[0] |= PCF2131_BIT_CTRL2_AIE; // Register REG_CTRL2 (0x1) Bit 1 - Enable Alarm Interrupt (Set to 1)
  byteBuf[0] &= ~PCF2131_BIT_CTRL2_AF; // Register REG_CTRL2 (0x1) Bit 4 - Clear Alarm Interrupt trigger (Set to 0)
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL2, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  pcf2131TxBuffer[0] = PCF2131_REG_CTRL2; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131SecondAlarm] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
#endif
  return (uint32_t) byteBuf[0];
}

uint32_t pcf2131SetAging(uint32_t agingOffsetPCF2131)
{
#if  PCF2131I2C
  IsI2C1Available();
//  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, pcf2131Buf, 16, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_AO; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = (uint8_t) (agingOffsetPCF2131 & 0x0F);
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 500) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131SetAging] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
}



void pcf2131_reset()
{
  //pcf2131_get_status(pcf2131Buf,16,0);

  // 1. Do a software reset (SR): 0010 1100 (2Ch) must be sent to register Reset (address 05h).
  // A software reset also triggers CPR and CTS
  // After software reset, the following mode is entered:
  // • 32.768 kHz CLKOUT active
  // • Power-On Reset Override (PORO) unchanged
  // • OTP not reloaded, OTPR unchanged.
  // • 24-hour mode is selected
  // • Battery switch-over function disabled, only one power supply (VDD)
  // • Temperature compensation enabled
  // • 100th second enabled
  // • Time 00:00:00.00
  // • Date 2001.01.01

  byteBuf[0] = PCF2131_SR_VAL_SoftReset;
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_SR_RESET, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_SR_RESET; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [PCF2131] Software reset done.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  // 2. Do an OTP refresh
  // The OTP refresh (see Section 7.3.2) should ideally be executed as the first instruction
  // after start-up and also after a reset due to an oscillator stop.
  // Each IC is calibrated during production and testing of the device. The calibration
  // parameters are stored on EPROM cells called One Time Programmable (OTP) cells. It is
  // recommended to process an OTP refresh once after the power is up and the oscillator is
  // operating stable. The OTP refresh takes less than 100 ms to complete.
  // To perform an OTP refresh, bit OTPR has to be cleared (set to logic 0) and then set to logic 1 again.
  // When read OTPR bit, its state is:
  // "0" until the OTP read state machine completes copying of the eFuse data into the shadow
  //     registers. This could be due to a POR event or to writing a 0 > 1 to the OTPR register bit.
  // "1" when the OTP read state machine completes copying to the shadow registers from the eFuse
  //     instances. During normal operation OTPR must be kept at 1 to prevent higher power usage.
  pcf2131Buf[PCF2131_CLKOUT_CTL] &= ~PCF2131_CLKOUT_CTL_OTPR;
  byteBuf[0] = pcf2131Buf[PCF2131_CLKOUT_CTL];
  //HAL_GPIO_TogglePin(GNSS_WakeUp_GPIO_Port, GNSS_WakeUp_Pin);  // toggle GNSS WakeUp for testing purposes
  //HAL_GPIO_TogglePin(GNSS_Reset_GPIO_Port, GNSS_Reset_Pin);    // toggle GNSS Reset for testing purposes
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_CLKOUT_CTL, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_CLKOUT_CTL; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  pcf2131Buf[PCF2131_CLKOUT_CTL] |= PCF2131_CLKOUT_CTL_OTPR;
  byteBuf[0] = pcf2131Buf[PCF2131_CLKOUT_CTL];
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_CLKOUT_CTL, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_CLKOUT_CTL; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_OTP_refresh] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  while ((pcf2131Buf[PCF2131_CLKOUT_CTL] & PCF2131_CLKOUT_CTL_OTPR) != PCF2131_CLKOUT_CTL_OTPR)
  {
    pcf2131_get_status(pcf2131Buf,16,0);
    vTaskDelay(10);
    {
#if PRINTF_PCF2131
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [pcf2131.c] [pcf2131_reset] waiting for OTP refresh is done.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
#endif
    }
  }
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [PCF2131] OTP refresh done.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  // 3. Clear the Oscillator Stop Flag (OSF)
  // The POR is active whenever the oscillator is stopped. The oscillator is considered to be
  // stopped during the time between power-on and stable crystal resonance (see Figure 9).
  // This time may be in the range of 200 ms to 2 s depending on temperature and supplyµ
  // voltage. Whenever an internal reset occurs, the oscillator stop flag is set (OSF set logic 1).
  pcf2131Buf[PCF2131_REG_SC] &= ~PCF2131_BIT_SC_OSF;
  byteBuf[0] = pcf2131Buf[PCF2131_REG_SC];
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_SC, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_SC; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_OSF_clear] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  // 4. Start the RTC_PPS via INTA pin (PA12 on STM32: RTC_NINTA).
  // HOST: this is used to compare the drift of the RTC clock with the gnss_PPS signal. This drift can be placed into the
  //       RTC temperature compensation engine (see 7.3.1 of PCF2131DS.pdf)
  // NODE: this is used as rtc_PPS signal to synchronize the clock of the STM32. The RTC temperature compensation engine of the NODE
  //       needs to be compensated with the clock synchronization and comparison with this rtc_PPS signal.
  // 4.1 set Second Interrupt in Control_1 register:
  pcf2131Buf[PCF2131_REG_CTRL1] |= PCF2131_BIT_CTRL1_SI;      // Register CTRL1 (0x00) Bit 0 - Set Second Interrupt (Set to 1)
  pcf2131Buf[PCF2131_REG_CTRL1] |= PCF2131_BIT_CTRL1_TC_DIS;  // Register CTRL1 (0x00) Bit 6 - Disable Temperature Compensation(Set to 1)
  byteBuf[0] = pcf2131Buf[PCF2131_REG_CTRL1];
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_CTRL1; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_OSF_clear] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  // 4.2 set INTA pin to generate pulsed signal when MSF flag is set
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_WD_CTL, 1, byteBuf, 1, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_REG_WD_CTL | 0x80); // add 0x80 to register for read, see product DS page 53
  pcf2131TxBuffer[1] = 0x00;
  pcf2131TxBuffer[2] = 0x00;
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  // Because first byte is FF, we shift all values in pcf2131Buf 1 position to the left to get same result for both SPI and I2C
  // See also "log RTC implementation 20240225.doc"
  byteBuf[0] = pcf2131Buf[1];
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  byteBuf[0] |= PCF2131_BIT_WD_CTL_TI_TP; // Register WD_CTL (0x35) Bit 5 - Set Interrupt pin to generate pulsed signal (Set to 1)
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_WD_CTL, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_WD_CTL; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_OSF_clear] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
#endif
  // 4.3 Set Second Interrupt A in INTA_MASK_1 register
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_A_MASK1, 1, byteBuf, 1, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_INT_A_MASK1 | 0x80); // add 0x80 to register for read, see product DS page 53
  pcf2131TxBuffer[1] = 0x00;
  pcf2131TxBuffer[2] = 0x00;
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  // Because first byte is FF, we shift all values in pcf2131Buf 1 position to the left to get same result for both SPI and I2C
  // See also "log RTC implementation 20240225.doc"
  byteBuf[0] = pcf2131Buf[1];
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  byteBuf[0] &= ~PCF2131_INT_A_MASK1_SI; // Register INTA_MASK_1 (0x31) Bit 4 - Set Second Interrupt to pin INTA (Set to 1)
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_A_MASK1, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_INT_A_MASK1; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_OSF_clear] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [PCF2131] NINTA PPS started.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
//  // 5. Synchronize RTC with HAL tick
//
//  byteBuf[0] = PCF2131_BIT_CTRL1_STOP;
//  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, byteBuf, 1, 1000);
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//
//  pcf2131_get_status(pcf2131Buf,16,0);
//
//  pcf2131Buf[PCF2131_REG_100th_SC] = pcf2131Buf[PCF2131_REG_100th_SC] + (xTaskGetTickCount() / 10 - pcf2131Buf[PCF2131_REG_100th_SC]);
//  byteBuf[0] = pcf2131Buf[PCF2131_REG_100th_SC];
//  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_100th_SC, 1, byteBuf, 1, 1000);
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
//
//  pcf2131Buf[PCF2131_REG_CTRL1] &= ~PCF2131_BIT_CTRL1_STOP;
//  byteBuf[0] = pcf2131Buf[PCF2131_REG_CTRL1];
//  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, byteBuf, 1, 1000);
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

//  // check OSF:
//
//  if ((pcf2131Buf[PCF2131_REG_SC] & PCF2131_BIT_SC_OSF) == PCF2131_BIT_SC_OSF)
//  {
//	#if PRINTF_PCF2131
//		waitToPrint();
//		npf_snprintf(uart_buf, 200, "%u [PCF2131] OSF not cleared, waiting for 2s to be sure oscillator is stable.\r\n",(unsigned int) xTaskGetTickCount());
//		huart2print(uart_buf, strlen(uart_buf));
//	#endif
//	HAL_Delay(2000);
//  }
//  else
//  {
//#if PRINTF_PCF2131
//	waitToPrint();
//	npf_snprintf(uart_buf, 200, "%u [PCF2131] Oscillator Stop Flag cleared.\r\n",(unsigned int) xTaskGetTickCount());
//	huart2print(uart_buf, strlen(uart_buf));
//#endif
//  }
}


void pcf2131_si_off(void)
{ // switch off second interrupt in INT_A_MASK1 register
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_A_MASK1, 1, byteBuf, 1, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_INT_A_MASK1 | 0x80); // add 0x80 to register for read, see product DS page 53
  pcf2131TxBuffer[1] = 0x00;
  pcf2131TxBuffer[2] = 0x00;
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  // Because first byte is FF, we shift all values in pcf2131Buf 1 position to the left to get same result for both SPI and I2C
  // See also "log RTC implementation 20240225.doc"
  byteBuf[0] = pcf2131Buf[1];

  waitToPrint();
//  npf_snprintf(uart_buf, 200, "\r\n	[PCF2131] Raw data: REG_CTRL5 = 0x%02X " BYTE_TO_BINARY_PATTERN "b\r\n					 ",pcf2131Buf[PCF2131_REG_CTRL5], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL5]));
  npf_snprintf(uart_buf, 200, "[PCF2131] INT_A_MASK1 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[1], BYTE_TO_BINARY(pcf2131Buf[1]));
  huart2print(uart_buf, strlen(uart_buf));
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
//  byteBuf[0] &= ~PCF2131_INT_A_MASK1_SI; // Register INTA_MASK_1 (0x31) Bit 4 - Set Second Interrupt to pin INTA (Set to 1)
  byteBuf[0] |= PCF2131_INT_A_MASK1_SI; // Register INTA_MASK_1 (0x31) Bit 4 - Set Second Interrupt to pin INTA (Set to 1)
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_INT_A_MASK1, 1, byteBuf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_INT_A_MASK1; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = byteBuf[0];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_si_off] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
#if PRINTF_PCF2131
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [PCF2131] NINTA PPS stopped.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
}

void pcf2131_get_status(uint8_t *pcf2131Buf, uint8_t val, uint8_t print)
{
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, pcf2131Buf, val, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_REG_CTRL1 | 0x80); // add 0x80 to register for read, see product DS page 53
  for(uint16_t i = 0; i < val; i++)
  {
	  pcf2131TxBuffer[1 + i] = 0x00;
  }
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, val+1, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  // Because first byte is FF, we shift all values in pcf2131Buf 1 position to the left to get same result for both SPI and I2C
  // See also "log RTC implementation 20240225.doc"
  for(uint16_t i = 0; i <= val; i++)
  {
	  pcf2131Buf[i] = pcf2131Buf[i+1];
  }
  pcf2131Buf[val+1] = 0x00;
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  if (print)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] REG_CTRL1 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_CTRL1], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL1]));
    huart2print(uart_buf, strlen(uart_buf));

#if PRINTF_PCF2131_GET_STATUS
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_SI) == PCF2131_BIT_CTRL1_SI)
    {
	  npf_snprintf(uart_buf, 200, "                             |||||||+-> Second interrupt Enabled.\r\n");
    }
    else
    {
	npf_snprintf(uart_buf, 200, "                             |||||||+-> Second interrupt Disabled.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_MI) == PCF2131_BIT_CTRL1_MI)
    {
      npf_snprintf(uart_buf, 200, "                             ||||||+--> Minute interrupt Enabled.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             ||||||+--> Minute interrupt Disabled.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_12_24) == PCF2131_BIT_CTRL1_12_24)
    {
	  npf_snprintf(uart_buf, 200, "                             |||||+---> 12-hour mode.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |||||+---> 24-hour mode.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_POR_OVRD) == PCF2131_BIT_CTRL1_POR_OVRD)
    {
	  npf_snprintf(uart_buf, 200, "                             ||||+----> Power On Reset Override (PORO) Enabled.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             ||||+----> Power On Reset Override (PORO) Disabled.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_100TH_S_DIS) == PCF2131_BIT_CTRL1_100TH_S_DIS)
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> 100th seconds counter Disabled.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> 100th seconds counter Enabled.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_STOP) == PCF2131_BIT_CTRL1_STOP)
    {
	  npf_snprintf(uart_buf, 200, "                             ||+------> RTC clock stopped.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             ||+------> RTC clock Runs.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_TC_DIS) == PCF2131_BIT_CTRL1_TC_DIS)
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> Temperature Compensation Disabled.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> Temperature Compensation Enabled.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_EXT_TEST) == PCF2131_BIT_CTRL1_EXT_TEST)
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> External clock test mode.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> Normal mode.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
#endif

    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] REG_CTRL2 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_CTRL2], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL2]));
    huart2print(uart_buf, strlen(uart_buf));

#if PRINTF_PCF2131_GET_STATUS
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             |||||||+-> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_AIE) == PCF2131_BIT_CTRL2_AIE)
    {
	  npf_snprintf(uart_buf, 200, "                             ||||||+--> Alarm Interrupt Enabled.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             ||||||+--> No interrupt generated from the alarm flag.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             |||||+---> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             ||||+----> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_AF) == PCF2131_BIT_CTRL2_AF)
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> Alarm Flag set when alarm triggered, must be cleared.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> No alarm interrupt triggered.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             ||+------> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_WDTF) == PCF2131_BIT_CTRL2_WDTF)
    {
      npf_snprintf(uart_buf, 200, "                             |+-------> Watchdog timer interrupt will be generated.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> No watchdog timer interrupt will be generated.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
   waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL2] & PCF2131_BIT_CTRL2_MSF) == PCF2131_BIT_CTRL2_MSF)
    {
      npf_snprintf(uart_buf, 200, "                             +--------> Minute or Second interrupt will be generated.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> No Minute or Second interrupt will be generated.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
#endif

    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] REG_CTRL3 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_CTRL3], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL3]));
    huart2print(uart_buf, strlen(uart_buf));

#if PRINTF_PCF2131_GET_STATUS
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BLIE) == PCF2131_BIT_CTRL3_BLIE)
    {
	  npf_snprintf(uart_buf, 200, "                             | |||||+-> An interrupt will be generated when Battery is low.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             | |||||+-> No interrupt will be generated when Battery is low.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BIE) == PCF2131_BIT_CTRL3_BIE)
    {
	  npf_snprintf(uart_buf, 200, "                             | ||||+--> battery flag (BF) is set: interrupt will be generated.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             | ||||+--> no interrupt generated from the battery flag (BF).\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BLF) == PCF2131_BIT_CTRL3_BLF)
    {
	  npf_snprintf(uart_buf, 200, "                             | |||+---> Battery Low, flag is automatically cleared with battery replacement.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             | |||+---> Battery status OK.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BF) == PCF2131_BIT_CTRL3_BF)
    {
	  npf_snprintf(uart_buf, 200, "                             | ||+----> Battery switch-over interrupt occurred, must be cleared.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             | ||+----> No battery switch-over interrupt occurred.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BTSE) == PCF2131_BIT_CTRL3_BTSE)
    {
	  npf_snprintf(uart_buf, 200, "                             | |+-----> Time stamp will be generated when battery switch-over interrupt occurs.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             | |+-----> No Time stamp will be generated when battery switch-over interrupt occurs.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    uint pwrmgt = (pcf2131Buf[PCF2131_REG_CTRL3] & 0b11100000 >> 5);
    switch (pwrmgt)
    {
      case 0b000:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function enabled in standard mode, battery low detection function enabled.\r\n");
        break;
      case 0b001:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function enabled in standard mode, battery low detection function disabled.\r\n");
        break;
      case 0b010:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function enabled in standard mode, battery low detection function disabled.\r\n");
        break;
      case 0b011:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function enabled in direct switching mode, battery low detection function enabled.\r\n");
        break;
      case 0b100:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function enabled in direct switching mode, battery low detection function disabled.\r\n");
        break;
      case 0b101:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function enabled in direct switching mode, battery low detection function disabled.\r\n");
        break;
      case 0b111:
        npf_snprintf(uart_buf, 150, "                             +-+------> battery switch-over function disabled (only power supply), battery low detection function disabled.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
#endif
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] REG_CTRL4 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_CTRL4], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL4]));
    huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_PCF2131_GET_STATUS
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             |||||||+-> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             ||||||+--> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             |||||+---> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             ||||+----> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF4) == PCF2131_BIT_CTRL4_TSF4)
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> Time stamp interrupt generated for pin NTS4.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> No time stamp interrupt generated for pin NTS4.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF3) == PCF2131_BIT_CTRL4_TSF3)
    {
	  npf_snprintf(uart_buf, 200, "                             ||+------> Time stamp interrupt generated for pin NTS3.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             ||+------> No time stamp interrupt generated for pin NTS3.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF2) == PCF2131_BIT_CTRL4_TSF2)
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> Time stamp interrupt generated for pin NTS2.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> No time stamp interrupt generated for pin NTS2.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF1) == PCF2131_BIT_CTRL4_TSF1)
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> Time stamp interrupt generated for pin NTS1.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> No time stamp interrupt generated for pin NTS1.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
#endif

    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] REG_CTRL5 = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_CTRL5], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_CTRL5]));
    huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_PCF2131_GET_STATUS
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             |||||||+-> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             ||||||+--> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             |||||+---> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "                             ||||+----> Not used.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE4) == PCF2131_BIT_CTRL5_TSIE4)
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> Interrupt will be generated when time stamp flag is set of NTS4.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |||+-----> No interrupt will be generated from time stamp flag of NTS4.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE3) == PCF2131_BIT_CTRL5_TSIE3)
    {
	  npf_snprintf(uart_buf, 200, "                             ||+------> Interrupt will be generated when time stamp flag is set of NTS3.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             ||+------> No interrupt will be generated from time stamp flag of NTS3.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE2) == PCF2131_BIT_CTRL5_TSIE2)
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> Interrupt will be generated when time stamp flag is set of NTS2.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             |+-------> No interrupt will be generated from time stamp flag of NTS2.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_CTRL5] & PCF2131_BIT_CTRL5_TSIE1) == PCF2131_BIT_CTRL5_TSIE1)
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> Interrupt will be generated when time stamp flag is set of NTS1.\r\n");
    }
    else
    {
	  npf_snprintf(uart_buf, 200, "                             +--------> No interrupt will be generated from time stamp flag of NTS1.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
#endif
    waitToPrint();
    if ((pcf2131Buf[PCF2131_REG_SR_RESET] & 0b00100100) == 0b00100100)
    {
      npf_snprintf(uart_buf, 200, "[PCF2131] Software reset register value correct = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_SR_RESET], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_SR_RESET]));
    }
    else
    {
      npf_snprintf(uart_buf, 200, "[PCF2131] Software reset register error! 0b00100100 expected, value = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_SR_RESET], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_SR_RESET]));
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] SR_RESET      = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_SR_RESET], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_SR_RESET]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] SEC_ALARM     = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_SEC_ALARM], BYTE_TO_BINARY(pcf2131Buf[PCF2131_SEC_ALARM]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] MIN_ALARM     = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_MIN_ALARM], BYTE_TO_BINARY(pcf2131Buf[PCF2131_MIN_ALARM]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] HOUR_ALARM    = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_HOUR_ALARM], BYTE_TO_BINARY(pcf2131Buf[PCF2131_HOUR_ALARM]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] DAY_ALARM     = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_DAY_ALARM], BYTE_TO_BINARY(pcf2131Buf[PCF2131_DAY_ALARM]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] WEEKDAY_ALARM = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_WEEKDAY_ALARM], BYTE_TO_BINARY(pcf2131Buf[PCF2131_WEEKDAY_ALARM]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] CLKOUT_CTL    = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_CLKOUT_CTL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_CLKOUT_CTL]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] TS1_CTRL      = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_TS1_CTRL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_TS1_CTRL]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] TS2_CTRL      = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_TS2_CTRL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_TS2_CTRL]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] TS3_CTRL      = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_TS3_CTRL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_TS3_CTRL]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] TS4_CTRL      = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_TS4_CTRL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_TS4_CTRL]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] AGING_OFFSET  = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_AO], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_AO]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] INT_A_MASK1   = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_INT_A_MASK1], BYTE_TO_BINARY(pcf2131Buf[PCF2131_INT_A_MASK1]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] INT_A_MASK2   = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_INT_A_MASK2], BYTE_TO_BINARY(pcf2131Buf[PCF2131_INT_B_MASK2]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] INT_B_MASK1   = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_INT_B_MASK1], BYTE_TO_BINARY(pcf2131Buf[PCF2131_INT_A_MASK1]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] INT_B_MASK2   = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_INT_B_MASK2], BYTE_TO_BINARY(pcf2131Buf[PCF2131_INT_B_MASK2]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] WD_CTL        = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_WD_CTL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_WD_CTL]));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] WD_VAL        = 0x%02X 0b" BYTE_TO_BINARY_PATTERN "\r\n",pcf2131Buf[PCF2131_REG_WD_VAL], BYTE_TO_BINARY(pcf2131Buf[PCF2131_REG_WD_VAL]));
    huart2print(uart_buf, strlen(uart_buf));
  }
}

uint8_t pcf2131_get_time(PCF2131_time_t * PCF2131_time)
{
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_100th_SC, 1, pcf2131Buf, 8, 500); // read ONLY time info to minimise lead time
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_REG_100th_SC | 0x80); // add 0x80 to register for read, see product DS page 53
  for(uint16_t i = 0; i < 8; i++)
  {
	  pcf2131TxBuffer[1 + i] = 0x00;
  }
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 9, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  // Because first byte is FF, we shift all values in pcf2131Buf 1 position to the left to get same result for both SPI and I2C
  // See also "log RTC implementation 20240225.doc"
  for(uint16_t i = 0; i < 9; i++)
  {
	  pcf2131Buf[i] = pcf2131Buf[i+1];
  }
  pcf2131Buf[9] = 0x00;
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
//  if (pcf2131Buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BLF)
//  {
//#if PRINTF_PCF2131
//	waitToPrint();
//	npf_snprintf(uart_buf, 200, "%u [PCF2131] Low voltage detected, check/replace RTC battery.\r\n",(unsigned int) xTaskGetTickCount());
//	huart2print(uart_buf, strlen(uart_buf));
//#endif
//  }
  // Clock integrity is not guaranteed when OSF flag is set.
  if (pcf2131Buf[PCF2131_REG_SC - 0x06] & PCF2131_BIT_SC_OSF) // 0x06 needs to be subtracted as PCF2131_REG_100th_SC is now starting on index 0 iso index 0x06
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [PCF2131] Oscillator stop detected, date/time is not reliable.\r\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }

//#if PRINTF_PCF2131
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [PCF2131] Raw data: 100thSec=%02X, sec=%02X, min=%02X, hr=%02X.\r\n",
//			(unsigned int) xTaskGetTickCount(), pcf2131Buf[PCF2131_REG_100th_SC], pcf2131Buf[PCF2131_REG_SC], pcf2131Buf[PCF2131_REG_MN], pcf2131Buf[PCF2131_REG_HR]);
//  huart2print(uart_buf, strlen(uart_buf));
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [PCF2131] Raw data: mday=%02X, wday=%02X, mon=%02X, year=%02X.\r\n",
//			(unsigned int) xTaskGetTickCount(), pcf2131Buf[PCF2131_REG_DM], pcf2131Buf[PCF2131_REG_DW], pcf2131Buf[PCF2131_REG_MO], pcf2131Buf[PCF2131_REG_YR]);
//  huart2print(uart_buf, strlen(uart_buf));
//#endif

  PCF2131_time->subSeconds = bcd2bin(pcf2131Buf[PCF2131_REG_100th_SC - 0x06]);        // rtc 100th seconds: 0-99
  PCF2131_time->seconds    = bcd2bin(pcf2131Buf[PCF2131_REG_SC       - 0x06] & 0x7F); // rtc seconds: 0-59
  PCF2131_time->minutes    = bcd2bin(pcf2131Buf[PCF2131_REG_MN       - 0x06] & 0x7F); // rtc minutes: 0-59
  PCF2131_time->hours      = bcd2bin(pcf2131Buf[PCF2131_REG_HR       - 0x06] & 0x3F); // rtc hour: 0-23
  PCF2131_time->dayOfMonth = bcd2bin(pcf2131Buf[PCF2131_REG_DM       - 0x06] & 0x3F); // rtc day of month: 1-31
  PCF2131_time->dayOfWeek  =         pcf2131Buf[PCF2131_REG_DW       - 0x06] & 0x07;  // rtc WEEKDAYS: 0-6 (0 = Sunday)
  PCF2131_time->month      = bcd2bin(pcf2131Buf[PCF2131_REG_MO       - 0x06] & 0x1F); // rtc month: 1-12
  PCF2131_time->year       = bcd2bin(pcf2131Buf[PCF2131_REG_YR       - 0x06]);        // rtc year: 0-99
  PCF2131_time->year      += 2000;                                                    // real year

  // create epoch time:

  //set_unixTime(*PCF2131_time);
//  struct tm t;
//  time_t t_of_day;
//
//  t.tm_year  = time->year-1900;  // Year - 1900
//  t.tm_mon   = time->month-1;    // Month, where 0 = jan
//  t.tm_mday  = time->dayOfMonth; // Day of the month
//  t.tm_hour  = time->hours;      // 24 hours format
//  t.tm_min   = time->minutes;
//  t.tm_sec   = time->seconds;
//  t.tm_isdst = -1;               // Is DST on? 1 = yes, 0 = no, -1 = unknown
//  t_of_day   = mktime(&t);
//
//  time->unixTime = t_of_day;

//    struct tm  ts;
//    char       buft[80];

//    // Get current time
//    time(&t_of_day);

//    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
//    ts = *localtime(&t_of_day);
//    strftime(buft, sizeof(buft), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
//
//#if PRINTF_APP_RTC
//	waitToPrint();
//	npf_snprintf(uart_buf, 200, "%u [app_rtc] %s.\r\n", (unsigned int) xTaskGetTickCount(), buft);
//	huart2print(uart_buf, strlen(uart_buf));
//#endif

//#if PRINTF_PCF2131
//	waitToPrint();
//	npf_snprintf(uart_buf, 200, "%u [app_rtc] seconds since the Epoch: %ld.\r\n", (unsigned int) xTaskGetTickCount(), (long) t_of_day);
//	huart2print(uart_buf, strlen(uart_buf));
//#endif

  return 0;
}

//uint8_t set_unixTime(PCF2131_time_t *PCF2131_time)
//{
//  struct tm t;
//  time_t epoch;
//
//  t.tm_year  = PCF2131_time.year-1900;  // Year - 1900
//  t.tm_mon   = PCF2131_time.month-1;    // Month, where 0 = jan
//  t.tm_mday  = PCF2131_time.dayOfMonth; // Day of the month
//  t.tm_hour  = PCF2131_time.hours;      // 24 hours format
//  t.tm_min   = PCF2131_time.minutes;
//  t.tm_sec   = PCF2131_time.seconds;
//  t.tm_isdst = -1;               // Is DST on? 1 = yes, 0 = no, -1 = unknown
//  epoch   = mktime(&t);
//
//  PCF2131_time.unixTime = epoch;
//
////  struct tm  ts;
////  char buft[80];
////  // Get current time
////  time(&epoch);
////  // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
////  ts = *localtime(&epoch);
////  strftime(buft, sizeof(buft), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
////
////#if PRINTF_PCF2131
////  waitToPrint();
////  npf_snprintf(uart_buf, 200, "%u [PCF2131] [set_unixTime] %s.\r\n", (unsigned int) xTaskGetTickCount(), buft);
////  huart2print(uart_buf, strlen(uart_buf));
////#endif
//
//
//
//  return 0;
//}

uint8_t pcf2131_set_time(PCF2131_time_t * PCF2131_time)
{
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Read(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, pcf2131Buf, 16, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = (PCF2131_REG_CTRL1 | 0x80); // add 0x80 to register for read, see product DS page 53
  for(uint16_t i = 0; i < 16; i++)
  {
	  pcf2131TxBuffer[1 + i] = 0x00;
  }
  if (HAL_SPI_TransmitReceive(&hspi3, pcf2131TxBuffer, pcf2131Buf, 17, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_get_status] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  // Because first byte is FF, we shift all values in pcf2131Buf 1 position to the left to get same result for both SPI and I2C
  // See also "log RTC implementation 20240225.doc"
  for(uint16_t i = 0; i < 17; i++)
  {
	  pcf2131Buf[i] = pcf2131Buf[i+1];
  }
  pcf2131Buf[17] = 0x00;
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  // Stop the clock:
  // Clear prescaler:
  // set new date & time:
  pcf2131Buf[PCF2131_REG_CTRL1]   |= PCF2131_BIT_CTRL1_STOP;                // Register CTRL1 (0x00) Bit 5 - STOP the clock (Set to 1)
  pcf2131Buf[PCF2131_REG_SR_RESET] = PCF2131_SR_VAL_CPR;                    // CPR: clear prescaler. To set the time for RTC mode, the clear prescaler instruction is needed.
                                                                            // Before sending this instruction, it is mandatory to first set stop by the STOP bit.
#if SENSOR_HOST
//  pcf2131Buf[PCF2131_REG_100th_SC] = bin2bcd(TIME_DELAY_TO_START_RTC);      // 100th Second increment of 60ms to count for extra build in delay
  pcf2131Buf[PCF2131_REG_100th_SC] = bin2bcd(subSecondsCorrection / ((int)tickSpeedToReference));          // 100th Second increment of 60ms to count for extra build in delay
//  pcf2131Buf[PCF2131_REG_100th_SC] = 0x00;                                   // 100th Second increment of 60ms to count for extra build in delay
#else
  pcf2131Buf[PCF2131_REG_100th_SC] = bin2bcd(PCF2131_time->subSeconds);     // the increment of TIME_DELAY_TO_START_RTC is already counted for in PCF2131_time->subSeconds (see app_network_connect.c)
#endif
  pcf2131Buf[PCF2131_REG_SC]       = bin2bcd(PCF2131_time->seconds);        // this will also clear OSF flag
  pcf2131Buf[PCF2131_REG_MN]       = bin2bcd(PCF2131_time->minutes);        // minutes: 0 - 59
  pcf2131Buf[PCF2131_REG_HR]       = bin2bcd(PCF2131_time->hours);          // hours: 0 - 23
  pcf2131Buf[PCF2131_REG_DM]       = bin2bcd(PCF2131_time->dayOfMonth);     // day of Month: 1 - 31
  pcf2131Buf[PCF2131_REG_DW]       = PCF2131_time->dayOfWeek & 0x07;        // day of week: 0 = Sun, 6 = Sat
  pcf2131Buf[PCF2131_REG_MO]       = bin2bcd(PCF2131_time->month);          // month: 1 - 12
  pcf2131Buf[PCF2131_REG_YR]       = bin2bcd(PCF2131_time->year);           // year: 0 - 99
  rtosTimeStampStopRTC = xTaskGetTickCount();
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, pcf2131Buf, 16, 1000);
  IsI2C1Available();
  /* Before starting a new communication transfer, you need to check the current state of the peripheral; if its busy you need to wait
     for the end of current transfer before starting a new one. For simplicity reasons, this example is just waiting till the end of
     the transfer, but application may perform other tasks while transfer operation is ongoing. */
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_CTRL1; // add nothing to register to write, see product DS page 53
  for(uint16_t i = 0; i < 16; i++)
  {
	  pcf2131TxBuffer[1 + i] = pcf2131Buf[i];
  }
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 17, 500) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_reset_OSF_clear] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
//#if PRINTF_PCF2131
//  waitToPrint();
//  npf_snprintf(uart_buf, 200, "%u [PCF2131] New time and date set.\r\n", (unsigned int) xTaskGetTickCount());
//  huart2print(uart_buf, strlen(uart_buf));
//#endif
  return 0;
}

uint8_t pcf2131_start_clock(void)
{
  pcf2131Buf[PCF2131_REG_CTRL1] &= ~PCF2131_BIT_CTRL1_STOP; // Register CTRL1 (0x00) Bit 5 - START the clock (Set to 0)
#if SENSOR_HOST
  vTaskDelay(msCorrection);
#else
//  vTaskDelay(15);
#endif
  rtosTimeStampStartRTC = xTaskGetTickCount();
#if  PCF2131I2C
  IsI2C1Available();
  HAL_I2C_Mem_Write(&hi2c1, PCF2131_ADDRESS, PCF2131_REG_CTRL1, 1, pcf2131Buf, 1, 1000);
  IsI2C1Available();
#else
  IsSPI3Available();
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_RESET);
  pcf2131TxBuffer[0] = PCF2131_REG_CTRL1; // add nothing to register to write, see product DS page 53
  pcf2131TxBuffer[1] = pcf2131Buf[PCF2131_REG_CTRL1];
  if (HAL_SPI_Transmit(&hspi3, pcf2131TxBuffer, 2, 100) != HAL_OK)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "[PCF2131] [pcf2131_start_clock] SPI issue.\r\n");
    huart2print(uart_buf, strlen(uart_buf));
#endif
  }
  HAL_GPIO_WritePin(CS_RTC_GPIO_Port, CS_RTC_Pin, GPIO_PIN_SET);
  IsSPI3Available();
#endif
  rtc_pps = 0;
  return 0;
}


uint8_t pcf2131_is_running(void)
{// Is the clock running? 0 = no, 1 = yes
  pcf2131_get_status(pcf2131Buf, 16, 0);
  if ((pcf2131Buf[PCF2131_REG_CTRL1] & PCF2131_BIT_CTRL1_STOP) == PCF2131_BIT_CTRL1_STOP)
  {
#if PRINTF_PCF2131
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [pcf2131] RTC is not running.\r\n", (unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
#endif
    return 0;
  }
  else
  {
//#if PRINTF_PCF2131
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [pcf2131] RTC runs.\r\n", (unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
//#endif
    return 1;
  }
}

//static int pcf2131_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
//{
//	struct pcf2131 *pcf2131 = dev_get_drvdata(dev);
//	int val, touser = 0;
//	int ret;
//
//	switch (cmd)
//	{
//	case RTC_VL_READ:
//		ret = regmap_read(pcf2131->regmap, PCF2131_REG_CTRL3, &val);
//		if (ret)
//			return ret;
//
//		if (val & PCF2131_BIT_CTRL3_BLF)
//			touser |= RTC_VL_BACKUP_LOW;
//
//		if (val & PCF2131_BIT_CTRL3_BF)
//			touser |= RTC_VL_BACKUP_SWITCH;
//
////		return put_user(touser, (unsigned int __user *)arg);
//
//	case RTC_VL_CLR:
//		return regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL3, PCF2131_BIT_CTRL3_BF, 0);
//
//	default:
//		return -ENOIOCTLCMD;
//	}
//}

//static const struct rtc_class_ops pcf2131_rtc_ops = {
//	.ioctl		= pcf2131_rtc_ioctl,
//	.read_time	= pcf2131_rtc_read_time,
//	.set_time	= pcf2131_rtc_set_time,
//};

/* watchdog driver */

//static int pcf2131_wdt_ping(struct watchdog_device *wdd)
//{
//	struct pcf2131 *pcf2131 = watchdog_get_drvdata(wdd);
//	return regmap_write(pcf2131->regmap, PCF2131_REG_WD_VAL, wdd->timeout);
//}

/* Restart watchdog timer if feature is active. */
//static int pcf2131_wdt_active_ping(struct watchdog_device *wdd)
//{
//	int ret = 0;
//	if (watchdog_active(wdd))
//	{
//		ret = pcf2131_wdt_ping(wdd);
//		if (ret)
//		{
//			dev_err(wdd->parent, "%s: watchdog restart failed, ret=%d\n", __func__, ret);
//		}
//	}
//	return ret;
//}
//
//static int pcf2131_wdt_start(struct watchdog_device *wdd)
//{
//	return pcf2131_wdt_ping(wdd);
//}
//
//static int pcf2131_wdt_stop(struct watchdog_device *wdd)
//{
//	struct pcf2131 *pcf2131 = watchdog_get_drvdata(wdd);
//	return regmap_write(pcf2131->regmap, PCF2131_REG_WD_VAL, PCF2131_WD_VAL_STOP);
//}
//
//static int pcf2131_wdt_set_timeout(struct watchdog_device *wdd, unsigned int new_timeout)
//{
//	dev_dbg(wdd->parent, "new watchdog timeout: %is (old: %is)\n", new_timeout, wdd->timeout);
//	wdd->timeout = new_timeout;
//	return pcf2131_wdt_active_ping(wdd);
//}
//
//static const struct watchdog_info pcf2131_wdt_info =
//{
//	.identity = "NXP PCF2131 Watchdog",
//	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
//};
//
//static const struct watchdog_ops pcf2131_watchdog_ops =
//{
//	.owner = THIS_MODULE,
//	.start = pcf2131_wdt_start,
//	.stop = pcf2131_wdt_stop,
//	.ping = pcf2131_wdt_ping,
//	.set_timeout = pcf2131_wdt_set_timeout,
//};
//
///* sysfs interface */
//
//static ssize_t timestamp0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
//{
//	struct pcf2131 *pcf2131 = dev_get_drvdata(dev->parent);
//	int ret;
//	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL4, PCF2131_BIT_CTRL4_TSF1, 0);
//	if (ret)
//	{
//		dev_err(dev, "%s: update ctrl1 ret=%d\n", __func__, ret);
//		return ret;
//	}
//	return count;
//};
//
//static ssize_t timestamp0_show(struct device *dev, struct device_attribute *attr, char *buf)
//{
//	struct pcf2131 *pcf2131 = dev_get_drvdata(dev->parent);
//	struct rtc_time tm;
//	int ret;
//	unsigned char data[25];
//
//	ret = regmap_bulk_read(pcf2131->regmap, PCF2131_REG_CTRL1, data, sizeof(data));
//	if (ret)
//	{
//		dev_err(dev, "%s: read error ret=%d\n", __func__, ret);
//		return ret;
//	}
//
//	dev_dbg(dev, "%s: raw data is cr1=%02x, cr2=%02x, cr3=%02x, ts_sc=%02x, ts_mn=%02x, ts_hr=%02x, ts_dm=%02x, ts_mo=%02x, ts_yr=%02x\n",
//		__func__, data[PCF2131_REG_CTRL1], data[PCF2131_REG_CTRL2],
//		data[PCF2131_REG_CTRL3], data[PCF2131_REG_TS1_SC],
//		data[PCF2131_REG_TS1_MN], data[PCF2131_REG_TS1_HR],
//		data[PCF2131_REG_TS1_DM], data[PCF2131_REG_TS1_MO],
//		data[PCF2131_REG_TS1_YR]);
//
//	if (!(data[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF1))
//		return 0;
//
//	tm.tm_sec = bcd2bin(data[PCF2131_REG_TS1_SC] & 0x7F);
//	tm.tm_min = bcd2bin(data[PCF2131_REG_TS1_MN] & 0x7F);
//	tm.tm_hour = bcd2bin(data[PCF2131_REG_TS1_HR] & 0x3F);
//	tm.tm_mday = bcd2bin(data[PCF2131_REG_TS1_DM] & 0x3F);
//	/* TS_MO register (month) value range: 1-12 */
//	tm.tm_mon = bcd2bin(data[PCF2131_REG_TS1_MO] & 0x1F) - 1;
//	tm.tm_year = bcd2bin(data[PCF2131_REG_TS1_YR]);
//	if (tm.tm_year < 70)
//		tm.tm_year += 100; /* assume we are in 1970...2069 */
//
//	ret = rtc_valid_tm(&tm);
//	if (ret)
//		return ret;
//
//	return sprintf(buf, "%llu\n", (unsigned long long)rtc_tm_to_time64(&tm));
//};
//
//static DEVICE_ATTR_RW(timestamp0);
//
//static struct attribute *pcf2131_attrs[] =
//{
//	&dev_attr_timestamp0.attr,
//	NULL
//};
//
//static const struct attribute_group pcf2131_attr_group =
//{
//	.attrs	= pcf2131_attrs,
//};
//
//static int pcf2131_probe(struct device *dev, struct regmap *regmap, const char *name, bool has_nvmem)
//{
//	struct pcf2131 *pcf2131;
//	u32 wdd_timeout;
//	int ret = 0;
//
//	dev_dbg(dev, "%s\n", __func__);
//
//	pcf2131 = devm_kzalloc(dev, sizeof(*pcf2131), GFP_KERNEL);
//	if (!pcf2131)
//		return -ENOMEM;
//
//	pcf2131->regmap = regmap;
//	dev_set_drvdata(dev, pcf2131);
//	pcf2131->rtc = devm_rtc_allocate_device(dev);
//	if (IS_ERR(pcf2131->rtc))
//		return PTR_ERR(pcf2131->rtc);
//	pcf2131->rtc->ops = &pcf2131_rtc_ops;
//	pcf2131->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
//	pcf2131->rtc->range_max = RTC_TIMESTAMP_END_2099;
//	pcf2131->rtc->set_start_time = true; /* Sets actual start to 1970 */
//
//	pcf2131->wdd.parent = dev;
//	pcf2131->wdd.info = &pcf2131_wdt_info;
//	pcf2131->wdd.ops = &pcf2131_watchdog_ops;
//	pcf2131->wdd.min_timeout = PCF2131_WD_VAL_MIN;
//	pcf2131->wdd.max_timeout = PCF2131_WD_VAL_MAX;
//	pcf2131->wdd.timeout = PCF2131_WD_VAL_DEFAULT;
//	pcf2131->wdd.min_hw_heartbeat_ms = 500;
//	pcf2131->wdd.status = WATCHDOG_NOWAYOUT_INIT_STATUS;
//
//	watchdog_set_drvdata(&pcf2131->wdd, pcf2131);
//
//	/*
//	 * Watchdog timer enabled and reset pin /RST activated when timed out.
//	 * Select 1Hz clock source for watchdog timer.
//	 * Note: Countdown timer disabled and not available.
//	 */
//	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_WD_CTL,
//				 PCF2131_BIT_WD_CTL_CD |
//				 PCF2131_BIT_WD_CTL_TF1 |
//				 PCF2131_BIT_WD_CTL_TF0,
//				 PCF2131_BIT_WD_CTL_CD |
//				 PCF2131_BIT_WD_CTL_TF1);
//	if (ret) {
//		dev_err(dev, "%s: watchdog config (wd_ctl) failed\n", __func__);
//		return ret;
//	}
//
//	/* Test if watchdog timer is started by bootloader */
//	ret = regmap_read(pcf2131->regmap, PCF2131_REG_WD_VAL, &wdd_timeout);
//	if (ret)
//		return ret;
//
//	if (wdd_timeout)
//		set_bit(WDOG_HW_RUNNING, &pcf2131->wdd.status);
//
//#ifdef CONFIG_WATCHDOG
//	ret = devm_watchdog_register_device(dev, &pcf2131->wdd);
//	if (ret)
//		return ret;
//#endif /* CONFIG_WATCHDOG */
//
//	/*
//	 * Disable battery low/switch-over timestamp and interrupts.
//	 * Clear battery interrupt flags which can block new trigger events.
//	 * Note: This is the default chip behaviour but added to ensure
//	 * correct tamper timestamp and interrupt function.
//	 */
//	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL3,
//				 PCF2131_BIT_CTRL3_BTSE |
//				 PCF2131_BIT_CTRL3_BIE |
//				 PCF2131_BIT_CTRL3_BLIE, 0);
//	if (ret)
//	{
//		dev_err(dev, "%s: interrupt config (ctrl3) failed\n", __func__);
//		return ret;
//	}
//
//	/*
//	 * Enable timestamp function and store timestamp of first trigger
//	 * event until TSF1 interrupt flags are cleared.
//	 */
//	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_TS1_CTRL,
//				 PCF2131_BIT_TS1_CTRL_TSOFF |
//				 PCF2131_BIT_TS1_CTRL_TSM,
//				 PCF2131_BIT_TS1_CTRL_TSM);
//	if (ret) {
//		dev_err(dev, "%s: tamper detection config (ts1_ctrl) failed\n",
//			__func__);
//		return ret;
//	}
//
//	/*
//	 * Enable interrupt generation when TSF1 timestamp flags
//	 * are set. Interrupt signal is an open-drain output and can be
//	 * left floating if unused.
//	 */
//	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL5, PCF2131_BIT_CTRL5_TSIE1, PCF2131_BIT_CTRL5_TSIE1);
//	if (ret)
//	{
//		dev_err(dev, "%s: tamper detection config (ctrl5) failed\n", __func__);
//		return ret;
//	}
//	ret = rtc_add_group(pcf2131->rtc, &pcf2131_attr_group);
//	if (ret)
//	{
//		dev_err(dev, "%s: tamper sysfs registering failed\n", __func__);
//		return ret;
//	}
//
//	return devm_rtc_register_device(pcf2131->rtc);
//}

uint8_t bcd2bin(uint8_t val)
{
  return (val & 0x0F) + (val >> 4) * 10;
}

uint8_t bin2bcd(uint8_t val)
{
  return ((val / 10) << 4) + val % 10;
}


//
//#ifdef CONFIG_OF
//static const struct of_device_id pcf2131_of_match[] = {
//	{ .compatible = "nxp,pcf2131" },
//	{}
//};
//MODULE_DEVICE_TABLE(of, pcf2131_of_match);
//#endif
//
//#if IS_ENABLED(CONFIG_I2C)
//
//static int pcf2131_i2c_write(void *context, const void *data, size_t count)
//{
//	struct device *dev = context;
//	struct i2c_client *client = to_i2c_client(dev);
//	int ret;
//
//	ret = i2c_master_send(client, data, count);
//	if (ret != count)
//		return ret < 0 ? ret : -EIO;
//
//	return 0;
//}
//
//static int pcf2131_i2c_gather_write(void *context,
//				const void *reg, size_t reg_size,
//				const void *val, size_t val_size)
//{
//	struct device *dev = context;
//	struct i2c_client *client = to_i2c_client(dev);
//	int ret;
//	void *buf;
//
//	if (WARN_ON(reg_size != 1))
//		return -EINVAL;
//
//	buf = kmalloc(val_size + 1, GFP_KERNEL);
//	if (!buf)
//		return -ENOMEM;
//
//	memcpy(buf, reg, 1);
//	memcpy(buf + 1, val, val_size);
//
//	ret = i2c_master_send(client, buf, val_size + 1);
//
//	kfree(buf);
//
//	if (ret != val_size + 1)
//		return ret < 0 ? ret : -EIO;
//
//	return 0;
//}
//
//static int pcf2131_i2c_read(void *context, const void *reg, size_t reg_size,
//				void *val, size_t val_size)
//{
//	struct device *dev = context;
//	struct i2c_client *client = to_i2c_client(dev);
//	int ret;
//
//	if (WARN_ON(reg_size != 1))
//		return -EINVAL;
//
//	ret = i2c_master_send(client, reg, 1);
//	if (ret != 1)
//		return ret < 0 ? ret : -EIO;
//
//	ret = i2c_master_recv(client, val, val_size);
//	if (ret != val_size)
//		return ret < 0 ? ret : -EIO;
//
//	return 0;
//}
//
///*
// * The reason we need this custom regmap_bus instead of using regmap_init_i2c()
// * is that the STOP condition is required between set register address and
// * read register data when reading from registers.
// */
//static const struct regmap_bus pcf2131_i2c_regmap = {
//	.write = pcf2131_i2c_write,
//	.gather_write = pcf2131_i2c_gather_write,
//	.read = pcf2131_i2c_read,
//};
//
//static struct i2c_driver pcf2131_i2c_driver;
//
//static int pcf2131_i2c_probe(struct i2c_client *client,
//				const struct i2c_device_id *id)
//{
//	struct regmap *regmap;
//	static const struct regmap_config config = {
//		.reg_bits = 8,
//		.val_bits = 8,
//		.max_register = 0x36,
//	};
//
//	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
//		return -ENODEV;
//
//	regmap = devm_regmap_init(&client->dev, &pcf2131_i2c_regmap,
//					&client->dev, &config);
//	if (IS_ERR(regmap)) {
//		dev_err(&client->dev, "%s: regmap allocation failed: %ld\n",
//			__func__, PTR_ERR(regmap));
//		return PTR_ERR(regmap);
//	}
//
//	return pcf2131_probe(&client->dev, regmap,
//			     pcf2131_i2c_driver.driver.name, id->driver_data);
//}
//
//static const struct i2c_device_id pcf2131_i2c_id[] = {
//	{ "pcf2131", 1 },
//	{ }
//};
//MODULE_DEVICE_TABLE(i2c, pcf2131_i2c_id);
//
//static struct i2c_driver pcf2131_i2c_driver = {
//	.driver		= {
//		.name	= "rtc-pcf2131-i2c",
//		.of_match_table = of_match_ptr(pcf2131_of_match),
//	},
//	.probe		= pcf2131_i2c_probe,
//	.id_table	= pcf2131_i2c_id,
//};
//
//static int pcf2131_i2c_register_driver(void)
//{
//	return i2c_add_driver(&pcf2131_i2c_driver);
//}
//
//static void pcf2131_i2c_unregister_driver(void)
//{
//	i2c_del_driver(&pcf2131_i2c_driver);
//}
//
//#else
//
//static int pcf2131_i2c_register_driver(void)
//{
//	return 0;
//}
//
//static void pcf2131_i2c_unregister_driver(void)
//{
//}
//
//#endif
//
//#if IS_ENABLED(CONFIG_SPI_MASTER)
//
//static struct spi_driver pcf2131_spi_driver;
//
//static int pcf2131_spi_probe(struct spi_device *spi)
//{
//	static const struct regmap_config config = {
//		.reg_bits = 8,
//		.val_bits = 8,
//		.read_flag_mask = 0xa0,
//		.write_flag_mask = 0x20,
//		.max_register = 0x36,
//	};
//	struct regmap *regmap;
//
//	regmap = devm_regmap_init_spi(spi, &config);
//	if (IS_ERR(regmap)) {
//		dev_err(&spi->dev, "%s: regmap allocation failed: %ld\n",
//			__func__, PTR_ERR(regmap));
//		return PTR_ERR(regmap);
//	}
//
//	return pcf2131_probe(&spi->dev, regmap, pcf2131_spi_driver.driver.name,
//			     spi_get_device_id(spi)->driver_data);
//}
//
//static const struct spi_device_id pcf2131_spi_id[] = {
//	{ "pcf2131", 1 },
//	{ }
//};
//MODULE_DEVICE_TABLE(spi, pcf2131_spi_id);
//
//static struct spi_driver pcf2131_spi_driver = {
//	.driver		= {
//		.name	= "rtc-pcf2131-spi",
//		.of_match_table = of_match_ptr(pcf2131_of_match),
//	},
//	.probe		= pcf2131_spi_probe,
//	.id_table	= pcf2131_spi_id,
//};
//
//static int pcf2131_spi_register_driver(void)
//{
//	return spi_register_driver(&pcf2131_spi_driver);
//}
//
//static void pcf2131_spi_unregister_driver(void)
//{
//	spi_unregister_driver(&pcf2131_spi_driver);
//}
//
//#else
//
//static int pcf2131_spi_register_driver(void)
//{
//	return 0;
//}
//
//static void pcf2131_spi_unregister_driver(void)
//{
//}
//
//#endif
