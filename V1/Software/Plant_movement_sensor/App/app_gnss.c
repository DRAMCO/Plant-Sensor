/*
 * app_gnss.c
 *
 *  Created on: Mar 11, 2023
 *      Author: Sarah Goossens
 */


#include "main.h"
#if SENSOR_HOST

#include "app_gnss.h"
//#if !STM32WBAUSED
#include "app_rtc.h"
//#endif
#include "usart.h"              // to declare huart2
#include "i2c.h"                // to declare hi2c1
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "time.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#define PRINTF_APP_GNSS 0

#define NMEA_START_DELIMITER    0x24 // "$"
#define NMEA_ELEMENT_DELIMITER  0x2C // ","
#define NMEA_CHECKSUM_DELIMITER 0x2A // "*"
#define NMEA_END_DELIMITER      0x0D // "/r" , in fact at the end it is /r/n  0x 0D 0A 24 (last byte is start of next NMEA message)

extern char              uart_buf[200];

extern time_t            gnssEpoch;
extern SemaphoreHandle_t gnssEpochMutex;
extern uint64_t          rtosTimeStampGPRMC;
extern uint32_t          gnss_pps;
extern uint64_t          rtosTimeStampGNSSPPS;
extern uint32_t          gnssEpochAvailable;
#if !STM32WBAUSED
extern uint8_t           setClock;
extern uint8_t           i2cInUseGNSS;
extern SemaphoreHandle_t i2cGNSSMutex;
extern uint64_t          rtosTimeStampI2CInUse;
extern uint64_t          rtosTimeStampI2CFree;
extern uint8_t           notifyOnlyOnceGNSS;
extern uint64_t          rtosTimeStampBCastStrt;
extern uint8_t           rtcSynchronized;
#endif

#if STM32WBAUSED
  TaskHandle_t gnssThreadHandle;
#else
  osThreadId gnssThreadHandle;
#endif

/* Private variables ---------------------------------------------------------*/
static uint8_t nmeaMatrix[48][MAX_MSG_LEN];
uint8_t gnssBuf[16];
char nmeaBuf[150];

void GNSSThreadInit()
{
#if STM32WBAUSED
  if (xTaskCreate ((TaskFunction_t)StartGNSSThread, "GNSSThread", (configSTACK_DEPTH_TYPE)512, NULL, osPriorityHigh7, &gnssThreadHandle) != pdPASS)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [app_gnss] [GNSSThread] Could not be started.\n",(unsigned int) xTaskGetTickCount());
    huart2print(uart_buf, strlen(uart_buf));
  }
#else
  osThreadDef(GNSSThread, StartGNSSThread, osPriorityNormal, 0, 128);
  gnssThreadHandle = osThreadCreate(osThread(GNSSThread), NULL);
#endif
}

void StartGNSSThread(const void * params)
{
//  TickType_t xLastWakeTime = xTaskGetTickCount();
//  vTaskDelayUntil(&xLastWakeTime, 2000U);
  vTaskDelay(2000U);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_gnss] [GNSSThread] Started.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));

//  HAL_GPIO_WritePin(GNSS_WakeUp_GPIO_Port, GNSS_WakeUp_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GNSS_Reset_GPIO_Port, GNSS_Reset_Pin, GPIO_PIN_SET);
  vTaskDelay(200);
  HAL_GPIO_WritePin(GNSS_Reset_GPIO_Port, GNSS_Reset_Pin, GPIO_PIN_RESET);
  HAL_Delay(150);
//  HAL_GPIO_WritePin(GNSS_WakeUp_GPIO_Port, GNSS_WakeUp_Pin, GPIO_PIN_RESET);

#if PRINTF_APP_GNSS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_gnss] [GNSSThread] GNSS module reset done.\r\n",(unsigned int) xTaskGetTickCount());
  huart2print(uart_buf, strlen(uart_buf));
#endif
  gnssEpoch = 0;
//  uint64_t gnss_own_epoch = 0;
  int i = 0; // NMEA message character position
  int j = 0; // field number inside app matrix
  int k = 0; // character position inside app field
  int l = 0; // character position inside nmeaBuf
  int startDelimiterFound = 0;
  static GPRMC_Info_t gprmc_data;     /**< $GPRMC Data holder */
  static GPRMC_Info_t gprmc_data_old; /**< $GPRMC Data holder */
  gprmc_data_old.status = 0;
  gprmc_data_old.speed  = 0;
  gprmc_data_old.date   = 0;
  static GPGGA_Info_t gpgga_data;     /**< $GPGGA Data holder */
  static GPGGA_Info_t gpgga_data_old; /**< $GPGGA Data holder */
  gpgga_data_old.xyz.lat = 0;
  gpgga_data_old.xyz.lon = 0;
  gpgga_data_old.xyz.alt = 0;
  gpgga_data_old.acc     = 0;
  gpgga_data_old.valid   = 0;
  gpgga_data_old.sats    = 0;

  uint32_t nmeaCheckSum     = 0;
  uint32_t nmeaCheckSumCalc = 0;

  struct tm gprmc_time;

  // Configure the message list:
  /* See CDB-ID 201 - This LOW_BITS Mask enables the following messages:
   * 0x1 $GPGNS Message
   * 0x2 $GPGGA Message
   * 0x4 $GPGSA Message
   * 0x8 $GPGST Message
   * 0x40 $GPRMC Message
   * 0x80000 $GPGSV Message
   * 0x100000 $GPGLL Message
   */
  int lowMask = 0x40;
  npf_snprintf(nmeaBuf, 150, "$PSTMCFGMSGL,0,1,%d,0*7E\r\n",lowMask);
//  npf_snprintf(nmeaBuf, 150, "$PSTMCFGMSGL,0,1,%d,0",lowMask);
//  nmeaCheckSumCalc = 0;
//  for (uint32_t m = 1U; m < strlen(nmeaBuf); m++)
//  {
//    nmeaCheckSumCalc = (nmeaCheckSumCalc ^ nmeaBuf[m]);
//  }
//  npf_snprintf(nmeaBuf, 150, "$PSTMCFGMSGL,0,1,%d,0*%X%X\r\n",lowMask,(uint8_t)((nmeaCheckSumCalc>>8) & 0xF),(uint8_t)(nmeaCheckSumCalc & 0xF));
  IsI2CAvailable();
#if STM32WBAUSED
  HAL_I2C_Master_Transmit(&hi2c3, GNSS_ADDRESS, nmeaBuf, strlen(nmeaBuf), 500);
#else
  HAL_I2C_Master_Transmit(&hi2c1, GNSS_ADDRESS, nmeaBuf, strlen(nmeaBuf), 500);
#endif
  IsI2CAvailable();
//  {
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_gnss] Waiting for I2C to become available.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
//  }
#if PRINTF_APP_GNSS
  waitToPrint();
  npf_snprintf(uart_buf, 200, "%u [app_gnss] [GNSSThread] Process NMEA Command: %s.\r\n",(unsigned int) xTaskGetTickCount(), nmeaBuf);
  huart2print(uart_buf, strlen(uart_buf));
#endif
//  // save parameters
//  npf_snprintf(nmeaBuf, 150, "$PSTMSAVEPAR*58\r\n");
////  npf_snprintf(nmeaBuf, 150, "$PSTMSAVEPAR");
////  nmeaCheckSumCalc = 0;
////  for (uint32_t m = 1U; m < strlen(nmeaBuf); m++)
////  {
////    nmeaCheckSumCalc = (nmeaCheckSumCalc ^ nmeaBuf[m]);
////  }
////  npf_snprintf(nmeaBuf, 150, "$PSTMSAVEPAR*%1X%1X\r\n",(unsigned int)((nmeaCheckSumCalc>>8) & 0x0F),(unsigned int)(nmeaCheckSumCalc & 0x0F));
////  waitToPrint();
////  npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] Process NMEA Command: %s.\r\n",(unsigned int) xTaskGetTickCount(), nmeaBuf);
////  huart2print(uart_buf, strlen(uart_buf));
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//  {
////#if PRINTF_APP_GNSS
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_gnss] Waiting for I2C to become available.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
////#endif
//  }
//  HAL_I2C_Master_Transmit(&hi2c1, GNSS_ADDRESS, nmeaBuf, strlen(nmeaBuf), 500);
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//  {
////#if PRINTF_APP_GNSS
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_gnss] Waiting for I2C to become available.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
////#endif
//  }
//
//  // System Reset
//  npf_snprintf(nmeaBuf, 150, "$PSTMSRR*49\r\n");
////  npf_snprintf(nmeaBuf, 150, "$PSTMSRR");
////  nmeaCheckSumCalc = 0;
////  for (uint32_t m = 1U; m < strlen(nmeaBuf); m++)
////  {
////    nmeaCheckSumCalc = (nmeaCheckSumCalc ^ nmeaBuf[m]);
////  }
////  npf_snprintf(nmeaBuf, 150, "$PSTMSRR*%1X%1X\r\n",(unsigned int)((nmeaCheckSumCalc>>8) & 0x0F),(unsigned int)(nmeaCheckSumCalc & 0x0F));
////  waitToPrint();
////  npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] Process NMEA Command: %s.\r\n",(unsigned int) xTaskGetTickCount(), nmeaBuf);
////  huart2print(uart_buf, strlen(uart_buf));
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//  {
////#if PRINTF_APP_GNSS
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_gnss] Waiting for I2C to become available.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
////#endif
//  }
//  HAL_I2C_Master_Transmit(&hi2c1, GNSS_ADDRESS, nmeaBuf, strlen(nmeaBuf), 500);
//  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//  {
////#if PRINTF_APP_GNSS
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "%u [app_gnss] Waiting for I2C to become available.\r\n",(unsigned int) xTaskGetTickCount());
//    huart2print(uart_buf, strlen(uart_buf));
////#endif
//  }

#if !STM32WBAUSED
  uint8_t  gnssOngoing          = 0;
  uint8_t  i2cInUseGNSSCopy     = 0;
  int32_t  timeForNextBroadcast = 0;
  uint64_t timeSBeforeStrtGNSS  = 0;
#endif
  time_t   gnssEpochTemp        = 0;
  /* Infinite loop */
  for(;;)
  {
#if !STM32WBAUSED
    if (setClock && notifyOnlyOnceGNSS)
    {// notify app_rtc that the i2c bus is not in use by the GNSS module
      app_rtc_notify(NOTIFICATION_FROM_APP_GNSS);
      // notifyOnlyOnceGNSS = 0;
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_gnss] Notification given that clock can be set.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
    }
    timeSBeforeStrtGNSS = xTaskGetTickCount();
    timeForNextBroadcast = 10000 - (timeSBeforeStrtGNSS - rtosTimeStampBCastStrt);
    if (rtcSynchronized && (timeForNextBroadcast < 100) && !setClock)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [app_gnss] A next broadcast will start within %dms, do not use i2c for the next 110ms. rtosTimeStampBCastStrt = %u.\r\n",
          (unsigned int) timeSBeforeStrtGNSS, (int) (timeForNextBroadcast), (uint) rtosTimeStampBCastStrt);
      huart2print(uart_buf, strlen(uart_buf));
      vTaskDelay(110);
    }
    xSemaphoreTake(i2cGNSSMutex, pdMS_TO_TICKS(400));
    i2cInUseGNSSCopy = i2cInUseGNSS;
    xSemaphoreGive(i2cGNSSMutex);
    if (!setClock || i2cInUseGNSSCopy)
    { // to prevent that the i2c bus is in use while the RTC is being changed
      xSemaphoreTake(i2cGNSSMutex, pdMS_TO_TICKS(400));
      i2cInUseGNSS = 1;
      xSemaphoreGive(i2cGNSSMutex);
      notifyOnlyOnceGNSS = 1;
      rtosTimeStampI2CInUse = xTaskGetTickCount();
//#if PRINTF_APP_GNSS
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[app_gnss]     i2c in use by GNSS at %ums -> %ums later then GNSSPPS #%u.\r\n",
//          (unsigned int) rtosTimeStampI2CInUse, (unsigned int) (rtosTimeStampI2CInUse - rtosTimeStampGNSSPPS), (unsigned int) gnss_pps);
//      huart2print(uart_buf, strlen(uart_buf));
//#endif
#endif

      IsI2CAvailable();
#if STM32WBAUSED
      HAL_I2C_Master_Receive(&hi2c3, GNSS_ADDRESS, gnssBuf, 16, 500);
#else
      HAL_I2C_Master_Receive(&hi2c1, GNSS_ADDRESS, gnssBuf, 16, 500);
#endif
      IsI2CAvailable();
//      {
//        waitToPrint();
//        npf_snprintf(uart_buf, 200, "%u [app_gnss] Waiting for I2C to become available.\r\n",(unsigned int) xTaskGetTickCount());
//        huart2print(uart_buf, strlen(uart_buf));
//      }
#if !STM32WBAUSED
      xSemaphoreTake(i2cGNSSMutex, pdMS_TO_TICKS(400));
      i2cInUseGNSS = 0;
      xSemaphoreGive(i2cGNSSMutex);
      rtosTimeStampI2CFree = xTaskGetTickCount();
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "[app_gnss]   i2c released by GNSS at %ums -> i2c used during %ums by GNSS module.\r\n",
//          (unsigned int) rtosTimeStampI2CFree, (unsigned int) (rtosTimeStampI2CFree - rtosTimeStampI2CInUse));
//      huart2print(uart_buf, strlen(uart_buf));
#endif
//#if PRINTF_APP_GNSS
//    waitToPrint();
////    npf_snprintf(uart_buf, 150, "                                                                                                                                                      ");
//    npf_snprintf(uart_buf, 150, "%u [app_gnss] [rtos_gnss_thread] GNSS RAW data: 0x%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X.\r\n",(unsigned int) xTaskGetTickCount(),
//		  gnssBuf[0],gnssBuf[1],gnssBuf[2],gnssBuf[3],gnssBuf[4],gnssBuf[5],gnssBuf[6],gnssBuf[7],gnssBuf[8],
//		  gnssBuf[9],gnssBuf[10],gnssBuf[11],gnssBuf[12],gnssBuf[13],gnssBuf[14],gnssBuf[15]);
//    huart2print(uart_buf, strlen(uart_buf));
//#endif

      // Only check for $GPGGA messages... delete the rest
      for (i = 0; i < 16; i++)
      {
        if (!startDelimiterFound && (gnssBuf[i] == NMEA_START_DELIMITER))
        {
    	  startDelimiterFound = 1;
    	  nmeaBuf[l++] = gnssBuf[i];
    	  nmeaMatrix[j][k++] = gnssBuf[i];
        }
        else
        {
          if (gnssBuf[i] == NMEA_END_DELIMITER)
          {
            nmeaBuf[l] = '\0';
            nmeaMatrix[j][k] = (uint8_t)'\0';
            // check if valid message:
            nmeaCheckSum = (char2int(nmeaBuf[l-2U]) << 4) | char2int(nmeaBuf[l-1U]);
            //nmeaCheckSum = nmea_checksum(nmeaMatrix[j]);
            nmeaCheckSumCalc = 0;
            for (uint32_t m = 1U; m < (l-3U); m++)
            {
              nmeaCheckSumCalc = (nmeaCheckSumCalc ^ nmeaBuf[m]);
            }
            startDelimiterFound = 0;
            j = 0;
            k = 0;
            l = 0;
            if (nmeaCheckSum != nmeaCheckSumCalc)
            {
//#if PRINTF_APP_GNSS
              waitToPrint();
              npf_snprintf(uart_buf, 200, "%u [app_gnss] [GNSSThread] Bad checksum NMEA Message: %s. Received checksum: %u, calculated checksum: %u.\r\n",(unsigned int) xTaskGetTickCount(), nmeaBuf, (unsigned int)nmeaCheckSum, (unsigned int)nmeaCheckSumCalc);
              huart2print(uart_buf, strlen(uart_buf));
//#endif
            }
            else
            {
              if (strcmp((char *)nmeaMatrix[0], "$GPRMC") == 0)
              {
//              RMC - Recommended Minimum Navigation Information
//              This is one of the sentences commonly emitted by GPS units.
//                     1         2 3       4 5        6  7   8   9    10 11
//                     |         | |       | |        |  |   |   |    |  |
//              $--RMC,hhmmss.ss,A,ddmm.mm,a,dddmm.mm,a,x.x,x.x,xxxx,x.x,a*hh<CR><LF>
//              NMEA 2.3:
//              $--RMC,hhmmss.ss,A,ddmm.mm,a,dddmm.mm,a,x.x,x.x,xxxx,x.x,a,m*hh<CR><LF>
//              NMEA 4.1:
//              $--RMC,hhmmss.ss,A,ddmm.mm,a,dddmm.mm,a,x.x,x.x,xxxx,x.x,a,m,s*hh<CR><LF>
//              Field Number:
//               1 - UTC of position fix, hh is hours, mm is minutes, ss.ss is seconds.
//               2 - Status, A = Valid, V = Warning
//               3 - Latitude, dd is degrees. mm.mm is minutes.
//               4 - N or S
//               5 - Longitude, ddd is degrees. mm.mm is minutes.
//               6 - E or W
//               7 - Speed over ground, knots
//               8 - Track made good, degrees true
//               9 - Date, ddmmyy
//              10 - Magnetic Variation, degrees
//              11 - E or W
//              12 - FAA mode indicator (NMEA 2.3 and later)
//              13 - Nav Status (NMEA 4.1 and later) A=autonomous, D=differential, E=Estimated, M=Manual input mode N=not valid, S=Simulator, V = Valid
//              14 - Checksum
//              A status of V means the GPS has a valid fix that is below an internal quality threshold, e.g. because the dilution of precision is too high or an elevation mask test failed.
//              The number of digits past the decimal point for Time, Latitude and Longitude is model dependent.
//              Example: $GNRMC,001031.00,A,4404.13993,N,12118.86023,W,0.146,,100117,,,A*7B
//              Example of this thread:
//                                                                     1          2 3          4 5           6  7   8  9    10 12  13
//                                                                     |          | |          | |           |  |   |  |     || |  |
//              000 [app_gnss] [rtos_gnss_thread] NMEA Message: $GPRMC,154440.000,A,5058.18969,N,00404.12957,E,0.0,0.0,290323,,,D*6D.
//              002 [app_gnss] [rtos_gnss_thread] $GPRMC - checksum: 109 (0x6D).
//            * UTC:			[ 15:44:40 ]
//            * Status:			[ A ]		-- Valid (reported in 2D and 3D fix conditions)
//              Latitude:			[ 50' 58'' N ]
//              Longitude:			[ 4' 04'' E ]
//              Speed over ground (knots):	[ 0.0 ]
//              Trackgood:			[ 0.0 ]
//            * Date (ddmmyy):		[ 290323 ]
//              Magnetic Variation:		[ 0.0 ]
//              Magnetic Var. Direction:	[ - ]
//              350
                rtosTimeStampGPRMC = xTaskGetTickCount();
#if PRINTF_APP_GNSS
                waitToPrint();
                npf_snprintf(uart_buf, 200, "[app_gnss]                 $GPRMC at %ums -> %ums later then GNSSPPS #%u.\r\n",
                    (unsigned int) rtosTimeStampGPRMC, (unsigned int) (rtosTimeStampGPRMC - rtosTimeStampGNSSPPS), (unsigned int) gnss_pps);
                huart2print(uart_buf, strlen(uart_buf));
#endif
                // GPRMC_Info_t gprmc_data; /**< $GPRMC Data holder */
                scan_utc(nmeaMatrix[1],  &gprmc_data.utc);
                gprmc_data.status      = *((uint8_t*)   nmeaMatrix[ 2]);
                gprmc_data.xyz.lat     = strtod((char *)nmeaMatrix[ 3], NULL);
                gprmc_data.xyz.ns      = *((uint8_t*)   nmeaMatrix[ 4]);
                gprmc_data.xyz.lon     = strtod((char *)nmeaMatrix[ 5], NULL);
                gprmc_data.xyz.ew      = *((uint8_t*)   nmeaMatrix[ 6]);
                gprmc_data.speed       = strtof((char *)nmeaMatrix[ 7], NULL);
                gprmc_data.trackgood   = strtof((char *)nmeaMatrix[ 8], NULL);
                gprmc_data.date        = strtol((char *)nmeaMatrix[ 9], NULL, 10);
                gprmc_data.mag_var     = strtof((char *)nmeaMatrix[10], NULL);
                gprmc_data.mag_var_dir = *((uint8_t*)   nmeaMatrix[11]);
                /* WARNING: from received msg, it seems there is another data (nmeaMatrix[12]) before the checksum */
                //gprmc_data.checksum    = nmea_checksum( nmeaMatrix[13]);
//#if PRINTF_APP_GNSS
//                waitToPrint();
//                npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] $GPRMC - checksum: %u (0x%02X).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) gprmc_data.checksum, (unsigned int) gprmc_data.checksum);
//                huart2print(uart_buf, strlen(uart_buf));
//#endif
//              GNSS_DATA_GetGPRMCInfo(&gprmc_data);
                // Calculate Epoch:
                gprmc_time.tm_year  = (uint16_t)(gprmc_data.date      - (100*(gprmc_data.date/100))  + 100);  // Year - 1900
                gprmc_time.tm_mon   = (uint8_t)((gprmc_data.date/100) - (100*(gprmc_data.date/10000))-   1);  // Month, where 0 = jan
                gprmc_time.tm_mday  = (uint8_t)((gprmc_data.date/10000));                                     // Day of the month
                gprmc_time.tm_hour  = (uint8_t)gprmc_data.utc.hh;                                             // 24 hours format
                gprmc_time.tm_min   = (uint8_t)gprmc_data.utc.mm;
                gprmc_time.tm_sec   = (uint8_t)gprmc_data.utc.ss;
                gprmc_time.tm_isdst = 0;                                                                      // Is DST on? 1 = yes, 0 = no, -1 = unknown
                // calculate epoch, mktime will also calculate tm_wday and tm_yday.
                // epoch = #s elapsed since 1/1/1970
                // mktime consumes 5.46KB of FASH memory!!
                gnssEpochTemp = mktime(&gprmc_time);
                xSemaphoreTake(gnssEpochMutex, pdMS_TO_TICKS(400));
                if (gnssEpochTemp > gnssEpoch)
                {
                  gnssEpoch = gnssEpochTemp;
                  gnssEpochAvailable = 1;
                }
                xSemaphoreGive(gnssEpochMutex);

//              // an alternative to save FLASH:
//              // https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap04.html#tag_04_15:
//              gnss_own_epoch =   (uint64_t)gprmc_data.utc.ss                         // #s, no extra second to see the difference with gnss_epoch
//		               + (uint64_t)gprmc_data.utc.mm              * 60
//		  	       + (uint64_t)gprmc_data.utc.hh              * 3600
//		               + (uint64_t)gprmc_time.tm_yday             * 86400    // 3600 s/h * 24 h/day = 86,400
//		               + (uint64_t)(gprmc_time.tm_year-70)        * 31536000 // 3600 s/h * 24 h/day * 365 days/year = 31,536,000s
//			                                                             // -70 since tm_year is # years since 1900 while epoch is since 1970
//  		               + ((uint64_t)(gprmc_time.tm_year-69)/4)    * 86400    // Every 4 year one day extra
//  		               - ((uint64_t)(gprmc_time.tm_year-1)/100)   * 86400    // exception to this rule?
//    		               + ((uint64_t)(gprmc_time.tm_year+299)/400) * 86400;   // exception to this rule?

//              // how to calculate dayOfWeek - see: https://cs.uwaterloo.ca/~alopez-o/math-faq/node73.html
//              // W = (k + [2.6 * m - 0.2] - 2 * C + Y + [Y / 4] + [C / 4]) mod7
//              // where [] denotes the integer floor function,
//              // k is day (1 to 31)
//              // m is month (1 = March, ..., 10 = December, 11 = Jan, 12 = Feb) Treat Jan & Feb as months of the preceding year
//              // C is century (1987 has C = 19). In this program, Century = 20
//              // Y is year (1987 has Y = 87 except Y = 86 for Jan & Feb)
//              // W is week day (0 = Sunday, ..., 6 = Saturday)
//              if (gnss_time.month > 2)
//              {
//                tempMonth  = gnss_time.month - 2;
//                tempYear   = gnss_time.year;
//              }
//              else
//              {
//                tempMonth  = gnss_time.month + 10;
//                tempYear   = gnss_time.year - 1;
//              }
//              tempDOWInt   = gnss_time.dayOfMonth + (2.6 * tempMonth - 0.2) - 2 * tempYear + (tempYear / 4) + 5;
//              tempDOWFloat = tempDOWInt / 7;
//              tempDOWInt   = tempDOWInt / 7;
//              gnss_time.dayOfWeek  = (tempDOWFloat - tempDOWInt) * 7;

//              waitToPrint();
//              npf_snprintf(uart_buf, 200, "%u [app_gnss] GNSS Date: %d/%d/%d, Time: %d:%d:%d, Epoch: %ld, Difference Own Epoch: %d.\r\n", (unsigned int) xTaskGetTickCount(),
//  			 gprmc_time.tm_mday, gprmc_time.tm_mon + 1, gprmc_time.tm_year + 1900, gprmc_time.tm_hour, gprmc_time.tm_min, gprmc_time.tm_sec,
//  			 (long)gnss_epoch, (int)((uint64_t)gnss_epoch - gnss_own_epoch));
//              huart2print(uart_buf, strlen(uart_buf));


             // Only print * marked items:
             // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
             // strftime(uart_buf, 200, "%a %Y-%m-%d %H:%M:%S %Z", &gprmc_time);
             // waitToPrint();
             // npf_snprintf(uart_buf, 200, "%u [app_gnss] [GPRMC] GNSS UTC time - %s", (unsigned int) xTaskGetTickCount(), asctime(&gprmc_time));
             // huart2print(uart_buf, strlen(uart_buf));

//              // Local time:
//
//              struct tm * timeinfo;
//              putenv("TZ=Central Europe Time-2:00");
//              timeinfo = localtime (&gnss_epoch);
//              strftime(buft, sizeof(buft), "%a %Y-%m-%d %H:%M:%S %Z", &timeinfo);
//              waitToPrint();
//              npf_snprintf(uart_buf, 200, "%u [app_gnss] [GPRMC] GNSS Local time - %s.\r\n", (unsigned int) xTaskGetTickCount(), buft);
//              huart2print(uart_buf, strlen(uart_buf));
//                if (gprmc_data.status != gprmc_data_old.status)
//                {

//                  waitToPrint();
//                  npf_snprintf(uart_buf, 200, "\tDate (ddmmyy):\t\t\t[ %02d%02d%02d ]\n\r", (int16_t)((gprmc_data.date/10000)),
//                    (int16_t)((gprmc_data.date/100) - (100*(gprmc_data.date/10000))), (int16_t)(gprmc_data.date - (100*(gprmc_data.date/100))));
//                  huart2print(uart_buf, strlen(uart_buf));
//
//                  waitToPrint();
//                  npf_snprintf(uart_buf, 200,  "\tUTC:\t\t\t\t[ %02d:%02d:%02d ]\n\r", gprmc_data.utc.hh, gprmc_data.utc.mm, gprmc_data.utc.ss);
//                  huart2print(uart_buf, strlen(uart_buf));
#if PRINTF_APP_GNSS
                  waitToPrint();
                  npf_snprintf(uart_buf, 200, "\tStatus:\t\t\t\t[ %c ]\t\t", gprmc_data.status);
                  huart2print(uart_buf, strlen(uart_buf));
                  waitToPrint();
                  if (gprmc_data.status == (uint8_t)'A')
                  {
                    npf_snprintf(uart_buf, 200, "-- Valid (reported in 2D and 3D fix conditions)\n\r");
                  }
                  else if (gprmc_data.status == (uint8_t)'V')
                  {
                    npf_snprintf(uart_buf, 200, "-- Warning (reported in NO FIX conditions)\n\r");
                  }
                  else
                  {
                    npf_snprintf(uart_buf, 200, "-- Unknown status\n\r");
                  }
                  huart2print(uart_buf, strlen(uart_buf));
#endif
//                }
                gprmc_data_old.status = gprmc_data.status;
              }
              else
              {
                if (strcmp((char *)nmeaMatrix[0], "$GPGGA") == 0)
                {
//                https://gpsd.gitlab.io/gpsd/NMEA.html#_gll_geographic_position_latitudelongitude
//                GGA - Global Positioning System Fix Data
//                This is one of the sentences commonly emitted by GPS units.
//                Time, Position and fix related data for a GPS receiver.
//                                                                    11
//                       1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
//                       |         |       | |        | | |  |   |   | |   | |   |    |
//                $--GGA,hhmmss.ss,ddmm.mm,a,ddmm.mm,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
//                Field Number:
//                 1 - UTC of this position report, hh is hours, mm is minutes, ss.ss is seconds.
//                 2 - Latitude, dd is degrees, mm.mm is minutes
//                 3 - N or S (North or South)
//                 4 - Longitude, dd is degrees, mm.mm is minutes
//                 5 - E or W (East or West)
//                 6 - GPS Quality Indicator (non null)
//                    0 - fix not available,
//                    1 - GPS fix,
//                    2 - Differential GPS fix (values above 2 are 2.3 features)
//                    3 = PPS fix
//                    4 = Real Time Kinematic
//                    5 = Float RTK
//                    6 = estimated (dead reckoning)
//                    7 = Manual input mode
//                    8 = Simulation mode
//                 7 - Number of satellites in use, 00 - 12
//                 8 - Horizontal Dilution of precision (meters)
//                 9 - Antenna Altitude above/below mean-sea-level (geoid) (in meters)
//                10 - Units of antenna altitude, meters
//                11 - Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
//                12 - Units of geoidal separation, meters
//                13 - Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
//                14 - Differential reference station ID, 0000-1023
//                15 - Checksum
//                The number of digits past the decimal point for Time, Latitude and Longitude is model dependent.
//                Example:
//                $GNGGA,001043.00,4404.14036,N,12118.85961,W,1,12,0.98,1113.0,M,-21.3,M*47
//                Note: Jackson Labs replaces the Quality indicator with GPSDO status.
//                Example of this thread:
//                                                                                                                                  11  13 15
//                                                                         1          2          3 4           5 6 7   8     9  10   | 12|14|
//                                                                         |          |          | |           | | |   |     |   |   |  ||| |
//                10586 [app_gnss] [rtos_gnss_thread] NMEA Message: $GPGGA,182417.000,5058.19135,N,00404.13287,E,2,10,0.8,011.48,M,47.3,M,,*6B.
//                10588 [app_gnss] [rtos_gnss_thread] $GPGGA - checksum: 107 (0x6B).
//                UTC:			[ 18:24:17 ]
//              * Latitude full:		[ 50° 58' 11.4809'' N ]
//              * Longitude full:		[ 4° 4' 7.9721'' E ]
//              * Satellites locked:	[ 10 ]
//              * GPS Quality indicator:	[ 2 ] Differential GPS fix.
//              * Position accuracy:	[ 0.8 ]
//              * Altitude:			[ 11.47m ]
//                Geoid infos:		[ 47M ]
//                Diff update:		[ 0 ]
//                10604
//                  GPGGA_Info_t gpgga_data; /**< $GPGGA Data holder */
//                  int32_t valid = strtol((char *)nmeaMatrix[6], NULL, 10);
//                  if((valid == 1) || (valid == 0))
//                  {
//                    gpgga_data.valid = (uint8_t)valid;
//                  }
                  scan_utc(nmeaMatrix[1], &gpgga_data.utc); // -> already received from gprmc message
                  gpgga_data.xyz.lat      = strtod((char *)nmeaMatrix[2], NULL);
                  gpgga_data.xyz.ns       = *((uint8_t*)   nmeaMatrix[3]);
                  gpgga_data.xyz.lon      = strtod((char *)nmeaMatrix[4], NULL);
                  gpgga_data.xyz.ew       = *((uint8_t*)   nmeaMatrix[5]);
                  gpgga_data.valid        = strtol((char *)nmeaMatrix[6], NULL, 10);
                  gpgga_data.sats         = strtol((char *)nmeaMatrix[7], NULL, 10);
                  gpgga_data.acc          = strtof((char *)nmeaMatrix[8], NULL);
                  gpgga_data.xyz.alt      = strtof((char *)nmeaMatrix[9], NULL);
                  gpgga_data.xyz.mis      = *((uint8_t*)   nmeaMatrix[10]);
                  gpgga_data.geoid.height = strtol((char *)nmeaMatrix[11], NULL, 10);
                  gpgga_data.geoid.mis    = *((uint8_t*)   nmeaMatrix[12]);
                  // This field is reserved
                  //gpgga_data->update = strtol((char *)nmeaMatrix[13], NULL, 10);
                  //gpgga_data.checksum     = nmea_checksum(nmeaMatrix[15]);
//#if PRINTF_APP_GNSS
//                waitToPrint();
//                npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] $GPGGA - checksum: %u (0x%02X).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) gpgga_data.checksum, (unsigned int) gpgga_data.checksum);
//                huart2print(uart_buf, strlen(uart_buf));
//#endif
//                GNSS_DATA_GetGPGGAInfo(&gpgga_data);
//                Only print marked items:
                  double lat_mod = fmod(gpgga_data.xyz.lat, 100.0);
                  double lon_mod = fmod(gpgga_data.xyz.lon, 100.0);
#if PRINTF_APP_GNSS
                  if (gpgga_data.xyz.lat != gpgga_data_old.xyz.lat)
                  {
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "\tLatitude:\t\t\t[ %.0f° %d' %.4f'' %c ]\n\r", (gpgga_data.xyz.lat - lat_mod) / 100.0, (int16_t)lat_mod, (gpgga_data.xyz.lat - (int16_t)gpgga_data.xyz.lat)*60, gpgga_data.xyz.ns);
                    huart2print(uart_buf, strlen(uart_buf));
                  }
                  if (gpgga_data.xyz.lon != gpgga_data_old.xyz.lon)
                  {
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "\tLongitude:\t\t\t[ %.0f° %d' %.4f'' %c ]\n\r", (gpgga_data.xyz.lon - lon_mod) / 100.0, (int16_t)lon_mod, (gpgga_data.xyz.lon - (int16_t)gpgga_data.xyz.lon)*60, gpgga_data.xyz.ew);
                    huart2print(uart_buf, strlen(uart_buf));
                  }
                  if (gpgga_data.xyz.alt != gpgga_data_old.xyz.alt)
                  {
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "\tAltitude:\t\t\t[ %.2f%c ]\n\r", gpgga_data.xyz.alt, (gpgga_data.xyz.mis + 32U));
                    huart2print(uart_buf, strlen(uart_buf));
                  }
                  if (gpgga_data.acc != gpgga_data_old.acc)
                  {
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "\tPosition accuracy:\t\t[ %.1f ]\n\r", gpgga_data.acc);
                    huart2print(uart_buf, strlen(uart_buf));
                  }
                  if (gpgga_data.valid != gpgga_data_old.valid)
                  {
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "\tGPS Quality indicator:\t\t[ %d ] ", gpgga_data.valid);
                    huart2print(uart_buf, strlen(uart_buf));
                    waitToPrint();
                    switch (gpgga_data.valid)
                    {
                      case 0U:
                        npf_snprintf(uart_buf, 200, "fix not available.\r\n");
                        break;
                      case 1U:
                        npf_snprintf(uart_buf, 200, "GPS fix.\r\n");
                        break;
                      case 2U:
                        npf_snprintf(uart_buf, 200, "Differential GPS fix.\r\n");
                        break;
                      case 3U:
                        npf_snprintf(uart_buf, 200, "PPS fix.\r\n");
                        break;
                      case 4U:
                        npf_snprintf(uart_buf, 200, "Real Time Kinematic.\r\n");
                        break;
                      case 5U:
                        npf_snprintf(uart_buf, 200, "Float RTK.\r\n");
                        break;
                      case 6U:
                        npf_snprintf(uart_buf, 200, "estimated (dead reckoning).\r\n");
                        break;
                      case 7U:
                        npf_snprintf(uart_buf, 200, "Manual input mode.\r\n");
                        break;
                      case 8U:
                        npf_snprintf(uart_buf, 200, "Simulation mode.\r\n");
                    }
                    huart2print(uart_buf, strlen(uart_buf));
                  }
                  if (gpgga_data.sats != gpgga_data_old.sats)
                  {
                    waitToPrint();
                    npf_snprintf(uart_buf, 200, "\tSatellites locked:\t\t[ %d ]\n\r", gpgga_data.sats);
                    huart2print(uart_buf, strlen(uart_buf));
                  }
#endif
                  gpgga_data_old.xyz   = gpgga_data.xyz;
                  gpgga_data_old.acc   = gpgga_data.acc;
                  gpgga_data_old.valid = gpgga_data.valid;
                  gpgga_data_old.sats  = gpgga_data.sats;
                }
                else
                {
//                if (strcmp((char *)nmeaMatrix[0], "$GPVTG") == 0)
//                {
////                https://gpsd.gitlab.io/gpsd/NMEA.html#_gll_geographic_position_latitudelongitude
////        	    VTG - Track made good and Ground speed
////        	    This is one of the sentences commonly emitted by GPS units.
////        	            1  2  3  4  5  6  7  8 9
////        	            |  |  |  |  |  |  |  | |
////        	    $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
////                NMEA 2.3:
////          	    $--VTG,x.x,T,x.x,M,x.x,N,x.x,K,m*hh<CR><LF>
////        	    Field Number:
////        	      Course over ground, degrees True
////		      T = True
////        	      Course over ground, degrees Magnetic
////        	      M = Magnetic
////        	      Speed over ground, knots
////        	      N = Knots
////        	      Speed over ground, km/hr
////        	      K = Kilometers Per Hour
////        	      FAA mode indicator (NMEA 2.3 and later)
////        	      Checksum
////        	    Note: in some older versions of NMEA 0183, the sentence looks like this:
////        	            1  2  3   4  5
////        	            |  |  |   |  |
////        	    $--VTG,x.x,x,x.x,x.x*hh<CR><LF>
////        	    Field Number:
////        	      True course over ground (degrees) 000 to 359
////        	      Magnetic course over ground 000 to 359
////        	      Speed over ground (knots) 00.0 to 99.9
////        	      Speed over ground (kilometers) 00.0 to 99.9
////        	      Checksum
////        	    The two forms can be distinguished by field 2, which will be the fixed text 'T' in the newer form. The new form appears to have been introduced with NMEA 3.01 in 2002.
////          	    Some devices, such as those described in [GLOBALSAT], leave the magnetic-bearing fields 3 and 4 empty.
////        	    Example: $GPVTG,220.86,T,,M,2.550,N,4.724,K,A*34
//                }
//                else
//       	  {
//                if (strcmp((char *)nmeaMatrix[0], "$GNGSA") == 0)
//                {
////                 https://gpsd.gitlab.io/gpsd/NMEA.html#_gll_geographic_position_latitudelongitude
////                 GSA - GPS DOP and active satellites
////                 This is one of the sentences commonly emitted by GPS units.
////                        1 2 3                        14 15  16  17  18
////                        | | |                         |  |   |   |   |
////                 $--GSA,a,a,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x.x,x.x,x.x*hh<CR><LF>
////                 Field Number:
////                  1 - Selection mode: M=Manual, forced to operate in 2D or 3D, A=Automatic, 2D/3D
////                  2 - Mode (1 = no fix, 2 = 2D fix, 3 = 3D fix)
////                  3 - ID of 1st satellite used for fix
////                  4 - ID of 2nd satellite used for fix
////                  5 - ID of 3rd satellite used for fix
////                  6 - ID of 4th satellite used for fix
////                  7 - ID of 5th satellite used for fix
////                  8 - ID of 6th satellite used for fix
////                  9 - ID of 7th satellite used for fix
////                 10 - ID of 8th satellite used for fix
////                 11 - ID of 9th satellite used for fix
////                 12 - ID of 10th satellite used for fix
////                 13 - ID of 11th satellite used for fix
////                 14 - ID of 12th satellite used for fix
////                 15 - PDOP
////                 16 - HDOP
////                 17 - VDOP
////                      System ID (NMEA 4.11), see above
////                 18 - xx. Checksum
////                 Example: $GNGSA,A,3,80,71,73,79,69,,,,,,,,1.83,1.09,1.47*17
////                 Note: NMEA 4.1+ systems (u-blox 9, Quectel LCD79) may emit an extra field, System ID, just before the checksum.
//                  GSA_Info_t   gngsa_data; /**< $GNGSA Data holder */
//                  (void)strncpy((char *)gngsa_data.constellation, (char *)nmeaMatrix[0], MAX_STR_LEN);
//                  gngsa_data.operating_mode = *((uint8_t*)nmeaMatrix[1]);
//                  gngsa_data.current_mode   = strtol((char *)nmeaMatrix[2], NULL, 10);
//                  int32_t *sat_prn         = gngsa_data.sat_prn;
//                  for (int8_t i = 0; i < MAX_SAT_NUM; i++)
//                  {
//                    *(&sat_prn[i])         = strtol((char *)nmeaMatrix[3+i], NULL, 10);
//                  }
//                  gngsa_data.pdop           = strtof((char *)nmeaMatrix[15], NULL);
//                  gngsa_data.hdop           = strtof((char *)nmeaMatrix[16], NULL);
//                  gngsa_data.vdop           = strtof((char *)nmeaMatrix[17], NULL);
//                  gngsa_data.checksum       = nmea_checksum(nmeaMatrix[18]);
//
////#if PRINTF_APP_GNSS
////                  waitToPrint();
////                  npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] $GNGSA - checksum: %u (0x%02X).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) gngsa_data.checksum, (unsigned int) gngsa_data.checksum);
////                  huart2print(uart_buf, strlen(uart_buf));
////#endif
////                  GNSS_DATA_GetGSAInfo(&gngsa_data);
//                  }
//                  else
//                  {
//                    if ((strcmp((char *)nmeaMatrix[0], "$GPGSV") == 0) || (strcmp((char *)nmeaMatrix[0], "$GLGSV") == 0))
//                    {
////                    https://gpsd.gitlab.io/gpsd/NMEA.html#_gll_geographic_position_latitudelongitude
////                    GSV - Satellites in view
////                    This is one of the sentences commonly emitted by GPS units.
////                    These sentences describe the sky position of a UPS satellite in view. Typically they’re shipped in a group of 2 or 3.
////                           1 2 3 4 5 6 7     n
////                           | | | | | | |     |
////                    $--GSV,x,x,x,x,x,x,x,...*hh<CR><LF>
////                    Field Number:
////                    1 - total number of GSV sentences to be transmitted in this group
////                    2 - Sentence number, 1-9 of this GSV message within current group
////                    3 - total number of satellites in view (leading zeros sent)
////                    4 - satellite ID or PRN number (leading zeros sent)
////                    5 - elevation in degrees (-90 to 90) (leading zeros sent)
////                    6 - azimuth in degrees to true north (000 to 359) (leading zeros sent)
////                    7 - SNR in dB (00-99) (leading zeros sent) more satellite info quadruples like 4-7 n-1) Signal ID (NMEA 4.11) n) checksum
////                    Example: $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
////                             $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
////                             $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
////                    Some GPS receivers may emit more than 12 quadruples (more than three GPGSV sentences), even though NMEA-0813 doesn’t allow this.
////                    (The extras might be WAAS satellites, for example.) Receivers may also report quads for satellites they aren’t tracking, in
////                    which case the SNR field will be null; we don’t know whether this is formally allowed or not.
////                    Example: $GLGSV,3,3,09,88,07,028*51
////                    Note: NMEA 4.10+ systems (u-blox 9, Quectel LCD79) may emit an extra field, Signal ID, just before the checksum. See the description of Signal ID’s above.
////                    Note: $GNGSV uses PRN in field 4. Other $GxGSV use the satellite ID in field 4. Jackson Labs, Quectel, Telit, and others get this wrong, in various conflicting ways.
////                    Example of this thread of a $GPGSV message:
////                           1 2 3 4 5 6 7     n
////                           | | | | | | |     |
////                    $--GSV,x,x,x,x,x,x,x,...*hh<CR><LF>
////                    Example of this thread of a GPGSV message:
////                                                                                    +----1----+  +----2----+  +----3----+  +----4----+
////                                                                             1 2 3  4  5  6   7  4  5  6   7  4  5  6   7  4  5  6   7
////                                                                             | | |  |  |  |   |  |  |  |   |  |  |  |   |  |  |  |   |
////                    10782 [app_gnss] [rtos_gnss_thread] NMEA Message: $GPGSV,2,1,08,01,77,312,,21,75,097,,03,43,233,38,32,34,058,*7A.
////                    10784 [app_gnss] [rtos_gnss_thread] $--GSV - checksum: 122 (0x7A).
////                    Constellation:		[ $GPGSV ]	-- message to report all GPS satellites
////                    GSV message:		[ 1 of 2 ]
////                    Num of Satellites:	[ 4 of 8 ]
////                    Sat01PRN:		[ 001 ]
////                    Sat01Elev (Â°):		[ 077 ]
////                    Sat01Azim (Â°):		[ 312 ]
////                    Sat01CN0 (dB):		[ 000 ]
////                    Sat02PRN:		[ 021 ]
////                    Sat02Elev (Â°):		[ 075 ]
////                    Sat02Azim (Â°):		[ 097 ]
////                    Sat02CN0 (dB):		[ 000 ]
////                    Sat03PRN:		[ 003 ]
////                    Sat03Elev (Â°):		[ 043 ]
////                    Sat03Azim (Â°):		[ 233 ]
////                    Sat03CN0 (dB):		[ 038 ]
////                    Sat04PRN:		[ 032 ]
////                    Sat04Elev (Â°):		[ 034 ]
////                    Sat04Azim (Â°):		[ 058 ]
////                    Sat04CN0 (dB):		[ 000 ]
////                                                                                  +----5----+  +----6----+  +----7----+  +----8----+
////                                                                           1 2 3  4  5  6   7  4  5  6   7  4  5  6   7  4  5  6   7
////                                                                           | | |  |  |  |   |  |  |  |   |  |  |  |   |  |  |  |   |
////                    10822 [app_gnss] [rtos_gnss_thread] NMEA Message: $GPGSV,2,2,08,08,29,167,38,17,26,313,,14,22,275,,10,05,056,*78.
////                    10824 [app_gnss] [rtos_gnss_thread] $GPGSV - checksum: 120 (0x78).
////                    Constellation:		[ $GPGSV ]	-- message to report all GPS satellites
////                    GSV message:		[ 2 of 2 ]
////                    Num of Satellites:	[ 4 of 8 ]
////                    Sat05PRN:		[ 008 ]
////                    Sat05Elev (Â°):		[ 029 ]
////                    Sat05Azim (Â°):		[ 167 ]
////                    Sat05CN0 (dB):		[ 038 ]
////                    Sat06PRN:		[ 017 ]
////                    Sat06Elev (Â°):		[ 026 ]
////                    Sat06Azim (Â°):		[ 313 ]
////                    Sat06CN0 (dB):		[ 000 ]
////                    Sat07PRN:		[ 014 ]
////                    Sat07Elev (Â°):		[ 022 ]
////                    Sat07Azim (Â°):		[ 275 ]
////                    Sat07CN0 (dB):		[ 000 ]
////                    Sat08PRN:		[ 010 ]
////                    Sat08Elev (Â°):		[ 005 ]
////                    Sat08Azim (Â°):		[ 056 ]
////                    Sat08CN0 (dB):		[ 000 ]
////                    10862
////                    Example of this thread of a GLGSV message:
////                                                                                    +----1---+ +----2----+  +----3---+ +----4---+
////                                                                             1 2 3  4  5  6  7 4  5  6   7  4  5  6  7 4  5  6  7
////                                                                             | | |  |  |  |  | |  |  |   |  |  |  |  | |  |  |  |
////                    10862 [app_gnss] [rtos_gnss_thread] NMEA Message: $GLGSV,2,1,08,87,77,356,,72,65,163,34,71,51,047,,86,35,117,*6A.
////                    10864 [app_gnss] [rtos_gnss_thread] $--GSV - checksum: 106 (0x6A).
////                    Constellation:		[ $GLGSV ]	-- message to report all GLONASS satellites
////                    GSV message:		[ 1 of 2 ]
////                    Num of Satellites:	[ 4 of 8 ]
////                    Sat01PRN:		[ 087 ]
////                    Sat01Elev (°):		[ 077 ]
////                    Sat01Azim (°):		[ 356 ]
////                    Sat01CN0 (dB):		[ 000 ]
////                    Sat02PRN:		[ 072 ]
////                    Sat02Elev (°):		[ 065 ]
////                    Sat02Azim (°):		[ 163 ]
////                    Sat02CN0 (dB):		[ 034 ]
////                    Sat03PRN:		[ 071 ]
////                    Sat03Elev (°):		[ 051 ]
////                    Sat03Azim (°):		[ 047 ]
////                    Sat03CN0 (dB):		[ 000 ]
////                    Sat04PRN:		[ 086 ]
////                    Sat04Elev (°):		[ 035 ]
////                    Sat04Azim (°):		[ 117 ]
////                    Sat04CN0 (dB):		[ 000 ]
////                    10902 [app_gnss] [rtos_gnss_thread] NMEA Message: $GLGSV,2,2,08,88,28,312,30,65,16,199,31,79,13,344,,78,11,294,*6C.
////                    10904 [app_gnss] [rtos_gnss_thread] $GPGSV - checksum: 108 (0x6C).
////                    Constellation:		[ $GLGSV ]	-- message to report all GLONASS satellites
////                    GSV message:		[ 2 of 2 ]
////                    Num of Satellites:	[ 4 of 8 ]
////                    Sat05PRN:		[ 088 ]
////                    Sat05Elev (°):		[ 028 ]
////                    Sat05Azim (°):		[ 312 ]
////                    Sat05CN0 (dB):		[ 030 ]
////                    Sat06PRN:		[ 065 ]
////                    Sat06Elev (°):		[ 016 ]
////                    Sat06Azim (°):		[ 199 ]
////                    Sat06CN0 (dB):		[ 031 ]
////                    Sat07PRN:		[ 079 ]
////                    Sat07Elev (°):		[ 013 ]
////                    Sat07Azim (°):		[ 344 ]
////                    Sat07CN0 (dB):		[ 000 ]
////                    Sat08PRN:		[ 078 ]
////                    Sat08Elev (°):		[ 011 ]
////                    Sat08Azim (°):		[ 294 ]
////                    Sat08CN0 (dB):		[ 000 ]
////                    10932
//                      GSV_Info_t   gpgsv_data; /**< $GPGSV Data holder */
//                      //NMEA_ResetGSVMsg(pGSVInfo);
//                      int8_t app_idx = 4;
//                      int8_t gsv_idx = 0;
//                      gpgsv_data.current_sats = 0;
//                      (void)strncpy((char *)gpgsv_data.constellation, (char *)nmeaMatrix[0], MAX_STR_LEN);
//                      gpgsv_data.amount   = strtol((char *)nmeaMatrix[1], NULL, 10);
//                      gpgsv_data.number   = strtol((char *)nmeaMatrix[2], NULL, 10);
//                      gpgsv_data.tot_sats = strtol((char *)nmeaMatrix[3], NULL, 10);
//                      for (int8_t i = 1; i <= GSV_MSG_SATS; i++)
//                      {
//                        gpgsv_data.gsv_sat_i[gsv_idx].prn  = strtol((char *)nmeaMatrix[app_idx*i], NULL, 10);
//                        gpgsv_data.gsv_sat_i[gsv_idx].elev = strtol((char *)nmeaMatrix[(app_idx*i)+1], NULL, 10);
//                        gpgsv_data.gsv_sat_i[gsv_idx].azim = strtol((char *)nmeaMatrix[(app_idx*i)+2], NULL, 10);
//                        gpgsv_data.gsv_sat_i[gsv_idx].cn0  = strtol((char *)nmeaMatrix[(app_idx*i)+3], NULL, 10);
//                        if(gpgsv_data.gsv_sat_i[gsv_idx].prn != 0)
//                        {
//                  	    gpgsv_data.current_sats++;
//                        }
//                        gsv_idx++;
//                      }
//                      gpgsv_data.checksum                  = nmea_checksum(nmeaMatrix[20]);
////#if PRINTF_APP_GNSS
////                    waitToPrint();
////                    npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] $--GSV - checksum: %u (0x%02X).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) gpgsv_data.checksum, (unsigned int) gpgsv_data.checksum);
////                    huart2print(uart_buf, strlen(uart_buf));
////#endif
////                    GNSS_DATA_GetGSVInfo(&gpgsv_data);
//                    }
//                    if (strcmp((char *)nmeaMatrix[0], "$GPGLL") == 0)
//                    {
////                    https://gpsd.gitlab.io/gpsd/NMEA.html#_gll_geographic_position_latitudelongitude
////                    GLL - Geographic Position - Latitude/Longitude
////                    This is one of the sentences commonly emitted by GPS units.
////                           1       2 3        4 5         6 7
////                           |       | |        | |         | |
////                    $--GLL,ddmm.mm,a,dddmm.mm,a,hhmmss.ss,a*hh<CR><LF>
////                    NMEA 2.3:
////                    $--GLL,ddmm.mm,a,dddmm.mm,a,hhmmss.ss,a,m*hh<CR><LF>
////                    Field Number:
////                    1 - Latitude, dd is degrees, mm.mm is minutes
////                    2 - N or S (North or South)
////                    3 - Longitude, dd is degrees, mm.mm is minutes
////                    4 - E or W (East or West)
////                    5 - UTC of this position, hh is hours, mm is minutes, ss.ss is seconds
////                    6 - Status A - Data Valid, V - Data Invalid
////                    7 - FAA mode indicator (NMEA 2.3 and later)
////                    7/8 - Checksum
////                    The number of digits past the decimal point for Time, Latitude and Longitude is model dependent.
////                    Example: $GNGLL,4404.14012,N,12118.85993,W,001037.00,A,A*67
//
//                    }
//                    if (strcmp((char *)nmeaMatrix[0], "$PSTMCPU") == 0)
//                    {
//
//
//                    } // endif (strcmp((char *)nmeaMatrix[0], "$PSTMCPU") == 0)
//                  }// endif (strcmp((char *)nmeaMatrix[0], "$GPGLL") == 0)
//                }// endif ((strcmp((char *)nmeaMatrix[0], "$GPGSV") == 0) || (strcmp((char *)nmeaMatrix[0], "$GLGSV") == 0))
              } // endif (strcmp((char *)nmeaMatrix[0], "$GNGSA") == 0)
            } // endif (strcmp((char *)nmeaMatrix[0], "$GPRMC") == 0)
          }
        } // endif (gnssBuf[i] == NMEA_END_DELIMITER)
        else
        {
          if ((gnssBuf[i] != 0x0A) && (gnssBuf[i] != 0xFF))
          { // make sure 0x0A (\n) is not being placed in nmeaBuf
            nmeaBuf[l++] = gnssBuf[i];
            if ((gnssBuf[i] == (uint8_t)',') || (gnssBuf[i] == (uint8_t)'*'))
            { // new field
              nmeaMatrix[j++][k] = (uint8_t)'\0';
              k = 0;
            }
            else
            { // no new field
              nmeaMatrix[j][k++] = gnssBuf[i];
            }
          }
        } // endif (gnssBuf[i] != NMEA_END_DELIMITER)
      } // endif (!startDelimiterFound && (gnssBuf[i] == NMEA_START_DELIMITER))
    } // end for (i = 0; i < 16; i++)
    if (gnssBuf[0] == 0xFF)
    {
      vTaskDelay(200U);
    }
#if !STM32WBAUSED
  } // endif !setClock
#endif
//    else
//    {
//  vTaskDelay(50U);
////#if PRINTF_APP_GNSS
//      waitToPrint();
//      npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] Clock being set, skip use of i2c for GNSS.\r\n",(unsigned int) xTaskGetTickCount());
//      huart2print(uart_buf, strlen(uart_buf));
////#endif
//    }
  }
}

ParseStatus_t NMEA_ParseGPRMC(GPRMC_Info_t *pGPRMCInfo, char NMEA[])
{ // taken out of NMEA_parser.c
//  scan_utc(nmeaMatrix[1],  &pGPRMCInfo->utc);
//  pGPRMCInfo->status      = *((uint8_t*)   nmeaMatrix[2]);
//  pGPRMCInfo->xyz.lat     = strtod((char *)nmeaMatrix[3], NULL);
//  pGPRMCInfo->xyz.ns      = *((uint8_t*)   nmeaMatrix[4]);
//  pGPRMCInfo->xyz.lon     = strtod((char *)nmeaMatrix[5], NULL);
//  pGPRMCInfo->xyz.ew      = *((uint8_t*)   nmeaMatrix[6]);
//  pGPRMCInfo->speed       = strtof((char *)nmeaMatrix[7], NULL);
//  pGPRMCInfo->trackgood   = strtof((char *)nmeaMatrix[8], NULL);
//  pGPRMCInfo->date        = strtol((char *)nmeaMatrix[9], NULL, 10);
//  pGPRMCInfo->mag_var     = strtof((char *)nmeaMatrix[10], NULL);
//  pGPRMCInfo->mag_var_dir = *((uint8_t*)   nmeaMatrix[11]);
//  /* WARNING: from received msg, it seems there is another data (nmeaMatrix[12]) before the checksum */
//  pGPRMCInfo->checksum    = nmea_checksum( nmeaMatrix[13]);
//
  return PARSE_SUCC;
}

/* Puts to console the info about Recommended Minimum Specific GPS/Transit data got by the most recent reception process. */
void GNSS_DATA_GetGPRMCInfo(GPRMC_Info_t *pGPRMCInfo)
{ // coming out of gnss_data.c
#if PRINTF_APP_GNSS
  waitToPrint();
// npf_snprintf(uart_buf, 200, "%u [app_gnss] [rtos_gnss_thread] $GPRMC - checksum: %u (0x%02X).\r\n",(unsigned int) xTaskGetTickCount(), (unsigned int) pGPRMCInfo->checksum, (unsigned int) pGPRMCInfo->checksum);
  npf_snprintf(uart_buf, 200,  "UTC:\t\t\t\t[ %02d:%02d:%02d ]\n\r", pGPRMCInfo->utc.hh, pGPRMCInfo->utc.mm, pGPRMCInfo->utc.ss);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Status:\t\t\t\t[ %c ]\t\t", pGPRMCInfo->status);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if (pGPRMCInfo->status == (uint8_t)'A')
  {
	npf_snprintf(uart_buf, 200, "-- Valid (reported in 2D and 3D fix conditions)\n\r");
  }
  else if (pGPRMCInfo->status == (uint8_t)'V')
  {
	npf_snprintf(uart_buf, 200, "-- Warning (reported in NO FIX conditions)\n\r");
  }
  else
  {
	npf_snprintf(uart_buf, 200, "-- Unknown status\n\r");
  }
  huart2print(uart_buf, strlen(uart_buf));
  double lat_mod = fmod(pGPRMCInfo->xyz.lat, 100.0);
  double lon_mod = fmod(pGPRMCInfo->xyz.lon, 100.0);
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Latitude:\t\t\t[ %.0f' %02d'' %c ]\n\r", (pGPRMCInfo->xyz.lat - lat_mod) / 100.0, (int16_t)lat_mod, pGPRMCInfo->xyz.ns);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Longitude:\t\t\t[ %.0f' %02d'' %c ]\n\r", (pGPRMCInfo->xyz.lon - lon_mod) / 100.0, (int16_t)lon_mod, pGPRMCInfo->xyz.ew);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Speed over ground (knots):\t[ %.01f ]\n\r", pGPRMCInfo->speed);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Trackgood:\t\t\t[ %.01f ]\n\r", pGPRMCInfo->trackgood);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Date (ddmmyy):\t\t\t[ %02d%02d%02d ]\n\r", (int16_t)((pGPRMCInfo->date/10000)),
     (int16_t)((pGPRMCInfo->date/100) - (100*(pGPRMCInfo->date/10000))), (int16_t)(pGPRMCInfo->date - (100*(pGPRMCInfo->date/100))));
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Magnetic Variation:\t\t[ %.01f ]\n\r", pGPRMCInfo->mag_var);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if ((pGPRMCInfo->mag_var_dir != (uint8_t)'E') && (pGPRMCInfo->mag_var_dir != (uint8_t)'W'))
  {
	npf_snprintf(uart_buf, 200, "Magnetic Var. Direction:\t[ - ]\n\r");
  }
  else {
	npf_snprintf(uart_buf, 200, "Magnetic Var. Direction:\t[ %c ]\n\r", pGPRMCInfo->mag_var_dir);
  }
  huart2print(uart_buf, strlen(uart_buf));
#endif
}

ParseStatus_t NMEA_ParseGPGGA(GPGGA_Info_t *pGPGGAInfo, char NMEA[])
{
//  int32_t new_field;
//  uint8_t valid_msg = 0;
//  ParseStatus_t status = PARSE_FAIL;
//  if(NMEA != NULL)
//  {
//    /* clear the app[][] buffer */
//    for (int8_t i = 0; i < MAX_MSG_LEN; i++)
//    {
//      (void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
//    }
//    for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
//    {
//      new_field = 0;
//
//      if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
//      {
//        app[j][k] = (uint8_t)'\0';
//        new_field = 1;
//
//        if (strcmp((char *)app[0], "$GPGGA") == 0) {
//          j++;
//          k = 0;
//          valid_msg = 1;
//        }
//        else
//        {
//          break;
//        }
//      }
//      if(new_field == 0)
//      {
//        app[j][k] = NMEA[i];
//        k++;
//      }
//    }
//    if (valid_msg)
//    {
//      int32_t valid = strtol((char *)app[6], NULL, 10);
//      if((valid == 1) || (valid == 0))
//      {
//        pGPGGAInfo->valid = (uint8_t)valid;
//      }
//      scan_utc(app[1], &pGPGGAInfo->utc);
//      pGPGGAInfo->xyz.lat      = strtod((char *)app[2], NULL);
//      pGPGGAInfo->xyz.ns       = *((uint8_t*)app[3]);
//      pGPGGAInfo->xyz.lon      = strtod((char *)app[4], NULL);
//      pGPGGAInfo->xyz.ew       = *((uint8_t*)app[5]);
//      pGPGGAInfo->sats         = strtol((char *)app[7], NULL, 10);
//      pGPGGAInfo->acc          = strtof((char *)app[8], NULL);
//      pGPGGAInfo->xyz.alt      = strtof((char *)app[9], NULL);
//      pGPGGAInfo->xyz.mis      = *((uint8_t*)app[10]);
//      pGPGGAInfo->geoid.height = strtol((char *)app[11], NULL, 10);
//      pGPGGAInfo->geoid.mis    = *((uint8_t*)app[12]);
//      // This field is reserved
//      //pGPGGAInfo->update = strtol((char *)app[13], NULL, 10);
//      pGPGGAInfo->checksum     = nmea_checksum(app[15]);
//      status = PARSE_SUCC;
//    }
//  }
//  return status;
  return PARSE_SUCC;
}

/* Puts to console data of correctly parsed GPGGA sentence */
void GNSS_DATA_GetGPGGAInfo(GPGGA_Info_t *pGPGGAInfo)
{
//  if(pGPGGAInfo->valid == 1)
//  {
    double lat_mod = fmod(pGPGGAInfo->xyz.lat, 100.0);
    double lon_mod = fmod(pGPGGAInfo->xyz.lon, 100.0);
    waitToPrint();
    npf_snprintf(uart_buf, 200, "UTC:\t\t\t[ %02d:%02d:%02d ]\n\r", pGPGGAInfo->utc.hh, pGPGGAInfo->utc.mm, pGPGGAInfo->utc.ss);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Latitude full:\t\t[ %.0f° %d' %.4f'' %c ]\n\r", (pGPGGAInfo->xyz.lat - lat_mod) / 100.0, (int16_t)lat_mod, (pGPGGAInfo->xyz.lat - (int16_t)pGPGGAInfo->xyz.lat)*60, pGPGGAInfo->xyz.ns);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Longitude full:\t\t[ %.0f° %d' %.4f'' %c ]\n\r", (pGPGGAInfo->xyz.lon - lon_mod) / 100.0, (int16_t)lon_mod, (pGPGGAInfo->xyz.lon - (int16_t)pGPGGAInfo->xyz.lon)*60, pGPGGAInfo->xyz.ew);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "GPS Quality indicator:\t[ %d ] ", pGPGGAInfo->valid);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    switch (pGPGGAInfo->valid)
      {
      case 0U:
        npf_snprintf(uart_buf, 200, "fix not available.\r\n");
        break;
      case 1U:
        npf_snprintf(uart_buf, 200, "GPS fix.\r\n");
        break;
      case 2U:
        npf_snprintf(uart_buf, 200, "Differential GPS fix.\r\n");
        break;
      case 3U:
        npf_snprintf(uart_buf, 200, "PPS fix.\r\n");
        break;
      case 4U:
        npf_snprintf(uart_buf, 200, "Real Time Kinematic.\r\n");
        break;
      case 5U:
        npf_snprintf(uart_buf, 200, "Float RTK.\r\n");
        break;
      case 6U:
        npf_snprintf(uart_buf, 200, "estimated (dead reckoning).\r\n");
        break;
      case 7U:
        npf_snprintf(uart_buf, 200, "Manual input mode.\r\n");
        break;
      case 8U:
        npf_snprintf(uart_buf, 200, "Simulation mode.\r\n");
    }
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Satellites locked:\t[ %d ]\n\r", pGPGGAInfo->sats);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Position accuracy:\t[ %.1f ]\n\r", pGPGGAInfo->acc);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Altitude:\t\t[ %.2f%c ]\n\r", pGPGGAInfo->xyz.alt, (pGPGGAInfo->xyz.mis + 32U));
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Geoid infos:\t\t[ %d%c ]\n\r", pGPGGAInfo->geoid.height, pGPGGAInfo->geoid.mis);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Diff update:\t\t[ %d ]\n\r", pGPGGAInfo->update);
    huart2print(uart_buf, strlen(uart_buf));
//  }
//  else
//  {
//    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Last position wasn't valid.\n\n\r");
//    huart2print(uart_buf, strlen(uart_buf));
//  }
}

ParseStatus_t NMEA_ParseGSA(GSA_Info_t *pGSAInfo, char NMEA[])
{
//  int32_t new_field;
//  uint8_t valid_msg = 0;
//  ParseStatus_t status = PARSE_FAIL;
//  if(NMEA != NULL)
//  {
//    /* clear the app[][] buffer */
//    for (int8_t i = 0; i < MAX_MSG_LEN; i++)
//    {
//      (void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
//    }
//    for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
//    {
//      new_field = 0;
//      if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
//      {
//        app[j][k] = (uint8_t)'\0';
//        new_field = 1;
////        if (NMEA_CheckGSAMsg((char *)app[0]) == 0)
//        if (strcmp((char *)app[0], "$GNGSA") == 0)
//        {
//          j++;
//          k = 0;
//          valid_msg = 1;
//        }
//        else
//        {
//          break;
//        }
//      }
//      if(new_field == 0)
//      {
//        app[j][k] = NMEA[i];
//        k++;
//      }
//    }
//    if (valid_msg)
//    {
//      (void)strncpy((char *)pGSAInfo->constellation, (char *)app[0], MAX_STR_LEN);
//      pGSAInfo->operating_mode = *((uint8_t*)app[1]);
//      pGSAInfo->current_mode   = strtol((char *)app[2], NULL, 10);
//      int32_t *sat_prn         = pGSAInfo->sat_prn;
//      for (int8_t i = 0; i < MAX_SAT_NUM; i++)
//      {
//        *(&sat_prn[i])         = strtol((char *)app[3+i], NULL, 10);
//      }
//      pGSAInfo->pdop           = strtof((char *)app[15], NULL);
//      pGSAInfo->hdop           = strtof((char *)app[16], NULL);
//      pGSAInfo->vdop           = strtof((char *)app[17], NULL);
//      pGSAInfo->checksum       = nmea_checksum(app[18]);
//      status = PARSE_SUCC;
//    }
//  }
//  return status;
  return PARSE_SUCC;
}

/* Puts to console the info about GSA satellites got by the most recent reception process. */
void GNSS_DATA_GetGSAInfo(GSA_Info_t *pGSAInfo)
{
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Constellation:\t\t\t[ %s ]\t", pGSAInfo->constellation);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if (strcmp((char*)pGSAInfo->constellation, "$GPGSA") == 0)
  {
    npf_snprintf(uart_buf, 200, "-- only GPS constellation is enabled\n\r");
  }
  else if (strcmp((char*)pGSAInfo->constellation, "$GLGSA") == 0)
  {
    npf_snprintf(uart_buf, 200, "-- only GLONASS constellation is enabled\n\r");
  }
  else if (strcmp((char*)pGSAInfo->constellation, "$GAGSA") == 0)
  {
    npf_snprintf(uart_buf, 200, "-- only GALILEO constellation is enabled\n\r");
  }
  else if (strcmp((char*)pGSAInfo->constellation, "$BDGSA") == 0)
  {
    npf_snprintf(uart_buf, 200, "-- only BEIDOU constellation is enabled\n\r");
  }
  else if (strcmp((char*)pGSAInfo->constellation, "$GNGSA") == 0)
  {
    npf_snprintf(uart_buf, 200, "-- more than one constellation is enabled\n\r");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Operating Mode:\t\t\t[ %c ]\t\t", pGSAInfo->operating_mode);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if (pGSAInfo->operating_mode == (uint8_t)'A')
  {
    npf_snprintf(uart_buf, 200, "-- Auto (2D/3D)\n\r");
  }
  else if (pGSAInfo->operating_mode == (uint8_t)'M')
  {
    npf_snprintf(uart_buf, 200, "-- Manual\n\r");
  }
  else
  {
    npf_snprintf(uart_buf, 200, "-- Unknown op mode\n\r");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Current Mode:\t\t\t[ %d ]\t\t", pGSAInfo->current_mode);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if (pGSAInfo->current_mode == 1)
  {
    npf_snprintf(uart_buf, 200, "-- no fix available\n\r");
  }
  else if (pGSAInfo->current_mode == 2)
  {
    npf_snprintf(uart_buf, 200, "-- 2D\n\r");
  }
  else if (pGSAInfo->current_mode == 3)
  {
    npf_snprintf(uart_buf, 200, "-- 3D\n\r");
  }
  huart2print(uart_buf, strlen(uart_buf));
  int16_t *sat_prn = (int16_t*)(pGSAInfo->sat_prn);
  for (uint8_t i = 0; i < 12U; i++)
  {
    waitToPrint();
    npf_snprintf(uart_buf, 200, "SatPRN%02d:\t\t\t[ %d ]\n\r", i+1U, (*(&sat_prn[i])));
    huart2print(uart_buf, strlen(uart_buf));
  }
  waitToPrint();
  npf_snprintf(uart_buf, 200, "PDOP:\t\t\t\t[ %.01f ]\n\r", pGSAInfo->pdop);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "HDOP:\t\t\t\t[ %.01f ]\n\r", pGSAInfo->hdop);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "VDOP:\t\t\t\t[ %.01f ]\n\r", pGSAInfo->vdop);
  huart2print(uart_buf, strlen(uart_buf));
  return;
}

/* Puts to console the info about GSV satellites got by the most recent reception process. */
//void GNSS_DATA_GetGSVInfo(GNSSParser_Data_t *pGNSSParser_Data)
void GNSS_DATA_GetGSVInfo(GSV_Info_t *pGSVInfo)
{
  int16_t i;
  int16_t tot_sats = pGSVInfo->tot_sats;
  int16_t current_sats = pGSVInfo->current_sats;
  int16_t amount = pGSVInfo->amount;
  int16_t number = pGSVInfo->number;
  npf_snprintf(uart_buf, 200, "Constellation:\t\t\t[ %s ]\t-- message to report all ", pGSVInfo->constellation);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  if (strcmp((char*)pGSVInfo->constellation, "$GPGSV") == 0)
  {
    npf_snprintf(uart_buf, 200, "GPS satellites\n\r");
  }
  else if (strcmp((char*)pGSVInfo->constellation, "$GLGSV") == 0)
  {
    npf_snprintf(uart_buf, 200, "GLONASS satellites\n\r");
  }
  else if (strcmp((char*)pGSVInfo->constellation, "$GAGSV") == 0)
  {
    npf_snprintf(uart_buf, 200, "GALILEO satellites\n\r");
  }
  else if (strcmp((char*)pGSVInfo->constellation, "$BDGSV") == 0)
  {
    npf_snprintf(uart_buf, 200, "BEIDOU satellites\n\r");
  }
  else if (strcmp((char*)pGSVInfo->constellation, "$QZGSV") == 0)
  {
    npf_snprintf(uart_buf, 200, "QZSS satellites\n\r");
  }
  else if (strcmp((char*)pGSVInfo->constellation, "$GNGSV") == 0)
  {
    npf_snprintf(uart_buf, 200, "satellites for all enabled constellations\n\r");
  }
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "GSV message:\t\t\t[ %d of %d ]\n\r", number, amount);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  npf_snprintf(uart_buf, 200, "Num of Satellites:\t\t[ %d of %d ]\n\r", pGSVInfo->current_sats, tot_sats);
  huart2print(uart_buf, strlen(uart_buf));
  waitToPrint();
  for (i=0; i<current_sats; i++)
  {
    npf_snprintf(uart_buf, 200, "Sat%02dPRN:\t\t\t[ %03d ]\n\r", i+1+((number-1)*GSV_MSG_SATS), pGSVInfo->gsv_sat_i[i].prn);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Sat%02dElev (°):\t\t\t[ %03d ]\n\r", i+1+((number-1)*GSV_MSG_SATS), pGSVInfo->gsv_sat_i[i].elev);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Sat%02dAzim (°):\t\t\t[ %03d ]\n\r", i+1+((number-1)*GSV_MSG_SATS), pGSVInfo->gsv_sat_i[i].azim);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "Sat%02dCN0 (dB):\t\t\t[ %03d ]\n\r", i+1+((number-1)*GSV_MSG_SATS), pGSVInfo->gsv_sat_i[i].cn0);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
  }
}

/**
 * @brief  Function that converts a character to unsigned integer
 * @param  c        The character to convert
 * @retval The returned unsigned integer
 */
uint32_t char2int(uint8_t c)
{
  uint32_t ret = (unsigned char)0;
  if((c >= (uint8_t)'0') && (c <= (uint8_t)'9'))
  {
    ret = (unsigned char)(c - (uint8_t)'0');
  }
  if((c >= (uint8_t)'A') && (c <= (uint8_t)'F'))
  {
    ret = (unsigned char)(c - (uint8_t)'A') + (unsigned)10;
  }
  if((c >= (uint8_t)'a') && (c <= (uint8_t)'f'))
  {
    ret = (unsigned char)(c - (uint8_t)'a') + (unsigned)10;
  }
  return ret;
}

/*
 * Function that executes the 'OR' operation between first two elements of a buffer
 */
uint32_t nmea_checksum(const uint8_t buf[])
{
  return ((char2int(buf[0]) << 4) | (char2int(buf[1])));
}

/*
 * Function that scans a string with UTC Info_t and fills all fields of a
 * UTC_Info_t struct
 */
void scan_utc(uint8_t *pUTCStr, UTC_Info_t *pUTC)
{
  pUTC->utc = strtol((char *)pUTCStr,NULL,10);
  pUTC->hh = (pUTC->utc / 10000);
  pUTC->mm = (pUTC->utc - (pUTC->hh * 10000)) / 100;
  pUTC->ss = pUTC->utc - ((pUTC->hh * 10000) + (pUTC->mm * 100));
  return;
}

#endif // of #if SENSOR_HOST
