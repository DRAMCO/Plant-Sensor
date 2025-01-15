// SPDX-License-Identifier: GPL-2.0-only
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
 * PCF2131.h
 *
 */

#ifndef PCF2131_PCF2131_H_
#define PCF2131_PCF2131_H_


//#include <linux/bcd.h>
//#include <linux/i2c.h>
//#include <linux/module.h>
//#include <linux/of.h>
//#include <linux/regmap.h>
//#include <linux/rtc.h>
//#include <linux/slab.h>
//#include <linux/spi/spi.h>
//#include <linux/watchdog.h>

#include <stdio.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

#define PCF2131_ADDRESS             0xA6
#define MASTER_REQ_READ             0x12
#define MASTER_REQ_WRITE            0x34

#define BIT(nr) (1UL << (nr))


/* Control register 1 */
#define PCF2131_REG_CTRL1			0x00		// Reset Description
#define PCF2131_BIT_CTRL1_SI			BIT(0)  //    0  Second interrupt Enabled = 1 / Disabled = 0
#define PCF2131_BIT_CTRL1_MI			BIT(1)  //    0  Minute interrupt Enabled = 1 / Disabled = 0
#define PCF2131_BIT_CTRL1_12_24			BIT(2)  //    0  24-hour mode = 0, 12-hour mode = 1
#define PCF2131_BIT_CTRL1_POR_OVRD		BIT(3)  //    0  Power On Reset Override (PORO) Enabled = 1 / Disabled = 0 (set to 0 for normal operation)
#define PCF2131_BIT_CTRL1_100TH_S_DIS	BIT(4)  //    1  100th seconds counter Enabled = 0 / Disabled = 1, register 0x06 reset to 0x00
#define PCF2131_BIT_CTRL1_STOP			BIT(5)  //    0  RTC clock runs = 0, RTC clock stopped = 1, CLKOUT still available
#define PCF2131_BIT_CTRL1_TC_DIS		BIT(6)  //    0  Temperature Compensation Enabled = 0 / Disabled = 1
#define PCF2131_BIT_CTRL1_EXT_TEST		BIT(7)  //    0  normal mode = 0, external clock test mode = 1

/* Control register 2 */
#define PCF2131_REG_CTRL2			0x01		// Reset Description
//                                      BIT(0) unused 0
#define PCF2131_BIT_CTRL2_AIE			BIT(1)  //    0  No interrupt generated from the alarm flag = 0 / Alarm Interrupt Enabled = 1
//                                      BIT(2) unused 0
//                                      BIT(3) unused 0
#define PCF2131_BIT_CTRL2_AF			BIT(4)  //    0  No alarm interrupt triggered = 0 / Alarm Interrupt triggered = 1, must be cleared
//                                      BIT(5) unused 0
#define PCF2131_BIT_CTRL2_WDTF			BIT(6)  //    0  No watchdog timer interrupt generated = 0 / Flag set if watchdog timer interrupt
#define PCF2131_BIT_CTRL2_MSF			BIT(7)  //    0  No Minute or Second interrupt generated = 0 / Flag set if Minute or Second interrupt

/* Control register 3 */
#define PCF2131_REG_CTRL3			0x02		// Reset Description
#define PCF2131_BIT_CTRL3_BLIE			BIT(0)  //    0  When Battery Low Flag is set, an interrupt will be generated when 1
#define PCF2131_BIT_CTRL3_BIE			BIT(1)  //    0  When Battery Flag is set,  an interrupt will be generated when 1
#define PCF2131_BIT_CTRL3_BLF			BIT(2)  //    0  Battery status OK = 0, Battery Low = 1, flag is automatically cleared with battery replacement
#define PCF2131_BIT_CTRL3_BF			BIT(3)  //    0  Battery switch-over interrupt occurred = 1, must be cleared
#define PCF2131_BIT_CTRL3_BTSE			BIT(4)  //    0  Time stamped when battery switch-over interrupt occurs = 1
//                                          bits [5:7]: PWRMNG[2:0] Function
//                                                          000     Battery switch-over is enabled in standard mode, battery low detection is enabled
//                                                        001,010   Battery switch-over is enabled in standard mode, battery low detection is disabled
//                                                          011     Battery switch-over is enabled in direct switching mode, battery low detection is enabled
//                                                        100,101   Battery switch-over is enabled in direct switching mode, battery low detection is disabled
//                                                   111  110,111   Battery switch-over is disabled, only one power supply (VDD), battery low detection is disabled

/* Control register 4 */
#define PCF2131_REG_CTRL4			0x03
//                                      BIT(0) unused 0
//                                      BIT(1) unused 0
//                                      BIT(2) unused 0
//                                      BIT(3) unused 0
#define PCF2131_BIT_CTRL4_TSF4			BIT(4) //     0  no time stamp interrupt generated for pin NTS4 = 0
#define PCF2131_BIT_CTRL4_TSF3			BIT(5) //     0  no time stamp interrupt generated for pin NTS3 = 0
#define PCF2131_BIT_CTRL4_TSF2			BIT(6) //     0  no time stamp interrupt generated for pin NTS2 = 0
#define PCF2131_BIT_CTRL4_TSF1			BIT(7) //     0  no time stamp interrupt generated for pin NTS1 = 0

/* Control register 5 */
#define PCF2131_REG_CTRL5			0x04
//                                      BIT(0) unused 0
//                                      BIT(1) unused 0
//                                      BIT(2) unused 0
//                                      BIT(3) unused 0
#define PCF2131_BIT_CTRL5_TSIE4			BIT(4) //     0  Interrupt generated when time stamp flag set of NTS4 = 1
#define PCF2131_BIT_CTRL5_TSIE3			BIT(5) //     0  Interrupt generated when time stamp flag set of NTS3 = 1
#define PCF2131_BIT_CTRL5_TSIE2			BIT(6) //     0  Interrupt generated when time stamp flag set of NTS2 = 1
#define PCF2131_BIT_CTRL5_TSIE1			BIT(7) //     0  Interrupt generated when time stamp flag set of NTS1 = 1

/* Software reset register */
#define PCF2131_REG_SR_RESET		0x05
#define PCF2131_SR_VAL_SoftReset	      0x2C // to trigger a software reset (SR), 0010 1100 (0x2C) must be sent to this register
//                                                A software reset also triggers CPR and CTS:
#define PCF2131_SR_VAL_CPR		          0xA4 //   clear prescaler (CPR), 1010 0100 (0xA4) must be sent
#define PCF2131_SR_VAL_CTS		          0x25 //   clear timestamp (CTS), 0010 0101 (0x25) must be sent
#define PCF2131_SR_VAL_CPR_CTS		      0xA5 //   It is possible to combine CPR and CTS by sending 1010 0101 (0xA5).
//                                                Reading the SR_RESET register will return a fixed pattern: 00100100;
//                                                Any other value sent to this register is ignored

/* Time and date registers */
#define PCF2131_REG_100th_SC		0x06 // Register 100th Seconds in BCD format, value between 0 and 99
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:7]: value 0 to 9 ten's place
#define PCF2131_REG_SC				0x07 // Register Seconds in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
#define PCF2131_BIT_SC_OSF				BIT(7) // Oscillator Stopped Flag: chip reset has occurred since flag was last cleared, clock integrity is not guaranteed = 1
#define PCF2131_REG_MN				0x08 // Register Minutes in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_HR				0x09 // Register Hours in BCD format, value between 0 and 24
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:5]: value 0 to 2 ten's place in 24 Hours mode
//                                          bit [4]: value 0 or 1 ten's place in 12 Hours mode
//                                          bit [5]: value 0 = AM  or 1 = PM in 12 Hours mode
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_DM				0x0A // Register Day in BCD format, value between 1 and 31
//                                          bits [0:3]: value 1 to 9 unit place
//                                          bits [4:5]: value 0 to 3 ten's place
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_DW				0x0B // Register Day of Week in BCD format, value between 0 and 6
//                                          bits [0:3]: value 0 to 6, 0 = Sunday
//                                      BIT(4) unused
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_MO				0x0C // Register Month in BCD format, value between 1 and 12
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 or 1 ten's place
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_YR				0x0D // Register Year in BCD format, value between 0 and 99
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:7]: value 0 or 9 ten's place

/* Alarm registers */
#define PCF2131_SEC_ALARM			0x0E // Register Second Alarm in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
#define PCF2131_AE_S			        BIT(7) // second alarm enabled = 0, disabled = 1
#define PCF2131_MIN_ALARM			0x0F // Register Minute Alarm in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
#define PCF2131_AE_M			        BIT(7) // Minute alarm enabled = 0, disabled = 1
#define PCF2131_HOUR_ALARM			0x10 // Register Hour Alarm in BCD format, value between 0 and 24
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:5]: value 0 to 2 ten's place in 24 Hours mode
//                                          bit [4]: value 0 or 1 ten's place in 12 Hours mode
//                                          bit [5]: value 0 = AM  or 1 = PM in 12 Hours mode
//                                      BIT(6) unused
#define PCF2131_AE_H			        BIT(7) // Hour alarm enabled = 0, disabled = 1
#define PCF2131_DAY_ALARM			0x11 // Register Day Alarm in BCD format, value between 1 and 31
//                                          bits [0:3]: value 1 to 9 unit place
//                                          bits [4:5]: value 0 to 3 ten's place
//                                      BIT(6) unused
#define PCF2131_AE_D			        BIT(7) // Day alarm enabled = 0, disabled = 1
#define PCF2131_WEEKDAY_ALARM		0x12 // Register Weekday Alarm in BCD format, value between 0 and 6
//                                          bits [0-2]: value 0 to 6 unit place, 0 = Sunday
//                                      BIT(3) unused
//                                      BIT(4) unused
//                                      BIT(5) unused
//                                      BIT(6) unused
#define PCF2131_AE_W			        BIT(7) // Weekday alarm enabled = 0, disabled = 1

/* CLKOUT control register */
#define PCF2131_CLKOUT_CTL			0x13 // Clock Out pin control register
//                                      BIT(0) COF[0]: Clock out frequency selection
//                                      BIT(1) COF[1]    COF[2-0]  Frequency (Hz)
//                                      BIT(2) COF[2]       000        32768
//                                                          001        16384
//                                                          010         8192
//                                                          011         4096
//                                                          100         2048
//                                                          101         1024
//                                                          110            1
//                                                          111    CLKOUT = high-Z
//                                      BIT(3) unused
//                                      BIT(4) unused
#define PCF2131_CLKOUT_CTL_OTPR	        BIT(5) // Each IC is calibrated during production and testing of the device. The calibration parameters are stored
//                                                on EPROM cells called One Time Programmable (OTP) cells. It is recommended to process an OTP refresh once
//                                                after the power is up and the oscillator is operating stable. The OTP refresh takes less than 100 ms to complete.
//                                                To perform an OTP refresh, bit OTPR has to be cleared (set to logic 0) and then set to logic 1 again.
//                                                The OTP logic is not reset nor affected by the Software Reset. The OTPR functionality is only reset by the
//                                                initial digital POR. During OTP refresh, VDD has to be above 1.8 V, the rising speed to 1.8 V needs to be
//                                                faster than 2 V/100 ms. After OTP refresh has finished, PCF2131 can operate with VDD as low as 1.2 V.
//                                      BIT(6) TCR[0] Temperature Conversion Rate: The temperature is measured immediately after power-on and then periodically
//                                      BIT(7) TCR[1] with a period set by the temperature conversion rate TCR[1:0]
//                                                      TCR[1-0]   Temperature measurement period (min)
//                                                         00                    32    (default value)
//                                                         01                    16
//                                                         10                     8
//                                                         11                     4

/* Timestamp 1 registers */
#define PCF2131_REG_TS1_CTRL		0x14 // Time stamp 1 register 100th Seconds in BCD format, value between 0 and 19 for 1/20, 0 and 15 for 1/16
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 to 1 ten's place
//                                          - 1⁄16 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 1 [1]
//                                          - 1⁄20 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 0
//                                          [1] The time recorded in the time stamps, when in 100 Hz disable mode (1 Hz mode), will be at least two 16 Hz clocks
//                                          behind the time stamp event and no more than 3 clocks behind. If the exact time of the time stamp event is required then
//                                          subtract 2 sub seconds from the time stamp value and the result will have -0 sub seconds to +1 sub seconds of uncertainty.
//                                      BIT(5) unused
#define PCF2131_BIT_TS1_CTRL_TSOFF		BIT(6) // Time stamp 1 function enabled = 0, disabled = 1
#define PCF2131_BIT_TS1_CTRL_TSM		BIT(7) // Time Stamp 1 Management Flag:
//                                                - when 0: in subsequent events without clearing the time stamp flags, the last event is stored
//                                                - when 1: in subsequent events without clearing the time stamp flags, the first event is stored
#define PCF2131_REG_TS1_SC			0x15 // Register Time Stamp 1 Second in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS1_MN			0x16 // Register Time Stamp 1 Minute in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS1_HR			0x17 // Register Time Stamp 1 Hour in BCD format, value between 0 and 24
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:5]: value 0 to 2 ten's place in 24 Hours mode
//                                          bit [4]: value 0 or 1 ten's place in 12 Hours mode
//                                          bit [5]: value 0 = AM  or 1 = PM in 12 Hours mode
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS1_DM			0x18 // Register Time Stamp 1 Day in BCD format, value between 1 and 31
//                                          bits [0:3]: value 1 to 9 unit place
//                                          bits [4:5]: value 0 to 3 ten's place
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS1_MO			0x19 // Register Time Stamp 1 Month in BCD format, value between 1 and 12
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 or 1 ten's place
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS1_YR			0x1A // Register Time Stamp 1 Year in BCD format, value between 0 and 99
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:7]: value 0 or 9 ten's place

/* Timestamp 2 registers */
#define PCF2131_REG_TS2_CTRL		0x1B // Time stamp 2 register 100th Seconds in BCD format, value between 0 and 19 for 1/20, 0 and 15 for 1/16
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 to 1 ten's place
//                                          - 1⁄16 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 1 [1]
//                                          - 1⁄20 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 0
//                                          [1] The time recorded in the time stamps, when in 100 Hz disable mode (1 Hz mode), will be at least two 16 Hz clocks
//                                          behind the time stamp event and no more than 3 clocks behind. If the exact time of the time stamp event is required then
//                                          subtract 2 sub seconds from the time stamp value and the result will have -0 sub seconds to +1 sub seconds of uncertainty.
//                                      BIT(5) unused
#define PCF2131_BIT_TS2_CTRL_TSOFF		BIT(6) // Time stamp 2 function enabled = 0, disabled = 1
#define PCF2131_BIT_TS2_CTRL_TSM		BIT(7) // Time Stamp 2 Management Flag:
//                                                - when 0: in subsequent events without clearing the time stamp flags, the last event is stored
//                                                - when 1: in subsequent events without clearing the time stamp flags, the first event is stored
#define PCF2131_REG_TS2_SC			0x1C // Register Time Stamp 2 Second in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS2_MN			0x1D // Register Time Stamp 2 Minute in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS2_HR			0x1E // Register Time Stamp 2 Hour in BCD format, value between 0 and 24
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:5]: value 0 to 2 ten's place in 24 Hours mode
//                                          bit [4]: value 0 or 1 ten's place in 12 Hours mode
//                                          bit [5]: value 0 = AM  or 1 = PM in 12 Hours mode
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS2_DM			0x1F // Register Time Stamp 2 Day in BCD format, value between 1 and 31
//                                          bits [0:3]: value 1 to 9 unit place
//                                          bits [4:5]: value 0 to 3 ten's place
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS2_MO			0x20 // Register Time Stamp 2 Month in BCD format, value between 1 and 12
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 or 1 ten's place
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS2_YR			0x21 // Register Time Stamp 2 Year in BCD format, value between 0 and 99
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:7]: value 0 or 9 ten's place

/* Timestamp 3 registers */
#define PCF2131_REG_TS3_CTRL		0x22 // Time stamp 3 register 100th Seconds in BCD format, value between 0 and 19 for 1/20, 0 and 15 for 1/16
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 to 1 ten's place
//                                          - 1⁄16 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 1 [1]
//                                          - 1⁄20 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 0
//                                          [1] The time recorded in the time stamps, when in 100 Hz disable mode (1 Hz mode), will be at least two 16 Hz clocks
//                                          behind the time stamp event and no more than 3 clocks behind. If the exact time of the time stamp event is required then
//                                          subtract 2 sub seconds from the time stamp value and the result will have -0 sub seconds to +1 sub seconds of uncertainty.
//                                      BIT(5) unused
#define PCF2131_BIT_TS3_CTRL_TSOFF		BIT(6) // Time stamp 3 function enabled = 0, disabled = 1
#define PCF2131_BIT_TS3_CTRL_TSM		BIT(7) // Time Stamp 3 Management Flag:
//                                                - when 0: in subsequent events without clearing the time stamp flags, the last event is stored
//                                                - when 1: in subsequent events without clearing the time stamp flags, the first event is stored
#define PCF2131_REG_TS3_SC			0x23 // Register Time Stamp 3 Second in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS3_MN			0x24 // Register Time Stamp 3 Minute in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS3_HR			0x25 // Register Time Stamp 3 Hour in BCD format, value between 0 and 24
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:5]: value 0 to 2 ten's place in 24 Hours mode
//                                          bit [4]: value 0 or 1 ten's place in 12 Hours mode
//                                          bit [5]: value 0 = AM  or 1 = PM in 12 Hours mode
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS3_DM			0x26 // Register Time Stamp 3 Day in BCD format, value between 1 and 31
//                                          bits [0:3]: value 1 to 9 unit place
//                                          bits [4:5]: value 0 to 3 ten's place
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS3_MO			0x27 // Register Time Stamp 3 Month in BCD format, value between 1 and 12
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 or 1 ten's place
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS3_YR			0x28 // Register Time Stamp 3 Year in BCD format, value between 0 and 99
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:7]: value 0 or 9 ten's place

/* Timestamp 4 registers */
#define PCF2131_REG_TS4_CTRL		0x29 // Time stamp 4 register 100th Seconds in BCD format, value between 0 and 19 for 1/20, 0 and 15 for 1/16
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 to 1 ten's place
//                                          - 1⁄16 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 1 [1]
//                                          - 1⁄20 second time stamp information when PCF2131_BIT_CTRL1_100TH_S_DIS = 0
//                                          [1] The time recorded in the time stamps, when in 100 Hz disable mode (1 Hz mode), will be at least two 16 Hz clocks
//                                          behind the time stamp event and no more than 3 clocks behind. If the exact time of the time stamp event is required then
//                                          subtract 2 sub seconds from the time stamp value and the result will have -0 sub seconds to +1 sub seconds of uncertainty.
//                                      BIT(5) unused
#define PCF2131_BIT_TS4_CTRL_TSOFF		BIT(6) // Time stamp 4 function enabled = 0, disabled = 1
#define PCF2131_BIT_TS4_CTRL_TSM		BIT(7) // Time Stamp 4 Management Flag:
//                                                - when 0: in subsequent events without clearing the time stamp flags, the last event is stored
//                                                - when 1: in subsequent events without clearing the time stamp flags, the first event is stored
#define PCF2131_REG_TS4_SC			0x2A // Register Time Stamp 4 Second in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS4_MN			0x2B // Register Time Stamp 4 Minute in BCD format, value between 0 and 59
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:6]: value 0 to 5 ten's place
//                                      BIT(7) unused
#define PCF2131_REG_TS4_HR			0x2C // Register Time Stamp 4 Hour in BCD format, value between 0 and 24
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:5]: value 0 to 2 ten's place in 24 Hours mode
//                                          bit [4]: value 0 or 1 ten's place in 12 Hours mode
//                                          bit [5]: value 0 = AM  or 1 = PM in 12 Hours mode
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS4_DM			0x2D // Register Time Stamp 4 Day in BCD format, value between 1 and 31
//                                          bits [0:3]: value 1 to 9 unit place
//                                          bits [4:5]: value 0 to 3 ten's place
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS4_MO			0x2E // Register Time Stamp 4 Month in BCD format, value between 1 and 12
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bit [4]: value 0 or 1 ten's place
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_REG_TS4_YR			0x2F // Register Time Stamp 4 Year in BCD format, value between 0 and 99
//                                          bits [0:3]: value 0 to 9 unit place
//                                          bits [4:7]: value 0 or 9 ten's place

/* Aging Offset register */
#define PCF2131_REG_AO			    0x30 // The accuracy of the frequency of a quartz crystal depends on its aging. The aging offset adds an adjustment, positive
//                                          or negative, in the temperature compensation circuit which allows correcting the aging effect.
//                                          bits [0:3]: AO, Aging Offset value:
//                                          Decimal Value   Binary Value   Offset in ppm
//                                                0             0000           +16
//                                                1             0001           +14
//                                                2             0010           +12
//                                                3             0011           +10
//                                                4             0100            +8
//                                                5             0101            +6
//                                                6             0110            +4
//                                                7             0111            +2
//                                                8             1000             0  Default Value
//                                                9             1001            -2
//                                               10             1010            -4
//                                               11             1011            -6
//                                               12             1100            -8
//                                               13             1101           -10
//                                               14             1110           -12
//                                               15             1111           -14
//                                      BIT(4) unused
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused

/* Interrupt Mask registers */
#define PCF2131_INT_A_MASK1			0x31       // Reset Description
#define PCF2131_INT_A_MASK1_BLIE        BIT(0) //   1   Battery Low flag Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK1_BIE         BIT(1) //   1   Battery flag Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK1_AIE         BIT(2) //   1   Alarm Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK1_WD_CD       BIT(3) //   1   Watchdog interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK1_SI          BIT(4) //   1   Second Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK1_MI          BIT(5) //   1   Minute Interrupt A mask Enabled = 1, Disabled = 0
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_INT_A_MASK2			0x32       // Reset Description
#define PCF2131_INT_A_MASK2_TSIE4       BIT(0) //   1   Time Stamp 4 Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK2_TSIE3       BIT(1) //   1   Time Stamp 3 Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK2_TSIE2       BIT(2) //   1   Time Stamp 2 Interrupt A mask Enabled = 1, Disabled = 0
#define PCF2131_INT_A_MASK2_TSIE1       BIT(3) //   1   Time Stamp 1 Interrupt A mask Enabled = 1, Disabled = 0
//                                      BIT(4) unused
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_INT_B_MASK1			0x33       // Reset Description
#define PCF2131_INT_B_MASK1_BLIE        BIT(0) //   1   Battery Low flag Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK1_BIE         BIT(1) //   1   Battery flag Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK1_AIE         BIT(2) //   1   Alarm Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK1_WD_CD       BIT(3) //   1   Watchdog interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK1_SI          BIT(4) //   1   Second Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK1_MI          BIT(5) //   1   Minute Interrupt B mask Enabled = 1, Disabled = 0
//                                      BIT(6) unused
//                                      BIT(7) unused
#define PCF2131_INT_B_MASK2			0x34       // Reset Description
#define PCF2131_INT_B_MASK2_TSIE4       BIT(0) //   1   Time Stamp 4 Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK2_TSIE3       BIT(1) //   1   Time Stamp 3 Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK2_TSIE2       BIT(2) //   1   Time Stamp 2 Interrupt B mask Enabled = 1, Disabled = 0
#define PCF2131_INT_B_MASK2_TSIE1       BIT(3) //   1   Time Stamp 1 Interrupt B mask Enabled = 1, Disabled = 0
//                                      BIT(4) unused
//                                      BIT(5) unused
//                                      BIT(6) unused
//                                      BIT(7) unused

/* Watchdog registers */
#define PCF2131_REG_WD_CTL			0x35 // The watchdog timer has four selectable source clocks. It can, for example, be used to detect a micro-controller
//                                          with interrupt and reset capability which is out of control. The watchdog timer counts down from the software
//                                          programmed 8-bit binary value n in register PCF2131_REG_WD_VAL. When the counter reaches 1, the watchdog timer flag
//                                          WDTF (register Control_2) is set logic 1 and an interrupt is generated
//                                          bits [0:1]: timer source clock for watchdog timer
//                                          TF[1:0]   Frequency in Hz   Maximum timer period (PCF2131_REG_WD_VAL = 0xFF) in s
//                                             00            64                       3.984
//                                             01             4                      63.744
//                                             10            1⁄4                   1020
//                                             11           1⁄64                  16320
//                                      BIT(2) unused
//                                      BIT(3) unused
//                                      BIT(4) unused
#define PCF2131_BIT_WD_CTL_TI_TP		BIT(5) // the interrupt pin INTA/B is configured to generate a permanent active signal when MSF flag = 0
//                                                the interrupt pin INTA/B is configured to generate a pulsed signal when MSF flag = 1
//                                      BIT(6) unused
#define PCF2131_BIT_WD_CTL_CD			BIT(7) // watchdog timer interrupt disabled = 0, enabled = 1; the interrupt pin INTA/B is activated when timed out
#define PCF2131_REG_WD_VAL			0x36 // timer period in seconds, value between 0x00 and 0xFF



/**
* PCF2131_time_t - Structure containing time data.
*/
typedef struct
{
  uint64_t halTimestamp;  // timestamp of HAL tick
  uint64_t rtosTimestamp; // timestamp of RTOS tick
  uint8_t  subSeconds;    // 100th of seconds
  uint8_t  seconds;       // decimal value of seconds, value between 0-59
  uint8_t  minutes;       // decimal value of minutes, value between 0-59
  uint8_t  hours;         // decimal value of hours, value between 0 and 23
  uint8_t  dayOfWeek;     // decimal value of day of week, value between 1 (Mon) and 7 (Sun)
  uint8_t  dayOfMonth;    // decimal value of day of month, value between 1 and 31
  uint8_t  month;         // decimal value of month, value between 1 and 12
  uint16_t year;          // decimal value of year
  uint64_t unixTime;
  uint8_t  am_pm;         // 0 for am, 1 for pm
  uint8_t  mode;          // 0 for 24 hour, 1 for 12 hour
}PCF2131_time_t;

//struct pcf2131 {
//	struct rtc_device *rtc;
//	struct watchdog_device wdd;
//	struct regmap *regmap;
//};

uint8_t pcf2131_get_time(PCF2131_time_t * PCF2131_time);
uint8_t pcf2131_set_time(PCF2131_time_t * PCF2131_time);
//uint8_t set_unixTime(PCF2131_time_t PCF2131_time);
uint8_t pcf2131_start_clock(void);
uint8_t pcf2131_is_running(void);
uint8_t bcd2bin(uint8_t val);
uint8_t bin2bcd(uint8_t val);

void pcf2131_get_status(uint8_t *pData, uint8_t val, uint8_t print);
void pcf2131_reset();
void pcf2131_disable_poro();
void pcf2131_si_off(void);

uint32_t pcf2131SecondAlarm(uint32_t setVal);

uint32_t pcf2131SetAging(uint32_t agingOffsetPCF2131);


#endif /* PCF2131_PCF2131_H_ */
