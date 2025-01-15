/*
* ________________________________________________________________________________________________________
* Copyright � 2014 InvenSense Inc.  All rights reserved.
*
* This software and/or documentation  (collectively �Software�) is subject to InvenSense intellectual property rights 
* under U.S. and international copyright and other intellectual property rights laws.
*
* The Software contained herein is PROPRIETARY and CONFIDENTIAL to InvenSense and is provided 
* solely under the terms and conditions of a form of InvenSense software license agreement between 
* InvenSense and you and any use, modification, reproduction or disclosure of the Software without 
* such agreement or the express written consent of InvenSense is strictly prohibited.
*
* EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS 
* PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
* TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
* EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL 
* INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
* DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, 
* NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
* OF THE SOFTWARE.
* ________________________________________________________________________________________________________
*/

#include "Icm20948.h"
#include "Icm20948Dmp3Driver.h"
#include "Icm20948LoadFirmware.h"
#include "Icm20948Defs.h"

#include "usart.h"
extern char     uart_buf[200];
extern uint8_t   dmpBiasFlash[100];


/** Loads the dmp firmware for the icm20948 part.
* @param[in] dmp_image_sram Load DMP3 image from SRAM.
*/
int inv_icm20948_load_firmware(struct inv_icm20948 * s, const unsigned char *dmp3_image, unsigned int dmp3_image_size)
{
  return inv_icm20948_firmware_load(s, dmp3_image, dmp3_image_size, DMP_LOAD_START);
}

/** Loads the dmp firmware for the icm20948 part.
* @param[out] dmp_cnfg The config item
*/
void inv_icm20948_get_dmp_start_address(struct inv_icm20948 * s, unsigned short *dmp_cnfg)
{
  (void)s;
  *dmp_cnfg = DMP_START_ADDRESS;
}

/**
* Sets data output control register 1.
* @param[in] output_mask	Turns sensors on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*							DMP will also turn hw sensors on/off based on bits set in output_mask.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - enable/disable data output in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
int dmp_icm20948_set_data_output_control1(struct inv_icm20948 * s, int output_mask)
{
  
    int result;
    unsigned char data_output_control_reg1[2];
    
    data_output_control_reg1[0] = (unsigned char)(output_mask >> 8);
    data_output_control_reg1[1] = (unsigned char)(output_mask & 0xff);

    result = inv_icm20948_write_mems(s, DATA_OUT_CTL1, 2, data_output_control_reg1);

    return result;
}

/**
* Sets data output control register 2.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*	ACCEL_ACCURACY_SET	0x4000 - accel accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	GYRO_ACCURACY_SET	0x2000 - gyro accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	CPASS_ACCURACY_SET	0x1000 - compass accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
*	BATCH_MODE_EN		0x0100 - enable batching
*/
int dmp_icm20948_set_data_output_control2(struct inv_icm20948 * s, int output_mask)
{
    int result;
	static unsigned char data_output_control_reg2[2]={0};
    
    data_output_control_reg2[0] = (unsigned char)(output_mask >> 8);
    data_output_control_reg2[1] = (unsigned char)(output_mask & 0xff);

    result = inv_icm20948_write_mems(s, DATA_OUT_CTL2, 2, data_output_control_reg2);

    return result;
}

/**
* Clears all output control registers:
*	data output control register 1, data output control register 2, data interrupt control register, motion event control regsiter, data ready status register
*/
int dmp_icm20948_reset_control_registers(struct inv_icm20948 * s)
{
    int result;
    unsigned char data[4]={0};
    
    //reset data output control registers
    result = inv_icm20948_write_mems(s, DATA_OUT_CTL1, 2, &data[0]);
    result += inv_icm20948_write_mems(s, DATA_OUT_CTL2, 2, &data[0]);

	//reset data interrupt control register
    result += inv_icm20948_write_mems(s, DATA_INTR_CTL, 2, &data[0]);

    //reset motion event control register
    result += inv_icm20948_write_mems(s, MOTION_EVENT_CTL, 2, &data[0]);

    //reset data ready status register
    result += inv_icm20948_write_mems(s, DATA_RDY_STATUS, 2, &data[0]);
	//result += inv_icm20948_write_mems(s, DATA_RDY_STATUS, 2, inv_icm20948_convert_int16_to_big8(3, data)); //fixme

    if (result) 
        return result;

	return 0;
}

/**
* Sets data interrupt control register.
* @param[in] interrupt_ctl	Determines which sensors can generate interrupt according to following bit definition,
*							bit set indicates interrupt, bit clear indicates no interrupt.
*
*	ACCEL_SET			0x8000 - calibrated accel if accel calibrated, raw accel otherwise
*	GYRO_SET			0x4000 - raw gyro
*	CPASS_SET			0x2000 - raw magnetic
*	ALS_SET				0x1000 - ALS/proximity
*	QUAT6_SET			0x0800 - game rotation vector
*	QUAT9_SET			0x0400 - rotation vector with heading accuracy
*	PQUAT6_SET			0x0200 - truncated game rotation vector for batching
*	GEOMAG_SET			0x0100 - geomag rotation vector with heading accuracy
*	PRESSURE_SET		0x0080 - pressure
*	GYRO_CALIBR_SET		0x0040 - calibrated gyro
*	CPASS_CALIBR_SET	0x0020 - calibrated magnetic
*	PED_STEPDET_SET		0x0010 - timestamp when each step is detected
*	HEADER2_SET			0x0008 - data output defined in data output control register 2
*	PED_STEPIND_SET		0x0007 - number of steps detected will be attached to the 3 least significant bits of header
*/
int dmp_icm20948_set_data_interrupt_control(struct inv_icm20948 * s, uint32_t interrupt_ctl)
{
	int result;
    unsigned char big8[2]={0};

    result = inv_icm20948_write_mems(s, DATA_INTR_CTL, 2, inv_icm20948_convert_int16_to_big8(interrupt_ctl, big8));

    if (result) 
        return result;

	return 0;
}

/**
* Sets FIFO watermark. DMP will send FIFO interrupt if FIFO count > FIFO watermark
* @param[in] fifo_wm	FIFO watermark set to 80% of actual FIFO size by default
*/
int dmp_icm20948_set_FIFO_watermark(struct inv_icm20948 * s, unsigned short fifo_wm)
{
    int result;
	unsigned char big8[2]={0};

	result = inv_icm20948_write_mems(s, FIFO_WATERMARK, 2, inv_icm20948_convert_int16_to_big8(fifo_wm,big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets data rdy status register.
* @param[in] data_rdy	Indicates which sensor data is available.
*
*	gyro samples available		0x1
*	accel samples available		0x2
*	secondary samples available	0x8
*/
int dmp_icm20948_set_data_rdy_status(struct inv_icm20948 * s, unsigned short data_rdy)
{
	int result;
    unsigned char big8[2]={0};

    result = inv_icm20948_write_mems(s, DATA_RDY_STATUS, 2, inv_icm20948_convert_int16_to_big8(data_rdy, big8));

    if (result) 
        return result;

	return 0;
}

/**
* Sets motion event control register.
* @param[in] output_mask	Turns features on/off according to following bit definition,
*							bit set indicates on, bit clear indicates off.
*
*   BAC_WEAR_EN         0x8000 - change BAC behavior for wearable platform
*	PEDOMETER_EN		0x4000 - pedometer engine
*	PEDOMETER_INT_EN	0x2000 - pedometer step detector interrupt
*	SMD_EN				0x0800 - significant motion detection interrupt
*	ACCEL_CAL_EN		0x0200 - accel calibration
*	GYRO_CAL_EN			0x0100 - gyro calibration
*	COMPASS_CAL_EN		0x0080 - compass calibration
*	NINE_AXIS_EN        0x0040 - 9-axis algorithm execution
*	GEOMAG_EN			0x0008 - Geomag algorithm execution
*	BTS_LTS_EN          0x0004 - bring & look to see
*	BAC_ACCEL_ONLY_EN   0x0002 - run BAC as accel only
*/
int dmp_icm20948_set_motion_event_control(struct inv_icm20948 * s, unsigned short output_mask)
{
    int result;
    unsigned char motion_event_control_reg[2];
    
    motion_event_control_reg[0] = (unsigned char)(output_mask >> 8);
    motion_event_control_reg[1] = (unsigned char)(output_mask & 0xff);

    result = inv_icm20948_write_mems(s, MOTION_EVENT_CTL, 2, motion_event_control_reg);

    return result;
}

/**
* Sets sensor ODR.
* @param[in] sensor		sensor number based on INV_SENSORS
*	enum INV_SENSORS {
*		INV_SENSOR_ACCEL = 0,
*		INV_SENSOR_GYRO,        
*	    INV_SENSOR_LPQ,
*		INV_SENSOR_COMPASS,
*		INV_SENSOR_ALS,
*		INV_SENSOR_SIXQ,
*		INV_SENSOR_NINEQ,
*		INV_SENSOR_GEOMAG,
*		INV_SENSOR_PEDQ,
*		INV_SENSOR_PRESSURE,
*		INV_SENSOR_CALIB_GYRO,
*		INV_SENSOR_CALIB_COMPASS,
*		INV_SENSOR_NUM_MAX,
*		INV_SENSOR_INVALID,
*	};					
* @param[in] divider	desired ODR = base engine rate/(divider + 1)
*/
int dmp_icm20948_set_sensor_rate(struct inv_icm20948 * s, int invSensor, short divider)
{
	int result;
    unsigned char big8[2]={0};
	int odr_addr = 0;

    switch (invSensor) {
		case INV_SENSOR_ACCEL:
			odr_addr = ODR_ACCEL;
			break;
		case INV_SENSOR_GYRO:
			odr_addr = ODR_GYRO;
			break;
		case INV_SENSOR_COMPASS:
			odr_addr = ODR_CPASS;
			break;
		case INV_SENSOR_ALS:
			odr_addr = ODR_ALS;
			break;
		case INV_SENSOR_SIXQ:
			odr_addr = ODR_QUAT6;
			break;
		case INV_SENSOR_NINEQ:
			odr_addr = ODR_QUAT9;
			break;
		case INV_SENSOR_GEOMAG:
			odr_addr = ODR_GEOMAG;
			break;
		case INV_SENSOR_PEDQ:
			odr_addr = ODR_PQUAT6;
			break;
		case INV_SENSOR_PRESSURE:
			odr_addr = ODR_PRESSURE;
			break;
		case INV_SENSOR_CALIB_GYRO:
			odr_addr = ODR_GYRO_CALIBR;
			break;
		case INV_SENSOR_CALIB_COMPASS:
			odr_addr = ODR_CPASS_CALIBR;
			break;
		case INV_SENSOR_STEP_COUNTER:
			//odr_addr = PED_RATE + 2; //PED_RATE is a 4-byte address but only writing 2 bytes here
			break;
    }	

    result = inv_icm20948_write_mems(s, odr_addr, 2, inv_icm20948_convert_int16_to_big8(divider, big8));

	if (result)
        return result;

	return 0;
}

/**
* Resets batch counter and sets batch mode parameters.
* @param[in] thld	sets batch timeout in DMP ticks, e.g. batch 1 sec, thld= (1 sec * engine base rate in Hz)
* @param[in] mask	ties batch counter to engine specified with same bit definiton as HW register DATA_RDY_STATUS,
*					i.e. batch counter increments only if the engine specified is available in multi-rate setting
*	BIT 0 set: 1 - tie to gyro
*	BIT 1 set: 2 - tie to accel
*	BIT 2 set: 4 - tie to pressure in Diamond
*	BIT 3 set: 8 - tie to secondary
*/
int dmp_icm20948_set_batchmode_params(struct inv_icm20948 * s, unsigned int thld, short mask)
{
    int result;
	unsigned char big8[4]={0};
	unsigned char data[2]={0};

	result = inv_icm20948_write_mems(s, BM_BATCH_CNTR, 4, big8);
	result += inv_icm20948_write_mems(s, BM_BATCH_THLD, 4, inv_icm20948_convert_int32_to_big8(thld,big8));
	result += inv_icm20948_write_mems(s, BM_BATCH_MASK, 2, inv_icm20948_convert_int16_to_big8(mask,data));

	if (result)
        return result;
	return 0;
}

/**
* Sets acc's bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*/
int dmp_icm20948_set_bias_acc(struct inv_icm20948 * s, int *bias)
{
	int result;
	unsigned char big8[4]={0};

//	result  = inv_icm20948_write_mems(s, ACCEL_BIAS_X, 4, inv_icm20948_convert_int32_to_big8(bias[0], big8));
//	result += inv_icm20948_write_mems(s, ACCEL_BIAS_Y, 4, inv_icm20948_convert_int32_to_big8(bias[1], big8));
//	result += inv_icm20948_write_mems(s, ACCEL_BIAS_Z, 4, inv_icm20948_convert_int32_to_big8(bias[2], big8));

    big8[0] = dmpBiasFlash[0];
    big8[1] = dmpBiasFlash[1];
    big8[2] = dmpBiasFlash[2];
    big8[3] = dmpBiasFlash[3];
	result  = inv_icm20948_write_mems(s, ACCEL_BIAS_X, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[4];
    big8[1] = dmpBiasFlash[5];
    big8[2] = dmpBiasFlash[6];
    big8[3] = dmpBiasFlash[7];
	result += inv_icm20948_write_mems(s, ACCEL_BIAS_Y, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[8];
    big8[1] = dmpBiasFlash[9];
    big8[2] = dmpBiasFlash[10];
    big8[3] = dmpBiasFlash[11];
	result += inv_icm20948_write_mems(s, ACCEL_BIAS_Z, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[12];
    big8[1] = dmpBiasFlash[13];
	result += inv_icm20948_write_mems(s, ACCEL_ACCURACY, 2, big8);
	
	if (result)
		return result;
	return 0; 
}


/**
* Sets compass' bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] compass_x
*	[1] compass_y
*	[2] compass_z
*/
int dmp_icm20948_set_bias_cmp(struct inv_icm20948 * s, int *bias)
{
	int result;
	unsigned char big8[4]={0};

//	result  = inv_icm20948_write_mems(s, CPASS_BIAS_X, 4, inv_icm20948_convert_int32_to_big8(bias[0], big8));
//	result += inv_icm20948_write_mems(s, CPASS_BIAS_Y, 4, inv_icm20948_convert_int32_to_big8(bias[1], big8));
//	result += inv_icm20948_write_mems(s, CPASS_BIAS_Z, 4, inv_icm20948_convert_int32_to_big8(bias[2], big8));

	big8[0] = dmpBiasFlash[14];
	big8[1] = dmpBiasFlash[15];
	big8[2] = dmpBiasFlash[16];
	big8[3] = dmpBiasFlash[17];
	result  = inv_icm20948_write_mems(s, CPASS_BIAS_X, 4, big8);
    vTaskDelay(100U);
	big8[0] = dmpBiasFlash[18];
	big8[1] = dmpBiasFlash[19];
	big8[2] = dmpBiasFlash[20];
	big8[3] = dmpBiasFlash[21];
	result += inv_icm20948_write_mems(s, CPASS_BIAS_Y, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[22];
    big8[1] = dmpBiasFlash[23];
    big8[2] = dmpBiasFlash[24];
    big8[3] = dmpBiasFlash[25];
	result += inv_icm20948_write_mems(s, CPASS_BIAS_Z, 4, big8);
	big8[0] = dmpBiasFlash[26];
	big8[1] = dmpBiasFlash[27];
	result += inv_icm20948_write_mems(s, CPASS_ACCURACY, 2, big8);

	if (result)
		return result;
	return 0; 
}

/**
* Sets gyro's bias in DMP.
* @param[in] bias
*	array is set as follows:
*	[0] gyro_x
*	[1] gyro_y
*	[2] gyro_z
*/
int dmp_icm20948_set_bias_gyr(struct inv_icm20948 * s, int *bias)
{
	int result;
	unsigned char big8[4]={0};

//	result  = inv_icm20948_write_mems(s, GYRO_BIAS_X, 4, inv_icm20948_convert_int32_to_big8(bias[0], big8));
//	result += inv_icm20948_write_mems(s, GYRO_BIAS_Y, 4, inv_icm20948_convert_int32_to_big8(bias[1], big8));
//	result += inv_icm20948_write_mems(s, GYRO_BIAS_Z, 4, inv_icm20948_convert_int32_to_big8(bias[2], big8));

	big8[0] = dmpBiasFlash[28];
	big8[1] = dmpBiasFlash[29];
	big8[2] = dmpBiasFlash[30];
	big8[3] = dmpBiasFlash[31];
	result  = inv_icm20948_write_mems(s, GYRO_BIAS_X, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[32];
    big8[1] = dmpBiasFlash[33];
    big8[2] = dmpBiasFlash[34];
    big8[3] = dmpBiasFlash[35];
	result += inv_icm20948_write_mems(s, GYRO_BIAS_Y, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[36];
    big8[1] = dmpBiasFlash[37];
    big8[2] = dmpBiasFlash[38];
    big8[3] = dmpBiasFlash[39];
	result += inv_icm20948_write_mems(s, GYRO_BIAS_Z, 4, big8);
    vTaskDelay(100U);
    big8[0] = dmpBiasFlash[40];
    big8[1] = dmpBiasFlash[41];
	result += inv_icm20948_write_mems(s, GYRO_ACCURACY, 2, big8);

	if (result)
		return result;
	return 0; 
}


/**
* Gets acc's bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] accel_x
*	[1] accel_y
*	[2] accel_z
*/
int dmp_icm20948_get_bias_acc(struct inv_icm20948 * s, int *bias)
{
	int result;
	unsigned char big8[4]={0};

	result  = inv_icm20948_read_mems(s, ACCEL_BIAS_X, 4, big8);
	bias[0] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[0] = big8[0];
    dmpBiasFlash[1] = big8[1];
    dmpBiasFlash[2] = big8[2];
    dmpBiasFlash[3] = big8[3];
    vTaskDelay(100U);
	result += inv_icm20948_read_mems(s, ACCEL_BIAS_Y, 4, big8);
	bias[1] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[4] = big8[0];
    dmpBiasFlash[5] = big8[1];
    dmpBiasFlash[6] = big8[2];
    dmpBiasFlash[7] = big8[3];
    vTaskDelay(100U);
	result += inv_icm20948_read_mems(s, ACCEL_BIAS_Z, 4, big8);
	bias[2] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[8] = big8[0];
    dmpBiasFlash[9] = big8[1];
    dmpBiasFlash[10] = big8[2];
    dmpBiasFlash[11] = big8[3];
	result += inv_icm20948_read_mems(s, ACCEL_ACCURACY, 2, big8);
    dmpBiasFlash[12] = big8[0];
    dmpBiasFlash[13] = big8[1];
    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Icm20948Dmp3Driver] ACCEL_BIAS_X, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u, SHIFT --> %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[0], (bias[0]>>9));
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Accelerometer BIAS_X dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[0], (unsigned int)dmpBiasFlash[1], (unsigned int)dmpBiasFlash[2], (unsigned int)dmpBiasFlash[3]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
//    npf_snprintf(uart_buf, 200, "[Icm20948Dmp3Driver] read_mems ACCEL_BIAS_Y, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u, SHIFT --> %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[1], (bias[1]>>9));
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Accelerometer BIAS_Y dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[4], (unsigned int)dmpBiasFlash[5], (unsigned int)dmpBiasFlash[6], (unsigned int)dmpBiasFlash[7]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems ACCEL_BIAS_Z, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u, SHIFT --> %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[2], (bias[2]>>9));
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Accelerometer BIAS_Z dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[8], (unsigned int)dmpBiasFlash[9], (unsigned int)dmpBiasFlash[10], (unsigned int)dmpBiasFlash[11]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Accelerometer ACCURACY dmpAcc[0] = %u, dmpAcc[1] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[12], (unsigned int)dmpBiasFlash[13]);
    huart2print(uart_buf, strlen(uart_buf));

	if (result)
		return result;

	return 0; 
}


/**
* Gets compass' bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] compass_x
*	[1] compass_y
*	[2] compass_z
*/
int dmp_icm20948_get_bias_cmp(struct inv_icm20948 * s, int *bias)
{
	int result;
	unsigned char big8[4]={0};

	result  = inv_icm20948_read_mems(s, CPASS_BIAS_X, 4, big8);
	bias[0] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[14] = big8[0];
    dmpBiasFlash[15] = big8[1];
    dmpBiasFlash[16] = big8[2];
    dmpBiasFlash[17] = big8[3];
    vTaskDelay(100U);
	result += inv_icm20948_read_mems(s, CPASS_BIAS_Y, 4, big8);
	bias[1] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[18] = big8[0];
    dmpBiasFlash[19] = big8[1];
    dmpBiasFlash[20] = big8[2];
    dmpBiasFlash[21] = big8[3];
    vTaskDelay(100U);
	result += inv_icm20948_read_mems(s, CPASS_BIAS_Z, 4, big8);
	bias[2] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[22] = big8[0];
    dmpBiasFlash[23] = big8[1];
    dmpBiasFlash[24] = big8[2];
    dmpBiasFlash[25] = big8[3];
	result += inv_icm20948_read_mems(s, CPASS_ACCURACY, 4, big8);
    dmpBiasFlash[26] = big8[0];
    dmpBiasFlash[27] = big8[1];

    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems MAG_BIAS_X, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[0]);
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Magnetometer BIAS_X dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[14], (unsigned int)dmpBiasFlash[15], (unsigned int)dmpBiasFlash[16], (unsigned int)dmpBiasFlash[17]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    //    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems MAG_BIAS_Y, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[1]);
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Magnetometer BIAS_Y dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[18], (unsigned int)dmpBiasFlash[19], (unsigned int)dmpBiasFlash[20], (unsigned int)dmpBiasFlash[21]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems MAG_BIAS_Z, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[2]);
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Magnetometer BIAS_Z dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[22], (unsigned int)dmpBiasFlash[23], (unsigned int)dmpBiasFlash[24], (unsigned int)dmpBiasFlash[25]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Magnetometer ACCURACY dmpAcc[0] = %u, dmpAcc[1] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[26], (unsigned int)dmpBiasFlash[27]);
    huart2print(uart_buf, strlen(uart_buf));

	if (result)
		return result;

	return 0; 
}

/**
* Gets gyro's bias from DMP.
* @param[in] bias
* @param[out] bias
*	array is set as follows:
*	[0] gyro_x
*	[1] gyro_y
*	[2] gyro_z
*/
int dmp_icm20948_get_bias_gyr(struct inv_icm20948 * s, int *bias)
{
	int result;
	unsigned char big8[4]={0};

	result  = inv_icm20948_read_mems(s, GYRO_BIAS_X, 4, big8);
	bias[0] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[28] = big8[0];
    dmpBiasFlash[29] = big8[1];
    dmpBiasFlash[30] = big8[2];
    dmpBiasFlash[31] = big8[3];
    vTaskDelay(100U);
	result += inv_icm20948_read_mems(s, GYRO_BIAS_Y, 4, big8);
	bias[1] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[32] = big8[0];
    dmpBiasFlash[33] = big8[1];
    dmpBiasFlash[34] = big8[2];
    dmpBiasFlash[35] = big8[3];
    vTaskDelay(100U);
	result += inv_icm20948_read_mems(s, GYRO_BIAS_Z, 4, big8);
	bias[2] = inv_icm20948_convert_big8_to_int32(big8);
    dmpBiasFlash[36] = big8[0];
    dmpBiasFlash[37] = big8[1];
    dmpBiasFlash[38] = big8[2];
    dmpBiasFlash[39] = big8[3];
	result += inv_icm20948_read_mems(s, GYRO_ACCURACY, 4, big8);
    dmpBiasFlash[40] = big8[0];
    dmpBiasFlash[41] = big8[1];

    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems GYRO_BIAS_X, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[0]);
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Gyroscope BIAS_X dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[28], (unsigned int)dmpBiasFlash[29], (unsigned int)dmpBiasFlash[30], (unsigned int)dmpBiasFlash[31]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems GYRO_BIAS_Z, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[2]);
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Gyroscope BIAS_Z dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[36], (unsigned int)dmpBiasFlash[37], (unsigned int)dmpBiasFlash[38], (unsigned int)dmpBiasFlash[39]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
//    npf_snprintf(uart_buf, 200, "Icm20948Dmp3Driver.c read_mems GYRO_BIAS_Y, data = big8[0]: %u, 0x%02X; big8[1]: %u, 0x%02X; big8[2]: %u, 0x%02X; big8[3]: %u, 0x%02X. --> INT32: %u\r\n", big8[0], big8[0], big8[1], big8[1], big8[2], big8[2], big8[3], big8[3], bias[1]);
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Gyroscope BIAS_Y dmpBias[0] = %u, dmpBias[1] = %u, dmpBias[2] = %u, dmpBias[3] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[32], (unsigned int)dmpBiasFlash[33], (unsigned int)dmpBiasFlash[34], (unsigned int)dmpBiasFlash[35]);
    huart2print(uart_buf, strlen(uart_buf));
    waitToPrint();
    npf_snprintf(uart_buf, 200, "%u [Icm20948Dmp3Driver] Gyroscope ACCURACY dmpAcc[0] = %u, dmpAcc[1] = %u.\r\n",
  		  (unsigned int) xTaskGetTickCount(), (unsigned int)dmpBiasFlash[40], (unsigned int)dmpBiasFlash[41]);
    huart2print(uart_buf, strlen(uart_buf));

	if (result)
		return result;

	return 0; 
}


/**
* Sets the gyro_sf used by quaternions on the DMP.
* @param[in] gyro_sf	see inv_icm20948_set_gyro_sf() for value to set based on gyro rate and gyro fullscale range
*/
int dmp_icm20948_set_gyro_sf(struct inv_icm20948 * s, long gyro_sf)
{
    int result;
    unsigned char big8[4];

    result = inv_icm20948_write_mems(s, GYRO_SF, 4, inv_icm20948_convert_int32_to_big8(gyro_sf, big8));

    return result;
}

/**
* Sets the accel gain used by accel quaternion on the DMP.
* @param[in] accel_gain		value changes with accel engine rate
*/
int dmp_icm20948_set_accel_feedback_gain(struct inv_icm20948 * s, int accel_gain)
{
	int result;
    unsigned char big8[4]={0};

    result = inv_icm20948_write_mems(s, ACCEL_ONLY_GAIN, 4, inv_icm20948_convert_int32_to_big8(accel_gain, big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets accel cal parameters based on different accel engine rate/accel cal running rate
* @param[in] accel_cal
*	array is set as follows:
*	[0] = ACCEL_CAL_ALPHA_VAR
*	[1] = ACCEL_CAL_A_VAR
*   [2] = ACCEL_CAL_DIV - divider from hardware accel engine rate such that acce cal runs at accel_engine_rate/(divider+1)
*/
int dmp_icm20948_set_accel_cal_params(struct inv_icm20948 * s, int *accel_cal)
{
	int result;
    unsigned char big8[4]={0};

    result  = inv_icm20948_write_mems(s, ACCEL_ALPHA_VAR, 4, inv_icm20948_convert_int32_to_big8(accel_cal[ACCEL_CAL_ALPHA_VAR], big8));
    result |= inv_icm20948_write_mems(s, ACCEL_A_VAR, 4, inv_icm20948_convert_int32_to_big8(accel_cal[ACCEL_CAL_A_VAR], big8));
    result |= inv_icm20948_write_mems(s, ACCEL_CAL_RATE, 2, inv_icm20948_convert_int16_to_big8(accel_cal[ACCEL_CAL_DIV], big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets compass cal parameters based on different compass engine rate/compass cal running rate
* @param[in] compass_cal
*	array is set as follows:
*	[0] = CPASS_CAL_TIME_BUFFER
*	[1] = CPASS_CAL_ALPHA_VAR
*	[2] = CPASS_CAL_A_VAR
*	[3] = CPASS_CAL_RADIUS_3D_THRESH_ANOMALY
*	[4] = CPASS_CAL_NOMOT_VAR_THRESH
*/
int dmp_icm20948_set_compass_cal_params(struct inv_icm20948 * s, int *compass_cal)
{
	int result;
    unsigned char big8[4]={0};

    result = inv_icm20948_write_mems(s, CPASS_TIME_BUFFER, 2, inv_icm20948_convert_int16_to_big8(compass_cal[CPASS_CAL_TIME_BUFFER], big8));
    result += inv_icm20948_write_mems(s, CPASS_RADIUS_3D_THRESH_ANOMALY, 4, inv_icm20948_convert_int32_to_big8(compass_cal[CPASS_CAL_RADIUS_3D_THRESH_ANOMALY], big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets compass orientation matrix to DMP.
* @param[in] compass_mtx
*/
int dmp_icm20948_set_compass_matrix(struct inv_icm20948 * s, int *compass_mtx)
{
    int result;
    unsigned char big8[4]={0};
	    
    result  = inv_icm20948_write_mems(s, CPASS_MTX_00, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[0], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_01, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[1], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_02, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[2], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_10, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[3], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_11, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[4], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_12, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[5], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_20, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[6], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_21, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[7], big8));
    result += inv_icm20948_write_mems(s, CPASS_MTX_22, 4, inv_icm20948_convert_int32_to_big8(compass_mtx[8], big8));

	if (result)
        return result;

	return 0;
}

/**
* Gets pedometer step count.
* @param[in] steps
* @param[out] steps
*/
int dmp_icm20948_get_pedometer_num_of_steps(struct inv_icm20948 * s, unsigned long *steps)
{
    int result;
	unsigned char big8[4]={0};
    (void)s;
	result = inv_icm20948_read_mems(s, PEDSTD_STEPCTR, 4, big8);
    if (result) 
        return result;
    *steps = (big8[0]*(1L<<24)) + (big8[1]*(1L<<16)) + (big8[2]*256) + big8[3];
    
    return 0;
}

/**
* Sets pedometer engine running rate.
* @param[in] ped_rate	divider based on accel engine rate
*/
int dmp_icm20948_set_pedometer_rate(struct inv_icm20948 * s, int ped_rate)
{
	// int result; 
	// unsigned char big8[4]={0};
	// result = inv_icm20948_write_mems(s, PED_RATE, 4, inv_icm20948_convert_int32_to_big8(ped_rate, big8));
	// if (result)
	//    return result;

	(void)s;
	(void) ped_rate;

	return 0;
}

/**
* Turns software wake on motion feature on/off.
* @param[in] enable		0=off, 1=on
*/
int dmp_icm20948_set_wom_enable(struct inv_icm20948 * s, unsigned char enable)
{
	int result;
    unsigned char big8[2]={0};

	if (enable) {
		big8[1]= 0x1;
    }

    result = inv_icm20948_write_mems(s, WOM_ENABLE, 2, big8);

	if (result)
        return result;

	return 0;
}

/**
* Sets motion threshold to determine motion/no motion for wake on motion feature.
* @param[in] threshold
*/
int dmp_icm20948_set_wom_motion_threshold(struct inv_icm20948 * s, int threshold)
{
	int result;
    unsigned char big8[4]={0};

    result = inv_icm20948_write_mems(s, WOM_THRESHOLD, 4, inv_icm20948_convert_int32_to_big8(threshold, big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets minimum time threshold of no motion before DMP goes to sleep.
* @param[in] threshold
*/
int dmp_icm20948_set_wom_time_threshold(struct inv_icm20948 * s, unsigned short threshold)
{
	int result;
    unsigned char big8[2]={0};

    result = inv_icm20948_write_mems(s, WOM_CNTR_TH, 2, inv_icm20948_convert_int16_to_big8(threshold, big8));

	if (result)
        return result;

	return 0;
}

/**
* Sets scale in DMP to convert accel data to 1g=2^25 regardless of fsr.
* @param[in] fsr for accel parts
             2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.

             For 2g parts, 2g = 2^15 -> 1g = 2^14,.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^14 (<<16) becomes 1g=2^30, to make 1g=2^25, >>5 bits.
             In Q-30 math, >> 5 equals multiply by 2^25 = 33554432.

             For 8g parts, 8g = 2^15 -> 1g = 2^12.
             DMP takes raw accel data and left shifts by 16 bits, so 1g=2^12 (<<16) becomes 1g=2^28, to make 1g=2^25, >>3bits.
             In Q-30 math, >> 3 equals multiply by 2^27 = 134217728.
*/
int dmp_icm20948_set_accel_fsr(struct inv_icm20948 * s, short accel_fsr)
{
    unsigned char reg[4];
    int result;
    long scale;

    switch (accel_fsr) {
    case 2:
        scale =  33554432L;  // 2^25
        break;
    case 4:
        scale =  67108864L;  // 2^26
        break;
    case 8:
        scale = 134217728L;  // 2^27
        break;
    case 16:
        scale = 268435456L;  // 2^28
        break;
    case 32:
        scale = 536870912L;  // 2^29
        break;
    default:
        return -1;
    }

    result = inv_icm20948_write_mems(s, ACC_SCALE, 4, inv_icm20948_convert_int32_to_big8(scale,reg));

    if (result) {
        return result;
    } else {
        return 0;
    }
}

/**
* According to input fsr, a scale factor will be set at memory location ACC_SCALE2
* to convert calibrated accel data to 16-bit format same as what comes out of MPU register.
* It is a reverse scaling of the scale factor written to ACC_SCALE.
* @param[in] fsr for accel parts
			 2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.
*/
int dmp_icm20948_set_accel_scale2(struct inv_icm20948 * s, short accel_fsr)
{
    unsigned char reg[4];
    int result;
    long scale;

    switch (accel_fsr) {
    case 2:
        scale = 524288L;  // 2^19
        break;
    case 4:
        scale = 262144L;  // 2^18
        break;
    case 8:
        scale = 131072L;  // 2^17
        break;
    case 16:
        scale = 65536L;  // 2^16
        break;
    case 32:
        scale = 32768L;  // 2^15
        break;
    default:
        return -1;
    }

    result = inv_icm20948_write_mems(s, ACC_SCALE2, 4, inv_icm20948_convert_int32_to_big8(scale,reg));

    if (result) {
        return result;
    } else {
        return 0;
    }
}

/**
* Sets the input value for EIS library authentication.
* @param[in] eis_auth_input		random value between (-1,1) in Q30
*/
int dmp_icm20948_set_eis_auth_input(struct inv_icm20948 * s, long eis_auth_input)
{
    int result;
    unsigned char big8[4];

    result = inv_icm20948_write_mems(s, EIS_AUTH_INPUT, 4, inv_icm20948_convert_int32_to_big8(eis_auth_input, big8));

    return result;
}

/**
* Gets the output value from DMP for EIS library authentication.
* @param[out] &eis_auth_output
*/
int dmp_icm20948_get_eis_auth_output(struct inv_icm20948 * s, long *eis_auth_output)
{
    int result;
    unsigned char big8[4];

    result = inv_icm20948_read_mems(s, EIS_AUTH_OUTPUT, 4, big8);

	*eis_auth_output = inv_icm20948_convert_big8_to_int32(big8);

    return result;
}

/**
* BAC only works in 56 Hz. Set divider to make sure accel ODR into BAC is 56Hz.
* @param[in] bac_odr. the values are 56 , 112 , 225 450 or 900 Hz
*/
int dmp_icm20948_set_bac_rate(struct inv_icm20948 * s, short bac_odr)
{
    unsigned char reg[4]={0,0,0,0};
    int result;
    short odr;

    switch (bac_odr) {
    case DMP_ALGO_FREQ_56:
        odr = 0;
        break;
    case DMP_ALGO_FREQ_112:
        odr = 1;
        break;
    case DMP_ALGO_FREQ_225:
        odr = 3;
        break;
    case DMP_ALGO_FREQ_450:
        odr = 7;
        break;
    case DMP_ALGO_FREQ_900:
        odr = 15;
        break;
    default:
        return -1;
    }

    result = inv_icm20948_write_mems(s, BAC_RATE, 2, inv_icm20948_convert_int16_to_big8(odr,reg));
    if (result) {
        return result;
    } else {
        return 0;
    }
}

/**
* B2S only works in 56 Hz. Set divider to make sure accel ODR into B2S is 56Hz.
* @param[in] bac_odr. the values are 56 , 112 , 225 450 or 900 Hz
*/
int dmp_icm20948_set_b2s_rate(struct inv_icm20948 * s, short accel_odr)
{
    unsigned char reg[4]={0,0,0,0};
    int result;
    short odr;

    switch (accel_odr) {
    case DMP_ALGO_FREQ_56:
        odr = 0;
        break;
    case DMP_ALGO_FREQ_112:
        odr = 1;
        break;
    case DMP_ALGO_FREQ_225:
        odr = 3;
        break;
    case DMP_ALGO_FREQ_450:
        odr = 7;
        break;
    case DMP_ALGO_FREQ_900:
        odr = 15;
        break;
    default:
        return -1;
    }

    result = inv_icm20948_write_mems(s, B2S_RATE, 2, inv_icm20948_convert_int16_to_big8(odr,reg));
    if (result) {
        return result;
    } else {
        return 0;
    }
}


/**
* Sets B2S accel orientation matrix to DMP.
* @param[in] b2s_mtx. Unit: 1 = 2^30.
*/
int dmp_icm20948_set_B2S_matrix(struct inv_icm20948 * s, int *b2s_mtx)
{
    int result;
    unsigned char big8[4]={0};
	
    result  = inv_icm20948_write_mems(s, B2S_MTX_00, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[0], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_01, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[1], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_02, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[2], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_10, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[3], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_11, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[4], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_12, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[5], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_20, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[6], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_21, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[7], big8));
    result += inv_icm20948_write_mems(s, B2S_MTX_22, 4, inv_icm20948_convert_int32_to_big8(b2s_mtx[8], big8));

	if (result)
        return result;

	return 0;
}


/**
* PickUp only works in 56 Hz. Set divider to make sure accel ODR into PickUp is 56Hz.
* @param[in] bac_odr. the values are 56 , 112 , 225 450 or 900 Hz
*/
int dmp_icm20948_set_fp_rate(struct inv_icm20948 * s, short accel_odr)
{
	unsigned char reg[4]={0,0,0,0};
    int result;
    long odr;

    switch (accel_odr) {
    case DMP_ALGO_FREQ_56:
        odr = 0;
        break;
    case DMP_ALGO_FREQ_112:
        odr = 1;
        break;
    case DMP_ALGO_FREQ_225:
        odr = 3;
        break;
    case DMP_ALGO_FREQ_450:
        odr = 7;
        break;
    case DMP_ALGO_FREQ_900:
        odr = 15;
        break;
    default:
        return -1;
    }

    result = inv_icm20948_write_mems(s, FP_RATE, 4, inv_icm20948_convert_int32_to_big8(odr,reg));
    if (result) {
        return result;
    } else {
        return 0;
    }
}

/**
* Clear BAC states when restarting BAC/SMD/Pedometer/Tilt.
* This avoids false triggering of BAC-related modules.
*/
int dmp_icm20948_reset_bac_states(struct inv_icm20948 * s)
{
    int result;
    unsigned char big8[4]={0,0,0,0};
    unsigned char big8_s[2] = {0,0};
    long reset = 0;
    short reset_s = 0;

    result = inv_icm20948_write_mems(s, BAC_STATE,             4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_STATE_PREV,       4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_ON,           4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_OFF,          4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_STILL_S_F,        4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_RUN_S_F,          4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_DRIVE_S_F,        4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_WALK_S_F,         4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_SMD_S_F,          4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_BIKE_S_F,         4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_E1_SHORT,         4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_E2_SHORT,         4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_E3_SHORT,         4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_VAR_RUN,          4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_DRIVE_CONFIDENCE, 4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_WALK_CONFIDENCE,  4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_SMD_CONFIDENCE,   4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_BIKE_CONFIDENCE,  4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_STILL_CONFIDENCE, 4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_RUN_CONFIDENCE,   4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_MODE_CNTR,        4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_STATE_T_PREV,     4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_T_ON,         4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_T_OFF,        4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_STATE_WRDBS_PREV, 4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_WRDBS_ON,     4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_WRDBS_OFF,    4, inv_icm20948_convert_int32_to_big8(reset, big8));
    result += inv_icm20948_write_mems(s, BAC_ACT_ON_OFF,       2, inv_icm20948_convert_int16_to_big8(reset_s, big8_s));
    result += inv_icm20948_write_mems(s, PREV_BAC_ACT_ON_OFF,  2, inv_icm20948_convert_int16_to_big8(reset_s, big8_s));
    result += inv_icm20948_write_mems(s, BAC_CNTR,             2, inv_icm20948_convert_int16_to_big8(reset_s, big8_s));

	if (result)
        return result;

	return 0;
}

/**
* Set BAC ped y ration
* @param[in] ped_y_ratio: value will influence pedometer result
*/
int dmp_icm20948_set_ped_y_ratio(struct inv_icm20948 * s, long ped_y_ratio)
{
    int result;
    unsigned char big8[4]={0, 0, 0, 0};

    result = inv_icm20948_write_mems(s, PED_Y_RATIO, 4, inv_icm20948_convert_int32_to_big8(ped_y_ratio, big8));

    return result;
}
