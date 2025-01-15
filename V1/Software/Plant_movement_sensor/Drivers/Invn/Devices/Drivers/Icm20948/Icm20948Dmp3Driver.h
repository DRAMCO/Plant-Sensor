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

#ifndef _DMP_3_ICM20948_XFSD_H__
#define _DMP_3_ICM20948_XFSD_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CFG_FIFO_SIZE                   (4184)

// data output control
#define DATA_OUT_CTL1			(4 * 16)
#define DATA_OUT_CTL2			(4 * 16 + 2)
#define DATA_INTR_CTL			(4 * 16 + 12)
#define FIFO_WATERMARK			(31 * 16 + 14)

// motion event control
#define MOTION_EVENT_CTL		(4 * 16 + 14)

// indicates to DMP which sensors are available
/*	1: gyro samples available
	2: accel samples available
	8: secondary samples available	*/
#define DATA_RDY_STATUS			(8 * 16 + 10)

// batch mode
#define BM_BATCH_CNTR			(27 * 16)
#define BM_BATCH_THLD			(19 * 16 + 12)
#define BM_BATCH_MASK			(21 * 16 + 14)

// sensor output data rate
#define ODR_ACCEL				(11 * 16 + 14)
#define ODR_GYRO				(11 * 16 + 10)
#define ODR_CPASS				(11 * 16 +  6)
#define ODR_ALS					(11 * 16 +  2)
#define ODR_QUAT6				(10 * 16 + 12)
#define ODR_QUAT9				(10 * 16 +  8)
#define ODR_PQUAT6				(10 * 16 +  4)
#define ODR_GEOMAG				(10 * 16 +  0)
#define ODR_PRESSURE			(11 * 16 + 12)
#define ODR_GYRO_CALIBR			(11 * 16 +  8)
#define ODR_CPASS_CALIBR		(11 * 16 +  4)

// sensor output data rate counter
#define ODR_CNTR_ACCEL			(9 * 16 + 14)
#define ODR_CNTR_GYRO			(9 * 16 + 10)
#define ODR_CNTR_CPASS			(9 * 16 +  6)
#define ODR_CNTR_ALS			(9 * 16 +  2)
#define ODR_CNTR_QUAT6			(8 * 16 + 12)
#define ODR_CNTR_QUAT9			(8 * 16 +  8)
#define ODR_CNTR_PQUAT6			(8 * 16 +  4)
#define ODR_CNTR_GEOMAG			(8 * 16 +  0)
#define ODR_CNTR_PRESSURE		(9 * 16 + 12)
#define ODR_CNTR_GYRO_CALIBR	(9 * 16 +  8)
#define ODR_CNTR_CPASS_CALIBR	(9 * 16 +  4)

// mounting matrix
#define CPASS_MTX_00            (23 * 16)
#define CPASS_MTX_01            (23 * 16 + 4)
#define CPASS_MTX_02            (23 * 16 + 8)
#define CPASS_MTX_10            (23 * 16 + 12)
#define CPASS_MTX_11            (24 * 16)
#define CPASS_MTX_12            (24 * 16 + 4)
#define CPASS_MTX_20            (24 * 16 + 8)
#define CPASS_MTX_21            (24 * 16 + 12)
#define CPASS_MTX_22            (25 * 16)

#define GYRO_SF					(19 * 16)
#define ACCEL_FB_GAIN			(34 * 16)
#define ACCEL_ONLY_GAIN			(16 * 16 + 12)

// bias calibration
#define GYRO_BIAS_X				(139 * 16 +  4)
#define GYRO_BIAS_Y				(139 * 16 +  8)
#define GYRO_BIAS_Z				(139 * 16 + 12)
#define GYRO_ACCURACY			(138 * 16 +  2)
#define GYRO_BIAS_SET			(138 * 16 +  6)
#define GYRO_LAST_TEMPR			(134 * 16)
#define GYRO_SLOPE_X			( 78 * 16 +  4)
#define GYRO_SLOPE_Y			( 78 * 16 +  8)
#define GYRO_SLOPE_Z			( 78 * 16 + 12)

#define ACCEL_BIAS_X            (110 * 16 +  4)                   // 0x6E4
#define ACCEL_BIAS_Y            (110 * 16 +  8)
#define ACCEL_BIAS_Z            (110 * 16 + 12)
#define ACCEL_ACCURACY			(97 * 16)
#define ACCEL_CAL_RESET			(77 * 16)
#define ACCEL_VARIANCE_THRESH	(93 * 16)
#define ACCEL_CAL_RATE			(94 * 16 + 4)
#define ACCEL_PRE_SENSOR_DATA	(97 * 16 + 4)
#define ACCEL_COVARIANCE		(101 * 16 + 8)
#define ACCEL_ALPHA_VAR			(91 * 16)
#define ACCEL_A_VAR				(92 * 16)
#define ACCEL_CAL_INIT			(94 * 16 + 2)
#define ACCEL_CAL_SCALE_COVQ_IN_RANGE	(194 * 16)
#define ACCEL_CAL_SCALE_COVQ_OUT_RANGE	(195 * 16)
#define ACCEL_CAL_TEMPERATURE_SENSITIVITY	(194 * 16 + 4)
#define ACCEL_CAL_TEMPERATURE_OFFSET_TRIM	(194 * 16 + 12)

#define CPASS_BIAS_X            (126 * 16 +  4)
#define CPASS_BIAS_Y            (126 * 16 +  8)
#define CPASS_BIAS_Z            (126 * 16 + 12)
#define CPASS_ACCURACY			(37 * 16)
#define CPASS_BIAS_SET			(34 * 16 + 14)
#define MAR_MODE				(37 * 16 + 2)
#define CPASS_COVARIANCE		(115 * 16)
#define CPASS_COVARIANCE_CUR	(118 * 16 +  8)
#define CPASS_REF_MAG_3D		(122 * 16)
#define CPASS_CAL_INIT			(114 * 16)
#define CPASS_EST_FIRST_BIAS	(113 * 16)
#define MAG_DISTURB_STATE		(113 * 16 + 2)
#define CPASS_VAR_COUNT			(112 * 16 + 6)
#define CPASS_COUNT_7			( 87 * 16 + 2)
#define CPASS_MAX_INNO			(124 * 16)
#define CPASS_BIAS_OFFSET		(113 * 16 + 4)
#define CPASS_CUR_BIAS_OFFSET	(114 * 16 + 4)
#define CPASS_PRE_SENSOR_DATA	( 87 * 16 + 4)

// Compass Cal params to be adjusted according to sampling rate
#define CPASS_TIME_BUFFER		(112 * 16 + 14)
#define CPASS_RADIUS_3D_THRESH_ANOMALY	(112 * 16 + 8)

#define CPASS_STATUS_CHK		(25 * 16 + 12)

// 9-axis
#define MAGN_THR_9X				(80 * 16)
#define MAGN_LPF_THR_9X			(80 * 16 +  8)
#define QFB_THR_9X				(80 * 16 + 12)

// DMP running counter
#define DMPRATE_CNTR			(18 * 16 + 4)

// pedometer
#define PEDSTD_BP_B				(49 * 16 + 12)
#define PEDSTD_BP_A4			(52 * 16)
#define PEDSTD_BP_A3			(52 * 16 +  4)
#define PEDSTD_BP_A2			(52 * 16 +  8)
#define PEDSTD_BP_A1			(52 * 16 + 12)
#define PEDSTD_SB				(50 * 16 +  8)
#define PEDSTD_SB_TIME			(50 * 16 + 12)
#define PEDSTD_PEAKTHRSH		(57 * 16 +  8)
#define PEDSTD_TIML				(50 * 16 + 10)
#define PEDSTD_TIMH				(50 * 16 + 14)
#define PEDSTD_PEAK				(57 * 16 +  4)
#define PEDSTD_STEPCTR			(54 * 16)
#define PEDSTD_STEPCTR2			(58 * 16 +  8)
#define PEDSTD_TIMECTR			(60 * 16 +  4)
#define PEDSTD_DECI				(58 * 16)
#define PEDSTD_SB2				(60 * 16 + 14)
#define STPDET_TIMESTAMP		(18 * 16 +  8)
#define PEDSTEP_IND				(19 * 16 +  4)
#define PED_Y_RATIO				(17 * 16 +  0)

// SMD
#define SMD_VAR_TH              (141 * 16 + 12)
#define SMD_VAR_TH_DRIVE        (143 * 16 + 12)
#define SMD_DRIVE_TIMER_TH      (143 * 16 +  8)
#define SMD_TILT_ANGLE_TH       (179 * 16 + 12)
#define BAC_SMD_ST_TH           (179 * 16 +  8)
#define BAC_ST_ALPHA4           (180 * 16 + 12)
#define BAC_ST_ALPHA4A          (176 * 16 + 12)

// Wake on Motion
#define WOM_ENABLE              (64 * 16 + 14)
#define WOM_STATUS              (64 * 16 + 6)
#define WOM_THRESHOLD           (64 * 16)
#define WOM_CNTR_TH             (64 * 16 + 12)

// Activity Recognition
#define BAC_RATE                (48  * 16 + 10)
#define BAC_STATE               (179 * 16 +  0)
#define BAC_STATE_PREV          (179 * 16 +  4)
#define BAC_ACT_ON              (182 * 16 +  0)
#define BAC_ACT_OFF             (183 * 16 +  0)
#define BAC_STILL_S_F           (177 * 16 +  0)
#define BAC_RUN_S_F             (177 * 16 +  4)
#define BAC_DRIVE_S_F           (178 * 16 +  0)
#define BAC_WALK_S_F            (178 * 16 +  4)
#define BAC_SMD_S_F             (178 * 16 +  8)
#define BAC_BIKE_S_F            (178 * 16 + 12)
#define BAC_E1_SHORT            (146 * 16 +  0)
#define BAC_E2_SHORT            (146 * 16 +  4)
#define BAC_E3_SHORT            (146 * 16 +  8)
#define BAC_VAR_RUN             (148 * 16 + 12)
#define BAC_TILT_INIT           (181 * 16 +  0)
#define BAC_MAG_ON              (225 * 16 +  0)
#define BAC_PS_ON               (74  * 16 +  0)
#define BAC_BIKE_PREFERENCE     (173 * 16 +  8)
#define BAC_MAG_I2C_ADDR        (229 * 16 +  8)
#define BAC_PS_I2C_ADDR         (75  * 16 +  4)
#define BAC_DRIVE_CONFIDENCE    (144 * 16 +  0)
#define BAC_WALK_CONFIDENCE     (144 * 16 +  4)
#define BAC_SMD_CONFIDENCE      (144 * 16 +  8)
#define BAC_BIKE_CONFIDENCE     (144 * 16 + 12)
#define BAC_STILL_CONFIDENCE    (145 * 16 +  0)
#define BAC_RUN_CONFIDENCE      (145 * 16 +  4)
#define BAC_MODE_CNTR           (150 * 16)
#define BAC_STATE_T_PREV        (185 * 16 +  4)
#define BAC_ACT_T_ON            (184 * 16 +  0)
#define BAC_ACT_T_OFF           (184 * 16 +  4)
#define BAC_STATE_WRDBS_PREV    (185 * 16 +  8)
#define BAC_ACT_WRDBS_ON        (184 * 16 +  8)
#define BAC_ACT_WRDBS_OFF       (184 * 16 + 12)
#define BAC_ACT_ON_OFF          (190 * 16 +  2)
#define PREV_BAC_ACT_ON_OFF     (188 * 16 +  2)
#define BAC_CNTR                (48  * 16 +  2)

// Flip/Pick-up
#define FP_VAR_ALPHA            (245 * 16 +  8)
#define FP_STILL_TH             (246 * 16 +  4)
#define FP_MID_STILL_TH         (244 * 16 +  8)
#define FP_NOT_STILL_TH         (246 * 16 +  8)
#define FP_VIB_REJ_TH           (241 * 16 +  8)
#define FP_MAX_PICKUP_T_TH      (244 * 16 + 12)
#define FP_PICKUP_TIMEOUT_TH    (248 * 16 +  8)
#define FP_STILL_CONST_TH       (246 * 16 + 12)
#define FP_MOTION_CONST_TH      (240 * 16 +  8)
#define FP_VIB_COUNT_TH         (242 * 16 +  8)
#define FP_STEADY_TILT_TH       (247 * 16 +  8)
#define FP_STEADY_TILT_UP_TH    (242 * 16 + 12)
#define FP_Z_FLAT_TH_MINUS      (243 * 16 +  8)
#define FP_Z_FLAT_TH_PLUS       (243 * 16 + 12)
#define FP_DEV_IN_POCKET_TH     (76  * 16 + 12)
#define FP_PICKUP_CNTR          (247 * 16 +  4)
#define FP_RATE                 (240 * 16 + 12)

// Accel FSR
#define ACC_SCALE               (30 * 16 + 0)
#define ACC_SCALE2              (79 * 16 + 4)

// EIS authentication
#define EIS_AUTH_INPUT			(160 * 16 +   4)
#define EIS_AUTH_OUTPUT			(160 * 16 +   0)

// B2S
#define B2S_RATE                (48  * 16 +   8)
// mounting matrix
#define B2S_MTX_00              (208 * 16)
#define B2S_MTX_01              (208 * 16 + 4)
#define B2S_MTX_02              (208 * 16 + 8)
#define B2S_MTX_10              (208 * 16 + 12)
#define B2S_MTX_11              (209 * 16)
#define B2S_MTX_12              (209 * 16 + 4)
#define B2S_MTX_20              (209 * 16 + 8)
#define B2S_MTX_21              (209 * 16 + 12)
#define B2S_MTX_22              (210 * 16)

#define DMP_START_ADDRESS   ((unsigned short)0x1000)
#define DMP_MEM_BANK_SIZE   256
#define DMP_LOAD_START      0x90

#define DMP_CODE_SIZE 14290



/* forward declaration */
struct inv_icm20948;

/* enum for sensor
   The sequence is important.
   It represents the order of apperance from DMP */
enum INV_SENSORS {
	INV_SENSOR_ACCEL = 0,
	INV_SENSOR_GYRO,        
	INV_SENSOR_LPQ,             // 20610:  we'll find out if it breaks 20628 being inserted here....       
	INV_SENSOR_COMPASS,
	INV_SENSOR_ALS,
	INV_SENSOR_SIXQ,
	INV_SENSOR_NINEQ,
	INV_SENSOR_GEOMAG,
INV_SENSOR_PEDQ,
INV_SENSOR_PRESSURE,
	INV_SENSOR_CALIB_GYRO,
	INV_SENSOR_CALIB_COMPASS,
	INV_SENSOR_STEP_COUNTER,
	INV_SENSOR_ACTIVITY_CLASSIFIER,
	INV_SENSOR_FLIP_PICKUP,
    INV_SENSOR_BRING_TO_SEE,

INV_SENSOR_SIXQ_accel,
INV_SENSOR_NINEQ_accel,
INV_SENSOR_GEOMAG_cpass,
INV_SENSOR_NINEQ_cpass,

	INV_SENSOR_WAKEUP_ACCEL,
	INV_SENSOR_WAKEUP_GYRO,        
//INV_SENSOR_WAKEUP_LPQ,
	INV_SENSOR_WAKEUP_COMPASS,
	INV_SENSOR_WAKEUP_ALS,
	INV_SENSOR_WAKEUP_SIXQ,
	INV_SENSOR_WAKEUP_NINEQ,
	INV_SENSOR_WAKEUP_GEOMAG,
INV_SENSOR_WAKEUP_PEDQ,
INV_SENSOR_WAKEUP_PRESSURE,
	INV_SENSOR_WAKEUP_CALIB_GYRO,
	INV_SENSOR_WAKEUP_CALIB_COMPASS,
	INV_SENSOR_WAKEUP_STEP_COUNTER,
	INV_SENSOR_WAKEUP_TILT_DETECTOR,
//INV_SENSOR_WAKEUP_ACTIVITY_CLASSIFIER,

INV_SENSOR_WAKEUP_SIXQ_accel,
INV_SENSOR_WAKEUP_NINEQ_accel,
INV_SENSOR_WAKEUP_GEOMAG_cpass,
INV_SENSOR_WAKEUP_NINEQ_cpass,

	INV_SENSOR_NUM_MAX,
	INV_SENSOR_INVALID,
};


enum accel_cal_params {
    ACCEL_CAL_ALPHA_VAR = 0,
    ACCEL_CAL_A_VAR,
    ACCEL_CAL_DIV,
    NUM_ACCEL_CAL_PARAMS
};

enum compass_cal_params {
	CPASS_CAL_TIME_BUFFER = 0,
	CPASS_CAL_RADIUS_3D_THRESH_ANOMALY,
    NUM_CPASS_CAL_PARAMS
};

int inv_icm20948_load_firmware(struct inv_icm20948 * s, const unsigned char *dmp3_image, unsigned int dmp3_image_size);
void inv_icm20948_get_dmp_start_address(struct inv_icm20948 * s, unsigned short *dmp_cnfg);
int dmp_icm20948_reset_control_registers(struct inv_icm20948 * s);
int dmp_icm20948_set_data_output_control1(struct inv_icm20948 * s, int output_mask);
int dmp_icm20948_set_data_output_control2(struct inv_icm20948 * s, int output_mask);
int dmp_icm20948_set_data_interrupt_control(struct inv_icm20948 * s, uint32_t interrupt_ctl);
int dmp_icm20948_set_FIFO_watermark(struct inv_icm20948 * s, unsigned short fifo_wm);
int dmp_icm20948_set_data_rdy_status(struct inv_icm20948 * s, unsigned short data_rdy);
int dmp_icm20948_set_motion_event_control(struct inv_icm20948 * s, unsigned short motion_mask);
int dmp_icm20948_set_sensor_rate(struct inv_icm20948 * s, int sensor, short divider);
int dmp_icm20948_set_batchmode_params(struct inv_icm20948 * s, unsigned int thld, short mask);
int dmp_icm20948_set_bias_acc(struct inv_icm20948 * s, int *bias);
int dmp_icm20948_set_bias_gyr(struct inv_icm20948 * s, int *bias);
int dmp_icm20948_set_bias_cmp(struct inv_icm20948 * s, int *bias);
int dmp_icm20948_get_bias_acc(struct inv_icm20948 * s, int *bias);
int dmp_icm20948_get_bias_gyr(struct inv_icm20948 * s, int *bias);
int dmp_icm20948_get_bias_cmp(struct inv_icm20948 * s, int *bias);
int dmp_icm20948_set_gyro_sf(struct inv_icm20948 * s, long gyro_sf);
int dmp_icm20948_set_accel_feedback_gain(struct inv_icm20948 * s, int accel_gain);
int dmp_icm20948_set_accel_cal_params(struct inv_icm20948 * s, int *accel_cal);
int dmp_icm20948_set_compass_cal_params(struct inv_icm20948 * s, int *compass_cal);
int dmp_icm20948_set_compass_matrix(struct inv_icm20948 * s, int *compass_mtx);
int dmp_icm20948_get_pedometer_num_of_steps(struct inv_icm20948 * s, unsigned long *steps);
int dmp_icm20948_set_pedometer_rate(struct inv_icm20948 * s, int ped_rate);
int dmp_icm20948_set_wom_enable(struct inv_icm20948 * s, unsigned char enable);
int dmp_icm20948_set_wom_motion_threshold(struct inv_icm20948 * s, int threshold);
int dmp_icm20948_set_wom_time_threshold(struct inv_icm20948 * s, unsigned short threshold);
int dmp_icm20948_set_accel_fsr(struct inv_icm20948 * s, short accel_fsr);
int dmp_icm20948_set_accel_scale2(struct inv_icm20948 * s, short accel_fsr);
int dmp_icm20948_set_eis_auth_input(struct inv_icm20948 * s, long eis_auth_input);
int dmp_icm20948_get_eis_auth_output(struct inv_icm20948 * s, long *eis_auth_output);
int dmp_icm20948_set_bac_rate(struct inv_icm20948 * s, short bac_odr);
int dmp_icm20948_set_b2s_rate(struct inv_icm20948 * s, short accel_odr);
int dmp_icm20948_set_B2S_matrix(struct inv_icm20948 * s, int *b2s_mtx);
int dmp_icm20948_set_fp_rate(struct inv_icm20948 * s, short accel_odr);
int dmp_icm20948_reset_bac_states(struct inv_icm20948 * s);
int dmp_icm20948_set_ped_y_ratio(struct inv_icm20948 * s, long ped_y_ratio);


#ifdef __cplusplus
}
#endif

// _DMP_3_ICM20948_XFSD_H__
#endif
