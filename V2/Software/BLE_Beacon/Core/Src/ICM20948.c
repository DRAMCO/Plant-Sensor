#include "ICM20948.h"
#include "spi.h"
#include "tim.h"

// Invensense Drivers
#include "../Devices/SerifHal.h"
#include "../Devices/HostSerif.h"
#include "../Devices/DeviceIcm20948.h"
#include "../DynamicProtocol/DynProtocol.h"
#include "../DynamicProtocol/DynProtocolTransportUart.h"
#include "../EmbUtils/Message.h"
//#include "DataConverter.h"

#include "Device.h"
#include "SensorConfig.h"
#include "SensorConfig.h"


#define GRYO_FSR	250 // 250dps
#define ACCEL_FSR	2 //2G
#define IMU_DEFAULT_SAMPL_FREQ  10 // 225 Hz

// forward declarations
int my_serif_open_adapter(void);
int my_serif_close_adapter(void);
int my_serif_open_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
int my_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context);
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg);
static void imu_config_fsr_gyro(int fsr_in);
static void imu_config_fsr_accel(int fsr_in);
static void start_raw_sensors();
static void stop_raw_sensors();


enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

/* SPI transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;


/*
void ICM_20948_bankSelect(uint8_t bank)
{
	uint8_t tx_buf[2];
	tx_buf[0] = ICM_20948_REG_BANK_SEL & (~0x80);
	tx_buf[1] = (bank << 4);

	//HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi3,(uint8_t *)tx_buf, 2);


	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef err = HAL_SPI_Transmit_DMA(&hspi3,(uint8_t *)tx_buf, 2);
	if (err != HAL_OK)
	{
		//Transfer error in transmission process
		Error_Handler();
	}
	while (wTransferState == TRANSFER_WAIT)
	{
	}

	switch (wTransferState)
	{
	  case TRANSFER_COMPLETE :

		break;
	  default :
		Error_Handler();
		break;
	}
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	uint8_t rx_buf[2];
	tx_buf[0] = ICM_20948_REG_BANK_SEL | 0x80;

	if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx_buf, (uint8_t *)rx_buf, 2) != HAL_OK)
	{
	  // Transfer error in transmission process
	  Error_Handler();
	}
	while (wTransferState == TRANSFER_WAIT)
	{
	}

	switch (wTransferState)
	{
	  case TRANSFER_COMPLETE :

		break;
	  default :
		Error_Handler();
		break;
	}


}
*/
/*
void ICM_20948_registerRead(uint16_t addr, int numBytes, uint8_t *data)
{
	uint8_t regAddr;
	uint8_t bank;


	regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t) (addr >> 7);

	ICM_20948_bankSelect(bank);

	uint8_t tx_buf[3];
	tx_buf[0] = regAddr | 0x80;
	tx_buf[1] = 0x00;
	tx_buf[2] = 0x00;

	//uint8_t rx_buf[2];

	uint16_t len = numBytes+1;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx_buf, (uint8_t *)data, len) != HAL_OK)
	{
	  // Transfer error in transmission process
	  Error_Handler();
	}
	while (wTransferState == TRANSFER_WAIT)
	{
	}

	switch (wTransferState)
	{
	  case TRANSFER_COMPLETE :

		break;
	  default :
		Error_Handler();
		break;
	}
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	memcpy(data, data+1, numBytes);

	return;
}
*/
/*
void ICM_20948_registerWrite(uint16_t addr, uint8_t data)
{
	uint8_t regAddr;
	uint8_t bank;


	regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t) (addr >> 7);

	ICM_20948_bankSelect(bank);

	uint8_t tx_buf[2];
	tx_buf[0] = regAddr | 0x00;
	tx_buf[1] = data;

	uint16_t len = 2;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)tx_buf, len) != HAL_OK)
	{
	  // Transfer error in transmission process
	  Error_Handler();
	}
	while (wTransferState == TRANSFER_WAIT)
	{
	}

	switch (wTransferState)
	{
	  case TRANSFER_COMPLETE :

		break;
	  default :
		Error_Handler();
		break;
	}
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	return;
}
*/


static void check_rc(int rc)
{
	if( rc < 0 )
	{
		Error_Handler();
	}
}

/*
 * States for icm20948 device object
 */

static inv_device_icm20948_t device_icm20948;
static uint8_t dmp3_image[] = {
	#include "../Images/icm20948_img.dmp3a.h"
};

inv_device_t * device; /* just a handy variable to keep the handle to device object */



/*
 * A listener onject will handle sensor events
 */
inv_sensor_listener_t sensor_listener = {
        sensor_event_cb, /* callback that will receive sensor events */
        (void *)0xDEAD   /* some pointer passed to the callback */
};




// definition of the instance
const inv_host_serif_t serif_instance_spi = {
        my_serif_open_adapter,
        my_serif_close_adapter,
        my_serif_open_read_reg,
        my_serif_open_write_reg,
		my_adapter_register_interrupt_callback,
        256,
		256,
		INV_HOST_SERIF_TYPE_SPI
};

int my_serif_open_adapter(void)
{
				int rc = 0;
				return rc;
}

int my_serif_close_adapter(void)
{
				int rc = 0;
				return rc;
}

int my_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context)
{
//				(*interrupt_cb) = twi_handler;

        int rc=0;
        return rc;
}

int my_serif_open_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	uint8_t tx_buf[rlen+1];
	uint8_t rx_buf[rlen+1];
	tx_buf[0] = reg | 0x80;
	memset(tx_buf+1, 0x00, rlen);

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	wTransferState = TRANSFER_WAIT;
	if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx_buf, (uint8_t *)rx_buf, rlen+1) != HAL_OK)
	{
	  /* Transfer error in transmission process */
	  Error_Handler();
	}
	while (wTransferState == TRANSFER_WAIT)
	{
	}

	switch (wTransferState)
	{
	  case TRANSFER_COMPLETE :

		break;
	  default :
		Error_Handler();
		break;
	}
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	memcpy(rbuffer, rx_buf+1, rlen);

	return 0;
}

int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{

	uint8_t tx_buf[wlen+1];
	tx_buf[0] = reg | 0x00;

	memcpy(tx_buf+1, wbuffer, wlen);

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	wTransferState = TRANSFER_WAIT;
	if (HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)tx_buf, wlen+1) != HAL_OK)
	{
	  /* Transfer error in transmission process */
	  Error_Handler();
	}
	while (wTransferState == TRANSFER_WAIT)
	{
	}

	switch (wTransferState)
	{
	  case TRANSFER_COMPLETE :

		break;
	  default :
		Error_Handler();
		break;
	}
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	return 0;
}



/*
 * Time implementation for Icm20948.
 */
uint64_t inv_icm20948_get_time_us(void)
{
				//uint32_t time_us = nrf_drv_timer_capture(&TIMER_MICROS, NRF_TIMER_CC_CHANNEL0);
//				NRF_LOG_INFO("Timer value requested: %d", time_us);

	uint32_t time_us = __HAL_TIM_GET_COUNTER(&htim2);
}

/*
 * High resolution sleep implementation for Icm20948.
 * Used at initilization stage. ~100us is sufficient.
 */
void inv_icm20948_sleep_us(int us)
{
        /*
         * You may provide a sleep function that blocks the current programm
         * execution for the specified amount of us
         */

		// Timer 16 -> 0.1 ms resolution
        //(void)us;
		HAL_TIM_Base_Start(&htim16);
		__HAL_TIM_SET_COUNTER(&htim16,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim16) < (uint16_t)(us/100));  // wait for the counter to reach the us input in the parameter
		HAL_TIM_Base_Stop(&htim16);
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

const inv_host_serif_t * idd_io_hal_get_serif_instance_spi(void)
{
	return &serif_instance_spi;
}


void ICM20948_Init(void)
{
		/* To keep track of errors */
		int rc = 0;

		uint8_t whoami;

		rc += inv_host_serif_open(idd_io_hal_get_serif_instance_spi());

		/*
		 * Create ICM20948 Device
		 * Pass to the driver:
		 * - reference to serial interface object,
		 * - reference to listener that will catch sensor events,
		 */
		inv_device_icm20948_init(&device_icm20948, idd_io_hal_get_serif_instance_spi(), &sensor_listener, dmp3_image, sizeof(dmp3_image));
		HAL_Delay(100);

		/*
		 * Simply get generic device handle from Icm20948 Device
		 */
		device = inv_device_icm20948_get_base(&device_icm20948);
		HAL_Delay(100);

		/* Just get the whoami */
		rc += inv_device_whoami(device, &whoami);
		check_rc(rc);
		HAL_Delay(100);


		// Reset to known state
		rc += inv_device_reset(device);
		check_rc(rc);

		/* Configure and initialize the Icm20948 device */
		rc += inv_device_setup(device);
		check_rc(rc);
		HAL_Delay(100);

		//rc += inv_device_load(device, (int) NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, (int) NULL);
		//check_rc(rc);
		//HAL_Delay(100);

		//rc += inv_device_icm20948_setup(&device_icm20948);
		//check_rc(rc);

		imu_config_fsr_gyro(GRYO_FSR);
		imu_config_fsr_accel(ACCEL_FSR);

		start_raw_sensors();



		// Apply stored IMU offsets from flash
		//apply_stored_offsets();
}

static void start_raw_sensors()
{
	int rc = 0;

	rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
	check_rc(rc);
	rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GYROSCOPE, IMU_DEFAULT_SAMPL_FREQ);
	check_rc(rc);
	rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
	check_rc(rc);

	rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
	check_rc(rc);
	rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, IMU_DEFAULT_SAMPL_FREQ);
	check_rc(rc);
	rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
	check_rc(rc);

	rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
	check_rc(rc);
	rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER, IMU_DEFAULT_SAMPL_FREQ);
	check_rc(rc);
	rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
	check_rc(rc);
}


static void stop_raw_sensors()
{
	int rc = 0;

    rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
    check_rc(rc);
    rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
    check_rc(rc);
    rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
    check_rc(rc);
}


static void imu_config_fsr_gyro(int fsr_in)
{
	int rc = 0;

	inv_sensor_config_fsr_t fsr;
	fsr.fsr = (uint32_t) fsr_in;

	// Read previously configured FSR
	inv_sensor_config_fsr_t temp_fsr;
	inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
	check_rc(rc);
	LOG("Sensor GYRO FSR (current): %d", temp_fsr.fsr);

	if(fsr.fsr != temp_fsr.fsr)
	{
		inv_device_set_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_SENSOR_CONFIG_FSR, &fsr, sizeof(fsr));
		check_rc(rc);
		LOG("Sensor GYRO FSR (update) set to: %d", fsr.fsr);
	}else{
		LOG("Sensor GYRO FSR (already) set to: %d", temp_fsr.fsr);
	}

	// Read previously configured FSR
	inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
	check_rc(rc);
	LOG("Sensor GYRO FSR (current): %d", temp_fsr.fsr);
}

static void imu_config_fsr_accel(int fsr_in)
{
	int rc = 0;

	inv_sensor_config_fsr_t fsr;
	fsr.fsr = (uint32_t) fsr_in;

	// Read previously configured FSR
	inv_sensor_config_fsr_t temp_fsr;
	inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
	check_rc(rc);
	LOG("Sensor ACCEL FSR (current): %lu", temp_fsr.fsr);

	if(fsr.fsr != temp_fsr.fsr)
	{
		inv_device_set_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_FSR, &fsr, sizeof(fsr));
		check_rc(rc);
		LOG("Sensor ACCEL FSR (update) set to: %lu", fsr.fsr);
	}else{
		LOG("Sensor ACCEL FSR (already) set to: %lu", temp_fsr.fsr);
	}

	// Read previously configured FSR
	inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER, INV_SENSOR_CONFIG_FSR, &temp_fsr, sizeof(temp_fsr));
	check_rc(rc);

}


void poll_icm20948(void)
{
	  // Poll all data from IMU
	  int rc = inv_device_poll(device);
}


///////////////////////////////////////
// IMU CALLBACK
///////////////////////////////////////

/*
 * Callback called upon sensor event reception
 * This function is called in the same function than inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{

        /* arg will contained the value provided at init time */
        (void)arg;

        (void)event;
        /* ... do something with event */


	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

		size_t len_in;

		uint8_t config_data[1];


	switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {

		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
		{
			LOG("data event: %d %d %d %d", //inv_sensor_str(event->sensor),
					(int)event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
			break;
		}
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
		{

			LOG("data event %s (mg): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.acc.vect[0]*1000),
					(int)(event->data.acc.vect[1]*1000),
					(int)(event->data.acc.vect[2]*1000),
					(int)(event->data.acc.accuracy_flag));

				// Save latest data in buffer
				//imu_data.accel.x =      (int16_t)((event->data.acc.vect[0]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
				//imu_data.accel.y =      (int16_t)((event->data.acc.vect[1]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
				//imu_data.accel.z =      (int16_t)((event->data.acc.vect[2]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));

			break;
		}
		case INV_SENSOR_TYPE_GYROSCOPE:
		{
			LOG("data event %s (mdps): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.accuracy_flag));

				// Save latest data in buffer
				//imu_data.gyro.x =       (int16_t)((event->data.gyr.vect[0]) * (1 << RAW_Q_FORMAT_GYR_COMMA_BITS));
				//imu_data.gyro.y =       (int16_t)((event->data.gyr.vect[1]) * (1 << RAW_Q_FORMAT_GYR_COMMA_BITS));
				//imu_data.gyro.z =       (int16_t)((event->data.gyr.vect[2]) * (1 << RAW_Q_FORMAT_GYR_COMMA_BITS));

			break;
		}
		case INV_SENSOR_TYPE_MAGNETOMETER:
		{

			LOG("data event %s (nT): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.accuracy_flag));

				// Save latest data in buffer
				//imu_data.mag.y =   -(int16_t)((event->data.mag.vect[0]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS)); // Changed axes and inverted. Corrected for rotation of axes.
				//imu_data.mag.x =    (int16_t)((event->data.mag.vect[1]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS)); // Changed axes. Corrected for rotation of axes.
				//imu_data.mag.z =    (int16_t)((event->data.mag.vect[2]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS));

			break;
		}
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
		{
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		}
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		{
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		}
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR: // 6 Axis sensor fusion -> No accuracy flag - no accuracy
		case INV_SENSOR_TYPE_ROTATION_VECTOR: // 9 Axis sensor fusion - accuracy included (no accuracy flag)
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR: // Accel + Mag based quaternions - accuracy included (no accuracy flag)
		case INV_SENSOR_TYPE_ORIENTATION:
		{/*
//			NRF_LOG_INFO("data event %s (e-3):, %d, %d, %d, Accuracy: %d ", inv_sensor_str(event->sensor),
//					(int)(event->data.orientation.x*1000),
//					(int)(event->data.orientation.y*1000),
//					(int)(event->data.orientation.z*1000),
//					(int)(event->data.orientation.accuracy_flag*1000)); // 0 - 3: not calibrated - fully calibrated
		NRF_LOG_INFO("%s:	%d %d %d", // rewritten write funtion to allow easier plotting
					inv_sensor_str(event->sensor),
					(int)(event->data.orientation.x),
					(int)(event->data.orientation.y),
					(int)(event->data.orientation.z));
//					(int)(event->data.orientation.accuracy_flag),
					// (int)(event->data.gyr.accuracy_flag),
					// (int)(event->data.acc.accuracy_flag),
					// (int)(event->data.mag.accuracy_flag)); // 0 - 3: not calibrated - fully calibrated

			fixed_point_t p_euler[3];

			float euler[3];
			euler[0] = event->data.orientation.x;
			euler[1] = event->data.orientation.y;
			euler[2] = event->data.orientation.z;

			p_euler[0] = float_to_fixed_euler(euler[0]);
			p_euler[1] = float_to_fixed_euler(euler[1]);
			p_euler[2] = float_to_fixed_euler(euler[2]);

            imu_data.euler.yaw   = p_euler[0];
            imu_data.euler.pitch  = p_euler[1];
            imu_data.euler.roll    = p_euler[2];
*/
			break;
		}
		case INV_SENSOR_TYPE_BAC:
		case INV_SENSOR_TYPE_STEP_COUNTER:
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		default:
			//NRF_LOG_INFO("DEFAULT");
			// INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
			break;
		}
	}
}

