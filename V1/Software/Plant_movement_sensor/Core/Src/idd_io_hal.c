/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
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

#include "idd_io_hal.h"
#include "ICM20948.h"

// board drivers
//#include "i2c_master.h"
//#include "i2c_slave.h"
//#include "spi_master.h"
//#include "delay.h"

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "usart.h"              // to declare huart2

#include "../../Drivers/Invn/Devices/Drivers/Icm20948/Icm20948Defs.h"
extern char uart_buf[100];

/* Host Serif object definition for SPI ***************************************/

////////////////////////////
// Extern I2C communication
////////////////////////////

//extern volatile bool twi_tx_done ;
//extern volatile bool twi_rx_done;
//
//extern const nrf_drv_twi_t m_twi;
//
//extern ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count);
//extern ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop);
//extern void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

// forward declarations
int my_serif_open_adapter(void);
int my_serif_close_adapter(void);
//int my_serif_open_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
//static uint8_t read_SPI_single_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
//int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
//static uint8_t write_SPI_single_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);

//TODO: use these declarations instead of the ones in ICM20948.h
//static uint8_t spi_master_read_register( unsigned char register_addr, unsigned short len, unsigned char *value);
//static uint8_t spi_master_write_register(unsigned char register_addr, unsigned short len, const unsigned char *value);
//void select_userbank(userbank ub);

static void     cs_high();
static void     cs_low();

int my_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context);

//replaced to main.c
//
//// definition of the instance
//const inv_host_serif_t my_serif_instance = {
//        my_serif_open_adapter,
//        my_serif_close_adapter,
//        //my_serif_open_read_reg,
//		read_single_icm20948_reg,
//        my_serif_open_write_reg,
//		my_adapter_register_interrupt_callback,
//        256,
//		256,
//        INV_HOST_SERIF_TYPE_I2C
//};

// Not used - is integrated in I2C read - write
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

//int my_serif_open_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
//{
//	(void)reg, (void)rbuffer, (void)rlen;
//
//	ret_code_t error = i2c_read_bytes( &m_twi, ICM_20948_I2C_ADDRESS, reg, rbuffer, rlen);
//	if(error == NRF_SUCCESS)
//	{
//		return 0; // On success
//	}else{
//		return -1;	// shall return a negative value on error
//	}
//}

uint8_t spi_master_read_register( uint8_t register_addr, uint32_t len, const uint8_t *value)
{
//    waitToPrint();
//	npf_snprintf(uart_buf, 100, "In idd_io_hal spi_master_read_register. register address = %d, len = %d, value = %d.\r\n", register_addr, len, *value);
//	huart2print(uart_buf, strlen(uart_buf));


  uint8_t regAddr;
  uint8_t bank;

//	regAddr = (uint8_t) (register_addr & 0x7F);
//	bank = (uint8_t) (register_addr >> 7);
//
//	select_userbank(bank);

  uint8_t read_reg = READ | register_addr;
	//uint8_t read_reg = READ | regAddr;
  uint8_t reg_val;
//	select_user_bank(ub);

//    waitToPrint();
//	npf_snprintf(uart_buf, 100, "read_reg = %02x.\r\n", read_reg);
//	huart2print(uart_buf, strlen(uart_buf));


  uint32_t whileIterations = 0; // to prevent lock into while loop
  HAL_SPI_StateTypeDef SPIState = HAL_SPI_GetState(ICM20948_SPI);
  while (SPIState != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[IMU read register] IMU SPI busy before reading register, value = %u.\r\n", (int)SPIState);
    huart2print(uart_buf, strlen(uart_buf));
    SPIState = HAL_SPI_GetState(ICM20948_SPI);
    if (whileIterations++ > 10)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IMU read register] Error: Jump out of while loop IMU SPI busy before reading register.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      SPIState = HAL_SPI_STATE_READY;
    }
  }


  cs_low();
  HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
  HAL_SPI_Receive(ICM20948_SPI, value, len, 1000);
//	HAL_SPI_Receive(ICM20948_SPI, &reg_val, len, 1000);
  cs_high();


  whileIterations = 0;
  SPIState = HAL_SPI_GetState(ICM20948_SPI);
  while (SPIState != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[IMU read register] IMU SPI busy after reading register, value = %u.\r\n", (int)SPIState);
    huart2print(uart_buf, strlen(uart_buf));
    SPIState = HAL_SPI_GetState(ICM20948_SPI);
    if (whileIterations++ > 10)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IMU read register] Error: Jump out of while loop IMU SPI busy after reading register.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      SPIState = HAL_SPI_STATE_READY;
    }
  }


//	*value = reg_val;

 //   waitToPrint();
//	npf_snprintf(uart_buf, 100, "In idd_io_hal na SPI transmit. read_reg = %02x, len = %ld, reg_val = %02X. \r\n", read_reg, len, *value);
//	huart2print(uart_buf, strlen(uart_buf));

  return 0; //TODO: change to something meaningful about spi communication

}

//static uint8_t idd_io_hal_write_reg_spi(uint8_t reg, uint8_t * wbuffer, uint32_t wlen)
uint8_t spi_master_write_register(uint8_t register_addr, uint32_t len, const uint8_t *value)
{

  uint8_t regAddr;
  uint8_t bank;

//	regAddr = (uint8_t) (register_addr & 0x7F);
//	bank = (uint8_t) (register_addr >> 7);
//
//	select_userbank(bank);

//    waitToPrint();
//	npf_snprintf(uart_buf, 100, "Register address: %02X, bank: %d.\r\n", register_addr, bank);
//	huart2print(uart_buf, strlen(uart_buf));

  uint8_t write_reg = WRITE | register_addr;
	//uint8_t write_reg = WRITE | regAddr;


  uint32_t whileIterations = 0; // to prevent lock into while loop
  HAL_SPI_StateTypeDef SPIState = HAL_SPI_GetState(ICM20948_SPI);
  while (SPIState != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[IMU write register] IMU SPI busy before writing register, value = %u.\r\n", (int)SPIState);
    huart2print(uart_buf, strlen(uart_buf));
    SPIState = HAL_SPI_GetState(ICM20948_SPI);
    if (whileIterations++ > 10)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IMU write register] Error: Jump out of while loop IMU SPI busy before writing register.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      SPIState = HAL_SPI_STATE_READY;
    }
  }

  cs_low();
  HAL_SPI_Transmit(ICM20948_SPI, &write_reg, 1, 1000);
  HAL_SPI_Transmit(ICM20948_SPI, value, len, 1000);
  cs_high();


  whileIterations = 0;
  SPIState = HAL_SPI_GetState(ICM20948_SPI);
  while (SPIState != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[IMU write register] IMU SPI busy after writing register, value = %u.\r\n", (int)SPIState);
    huart2print(uart_buf, strlen(uart_buf));
    SPIState = HAL_SPI_GetState(ICM20948_SPI);
    if (whileIterations++ > 10)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IMU write register] Error: Jump out of while loop IMU SPI busy after writing register.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      SPIState = HAL_SPI_STATE_READY;
    }
  }


//    waitToPrint();
//	npf_snprintf(uart_buf, 100, "SPI write. write_reg = %02x, len = %ld, value = %d. \r\n", *write_reg, len, *value);
//	huart2print(uart_buf, strlen(uart_buf));

//  if (HAL_SPI_GetState(ICM20948_SPI) != HAL_SPI_STATE_READY)
//  {
//    waitToPrint();
//    npf_snprintf(uart_buf, 100, "SPI write not successful.\r\n");
//    huart2print(uart_buf, strlen(uart_buf));
//    return 1;
//  }
//  else
//  {
////	    waitToPrint();
////		npf_snprintf(uart_buf, 100, "SPI write successful.\r\n");
////		huart2print(uart_buf, strlen(uart_buf));
    return 0;
//  }
}


/* Static Functions */
static void cs_high()
{
  HAL_GPIO_WritePin(CS_ICM20948_GPIO_Port, CS_ICM20948_Pin, SET);
}

static void cs_low()
{
  HAL_GPIO_WritePin(CS_ICM20948_GPIO_Port, CS_ICM20948_Pin, RESET);
}

//int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
//{
//	(void)reg, (void)wbuffer, (void)wlen;
//
//	ret_code_t error = i2c_write_byte( &m_twi, ICM_20948_I2C_ADDRESS, reg, wbuffer, wlen, false);
//
//	if(error == NRF_SUCCESS)
//	{
//		return 0;   // On success
//	}else{
//		return -1;	// shall return a negative value on error
//	}
//}

void select_userbank(userbank ub)
//void select_userbank(uint8_t bank)
{
  uint8_t write_reg[2];
  write_reg[0] = WRITE | REG_BANK_SEL;
  write_reg[1] = ub;
	//write_reg[1] = (bank << 4);


  uint32_t whileIterations = 0; // to prevent lock into while loop
  HAL_SPI_StateTypeDef SPIState = HAL_SPI_GetState(ICM20948_SPI);
  while (SPIState != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[IMU select user bank] IMU SPI busy before selecting user bank, value = %u.\r\n", (int)SPIState);
    huart2print(uart_buf, strlen(uart_buf));
    SPIState = HAL_SPI_GetState(ICM20948_SPI);
    if (whileIterations++ > 10)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IMU select user bank] Error: Jump out of while loop IMU SPI busy before selecting user bank.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      SPIState = HAL_SPI_STATE_READY;
    }
  }

  cs_low();
  HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
  cs_high();


  whileIterations = 0;
  SPIState = HAL_SPI_GetState(ICM20948_SPI);
  while (SPIState != HAL_SPI_STATE_READY)
  {
    vTaskDelay(10);
    waitToPrint();
    npf_snprintf(uart_buf, 100, "[IMU select user bank] IMU SPI busy after selecting user bank, value = %u.\r\n", (int)SPIState);
    huart2print(uart_buf, strlen(uart_buf));
    SPIState = HAL_SPI_GetState(ICM20948_SPI);
    if (whileIterations++ > 10)
    {
      waitToPrint();
      npf_snprintf(uart_buf, 200, "%u [IMU select user bank] Error: Jump out of while loop IMU SPI busy after selecting user bank.\r\n",(unsigned int) xTaskGetTickCount());
      huart2print(uart_buf, strlen(uart_buf));
      SPIState = HAL_SPI_STATE_READY;
    }
  }

}

static int idd_io_hal_init_spi(void)
{
	//spi_master_init(SPI_NUM1, SPI_1562KHZ);
  return 0;
}

static int idd_io_hal_read_reg_spi(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  return spi_master_read_register(reg, rlen, rbuffer);
}

static int idd_io_hal_write_reg_spi(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  return spi_master_write_register(reg, wlen, wbuffer);
}

static const inv_host_serif_t serif_instance_spi = {
  idd_io_hal_init_spi,
        0,
	idd_io_hal_read_reg_spi, //my_serif_open_read_reg, in software Jona
	idd_io_hal_write_reg_spi, //is my_serif_open_write_reg in software Jona
	0,
	1024*32, /* max transaction size */
	1024*32, /* max transaction size */
	INV_HOST_SERIF_TYPE_SPI,
};

const inv_host_serif_t * idd_io_hal_get_serif_instance_spi(void)
{
  return &serif_instance_spi;
}
