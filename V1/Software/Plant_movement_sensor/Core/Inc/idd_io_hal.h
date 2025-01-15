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

 /**
 * @file
 * @brief Adapter used by the InvenSense device driver to access to the sensor with I2C/SPI
 */

#ifndef _IDD_IO_HAL_H_
#define _IDD_IO_HAL_H_

#include "../../Drivers/Invn/Devices/HostSerif.h"
#include "spi.h"			// header from stm32cubemx code generate

//TODO: tijdelijk ICM20948.h includen voor definition of userbank
#include "ICM20948.h"

#ifdef __cplusplus
extern "C" {
#endif

/* User Configuration */
#define ICM20948_SPI					(&hspi3)

/* Defines */
#define READ							0x80
#define WRITE							0x00

/* Typedefs */
typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

/** @brief Return handle to Serif for SPI
 */
const inv_host_serif_t * idd_io_hal_get_serif_instance_spi(void);

void select_userbank(userbank ub);
//void select_userbank(uint8_t bank);
uint8_t spi_master_write_register(uint8_t register_addr, uint32_t len, const uint8_t *value);
uint8_t spi_master_read_register(uint8_t register_addr, uint32_t len, const uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* _IDD_IO_HAL_H_ */
