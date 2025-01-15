/*
* ________________________________________________________________________________________________________
* Copyright � 2014-2015 InvenSense Inc. Portions Copyright � 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively �Software�) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/

#include "Icm20948.h"
#include "Icm20948LoadFirmware.h"
#include "Icm20948Defs.h"
#include "Icm20948DataBaseDriver.h"

#include <stdio.h>

#include "usart.h"              // to declare huart2
extern char uart_buf[200];

int inv_icm20948_firmware_load(struct inv_icm20948 * s, const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{ 
  waitToPrint();
  npf_snprintf(uart_buf, 100, " [Icm20948LoadFirmware.c] Load firmware started.\r\n");
  huart2print(uart_buf, strlen(uart_buf));
  npf_snprintf(uart_buf, 100, ".\r\n");
  int write_size;
  int result;
  unsigned short memaddr;
  const unsigned char *data;
  unsigned short size;
  unsigned char data_cmp[INV_MAX_SERIAL_READ];

    //mee bezig op 14/03/2023
//	waitToPrint();
//	npf_snprintf(uart_buf, 100, "lengte data_cmp: %u.\r\n", sizeof(data_cmp)/sizeof(data_cmp[0]));
//	huart2print(uart_buf, strlen(uart_buf));
//	waitToPrint();
//	npf_snprintf(uart_buf, 100, "size_start: %u.\r\n", size_start);
//	huart2print(uart_buf, strlen(uart_buf));

//  int flag = 0;
  if(s->base_state.firmware_loaded)
  {
    return 0;
  }
  // Write DMP memory
  data = data_start;
  size = size_start;
  memaddr = load_addr;
  int first = 1;
  int cmpresult = 0;
  while ((size > 0) && first)
  {
    write_size = min(size, INV_MAX_SERIAL_WRITE);
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      write_size = (memaddr & 0xff) + write_size - 0x100; // Moved across a bank
    }
    cmpresult = 1;
//    while (cmpresult != 0)
//    {

//      waitToPrint();
//      npf_snprintf(uart_buf, 100, "memaddr: 0x%04hX, write_size: %d.\r\n", memaddr, write_size);
//      huart2print(uart_buf, strlen(uart_buf));
//
//      print_hex(data, write_size);

      result = inv_icm20948_write_mems(s, memaddr, write_size, (unsigned char *)data);
//      first = 0;
      if (result)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 100, " [Icm20948LoadFirmware.c] Load firmware: write mems unsuccessful.\r\n");
        huart2print(uart_buf, strlen(uart_buf));
      }
//    if (result)
//    {
//      return result;
//    }
      // verify DMP memory
      result = inv_icm20948_read_mems(s, memaddr, write_size, data_cmp);
      print_hex(data_cmp, write_size);
      cmpresult = memcmp(data_cmp, data, write_size);
      // Print the result of the comparison
      if (cmpresult)
      {
        waitToPrint();
        npf_snprintf(uart_buf, 100, " [Icm20948LoadFirmware.c] Comparison result: %d.\r\n", cmpresult);
        huart2print(uart_buf, strlen(uart_buf));
        return -1;
      }
//    }
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }
//  // Verify DMP memory
//  data = data_start;
//  size = size_start;
//  memaddr = load_addr;
//  while (size > 0)
//  {
//    write_size = min(size, INV_MAX_SERIAL_READ);
//    if ((memaddr & 0xff) + write_size > 0x100)
//    {// Moved across a bank
//      write_size = (memaddr & 0xff) + write_size - 0x100;
//    }
//    result = inv_icm20948_read_mems(s, memaddr, write_size, data_cmp);
////        unsigned char lStartAddrSelected;
////        lStartAddrSelected = (load_addr & 0xff);
////        result = inv_icm20948_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
////        result = inv_icm20948_read_reg(s, REG_MEM_R_W, data_cmp, 16);
//
////	    waitToPrint();
////		npf_snprintf(uart_buf, 100, "Load firmware: read_mems, data = %02X. \r\n", data_cmp[0]);
////		huart2print(uart_buf, strlen(uart_buf));
//
//
////    	print_hex(data_cmp, write_size);
////        if (!result) {
////            waitToPrint();
////        	npf_snprintf(uart_buf, 100, "Load firmware: read mems successful.\r\n");
////        	huart2print(uart_buf, strlen(uart_buf));
////        }
//    if (result)
//    {
//      waitToPrint();
//      npf_snprintf(uart_buf, 100, "Load firmware: read mems error.\r\n");
//      huart2print(uart_buf, strlen(uart_buf));
//      flag++; // Error, DMP not written correctly
//    }
////    	print_hex(data_cmp, write_size);
////    	print_hex(data, write_size);
//
//    if (memcmp(data_cmp, data, write_size))
//    {
////			waitToPrint();
////			npf_snprintf(uart_buf, 100, "lengte data_cmp: %u, lengte data: %u, number of bytes: %d.\r\n", sizeof(data_cmp)/sizeof(data_cmp[0]), strlen((const char*) data), write_size);
////			huart2print(uart_buf, strlen(uart_buf));
//
//      // Compare the arrays
//      int result = memcmp(data_cmp, data, write_size);
//      // Print the result of the comparison
//      waitToPrint();
//      npf_snprintf(uart_buf, 100, "Comparison result: %d.\r\n", result);
//      huart2print(uart_buf, strlen(uart_buf));
//      return -1;
//    }
//    data += write_size;
//    size -= write_size;
//    memaddr += write_size;
//  }
#if defined(WIN32)   
    //if(!flag)
      // inv_log("DMP Firmware was updated successfully..\r\n");
#endif
  return 0;
}

void print_hex(const unsigned char *data, size_t length)
{
  waitToPrint();
  npf_snprintf(uart_buf, 100, "Hex dump:");
  huart2print(uart_buf, strlen(uart_buf));
  for (size_t i = 0; i < length; i++)
  {
 //    printf(" %02X", data[i]);
     waitToPrint();
     npf_snprintf(uart_buf, 100, " %02X", data[i]);
     huart2print(uart_buf, strlen(uart_buf));
  }
  waitToPrint();
  npf_snprintf(uart_buf, 100, "\n");
  huart2print(uart_buf, strlen(uart_buf));
}

