#include "ICM20948.h"
#include "spi.h"


enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

/* SPI transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

void ICM_20948_bankSelect(uint8_t bank)
{
	uint8_t tx_buf[2];
	tx_buf[0] = ICM_20948_REG_BANK_SEL & (~0x80);
	tx_buf[1] = (bank << 4);

	//HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi3,(uint8_t *)tx_buf, 2);

	if (HAL_SPI_Transmit_DMA(&hspi3,(uint8_t *)tx_buf, 2) != HAL_OK)
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

	uint8_t rx_buf[2];
	tx_buf[0] = ICM_20948_REG_BANK_SEL | 0x80;

	if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx_buf, (uint8_t *)rx_buf, 2) != HAL_OK)
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
}

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

	uint16_t len = numBytes+2;
	if (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx_buf, (uint8_t *)data, len) != HAL_OK)
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


	return;
}


void ICM_20948_registerWrite(uint16_t addr, uint8_t data)
{
	uint8_t regAddr;
	uint8_t bank;


	regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t) (addr >> 7);

	ICM_20948_bankSelect(bank);

	uint8_t tx_buf[2];
	tx_buf[0] = regAddr & (~0x80);
	tx_buf[1] = data;

	uint16_t len = 2;
	if (HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)tx_buf, len) != HAL_OK)
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


	return;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	wTransferState = TRANSFER_COMPLETE;
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
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
