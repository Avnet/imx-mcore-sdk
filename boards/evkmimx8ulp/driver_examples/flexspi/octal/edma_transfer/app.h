/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*${header:start}*/
#include "fsl_cache.h"
/*${header:end}*/
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI                 FLEXSPI0
#define EXAMPLE_CACHE                   CACHE64_CTRL0
#define FLASH_SIZE                      0x1000 /* 32Mb/KByte */
#define EXAMPLE_FLEXSPI_AMBA_BASE       FlexSPI0_AMBA_BASE
#define FLASH_PAGE_SIZE                 256
#define EXAMPLE_SECTOR                  100
#define SECTOR_SIZE                     0x1000 /* 4K */
#define EXAMPLE_FLEXSPI_CLOCK           kCLOCK_FlexSpi0
#define FLASH_PORT                      kFLEXSPI_PortA1
#define EXAMPLE_FLEXSPI_RX_SAMPLE_CLOCK kFLEXSPI_ReadSampleClkExternalInputFromDqsPad

#define NOR_CMD_LUT_SEQ_IDX_READ            0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS      1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE     2
#define NOR_CMD_LUT_SEQ_IDX_READID_SPI      3
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_OPI 4
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR     5
#define NOR_CMD_LUT_SEQ_IDX_CONFIG          6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM     7
#define NOR_CMD_LUT_SEQ_IDX_ENTEROPI        8
#define NOR_CMD_LUT_SEQ_IDX_WRITE           9
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI  10
#define NOR_CMD_LUT_SEQ_IDX_CHIPERASE       11

#define CUSTOM_LUT_LENGTH          64
#define FLASH_BUSY_STATUS_POL      1
#define FLASH_BUSY_STATUS_OFFSET   0
#define FLASH_ERROR_STATUS_MASK    0x0e
#define FLASH_UNPROTECT            1
#define FLASH_UNPROTECTVALUE       0x00
#define FLASH_ENABLE_OCTAL_DDRMODE 0x88
#define FLASH_ADESTO               1
#define CACHE_MAINTAIN             1

/* DMA related. */
#define EXAMPLE_FLEXSPI_DMA (DMA0)

#define EXAMPLE_TX_DMA_CHANNEL_CLOCK kCLOCK_Dma0Ch0
#define EXAMPLE_RX_DMA_CHANNEL_CLOCK kCLOCK_Dma0Ch1

#define FLEXSPI_TX_DMA_REQUEST_SOURCE kDmaRequestMux0FlexSPI0Tx
#define FLEXSPI_RX_DMA_REQUEST_SOURCE kDmaRequestMux0FlexSPI0Rx

#define FLEXSPI_TX_DMA_CHANNEL 0U
#define FLEXSPI_RX_DMA_CHANNEL 1U

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*${variable:start}*/
typedef struct _flexspi_cache_status
{
    volatile bool CacheEnableFlag;
} flexspi_cache_status_t;
/*${variable:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);

/*${prototype:end}*/

#endif /* _APP_H_ */
