/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexspi.h"
#include "app.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_reset.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
extern status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src);
extern status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId);
extern status_t flexspi_nor_enable_octal_mode(FLEXSPI_Type *base);
extern void flexspi_nor_flash_init(FLEXSPI_Type *base);
/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint8_t s_nor_program_buffer[256];
static uint8_t s_nor_read_buffer[256];

/*******************************************************************************
 * Code
 ******************************************************************************/
flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 98000000,
    .flashSize            = FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 2,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = NOR_CMD_LUT_SEQ_IDX_WRITE,
    .AWRSeqNumber         = 1,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};
#if defined(FLASH_ADESTO)
const uint32_t FastReadSDRLUTCommandSeq[4] = {
    /*  Fast read */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),
};

const uint32_t OctalReadDDRLUTCommandSeq[4] = {
    /*  OPI DDR read */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x0B, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 0x08, kFLEXSPI_Command_READ_DDR, kFLEXSPI_8PAD, 0x04),
};
#endif

const uint32_t customLUTOctalMode[CUSTOM_LUT_LENGTH] = {

    /*  OPI DDR read */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x0B, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 0x08, kFLEXSPI_Command_READ_DDR, kFLEXSPI_8PAD, 0x04),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Default config is for global unprotect entire memory */
    [4 * NOR_CMD_LUT_SEQ_IDX_CONFIG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID_SPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /*  Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_OPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /*  Erase Sector */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),

    /*  Erase Chip */
    [4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x60, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

    /*  Program */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x02, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR, kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_8PAD, 0),

    /* Enter OPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTEROPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xE8, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /*  Dummy write, do nothing when AHB write command is triggered. */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),

    /*  Read status register using Octal DDR read */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x05, kFLEXSPI_Command_DUMMY_DDR, kFLEXSPI_8PAD, 0x04),
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_OPI + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR, kFLEXSPI_8PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x0),
};



int main(void)
{
    uint32_t i = 0;
    status_t status;
    uint8_t vendorID = 0;

    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

#if !(defined(XIP_EXTERNAL_FLASH) && XIP_EXTERNAL_FLASH == 1)
    /* Only ram target needed. */
    RESET_PeripheralReset(kRESET_Flexspi0);
#endif

    /* 392MHz * 1U / 4U = 98MHz */
    BOARD_SetFlexspiClock(EXAMPLE_FLEXSPI, kCLOCK_Pcc0PlatIpSrcPll0Pfd3, 3U, 0U);

    flexspi_nor_flash_init(EXAMPLE_FLEXSPI);

    PRINTF("\r\nFLEXSPI example started!\r\n");

    /* Adesto's octal flash has an limitation for read ID, which is that the Read Manufacturer and Device ID command is
     * limited to a maximum clock frequency of fCLK. */
#if defined(FLASH_ADESTO) && FLASH_ADESTO
    /* Get vendor ID. */
    status = flexspi_nor_get_vendor_id(EXAMPLE_FLEXSPI, &vendorID);
    if (status != kStatus_Success)
    {
        return status;
    }
    PRINTF("Vendor ID: 0x%x\r\n", vendorID);

    /* Enter quad mode. */
    status = flexspi_nor_enable_octal_mode(EXAMPLE_FLEXSPI);
    if (status != kStatus_Success)
    {
        return status;
    }
#else
    /* Enter quad mode. */
    status = flexspi_nor_enable_octal_mode(EXAMPLE_FLEXSPI);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Get vendor ID. */
    status = flexspi_nor_get_vendor_id(EXAMPLE_FLEXSPI, &vendorID);
    if (status != kStatus_Success)
    {
        return status;
    }
    PRINTF("Vendor ID: 0x%x\r\n", vendorID);
#endif

    /* Erase sectors. */
    PRINTF("Erasing Serial NOR over FlexSPI...\r\n");
    status = flexspi_nor_flash_erase_sector(EXAMPLE_FLEXSPI, EXAMPLE_SECTOR * SECTOR_SIZE);
    if (status != kStatus_Success)
    {
        PRINTF("Erase sector failure !\r\n");
        return -1;
    }

#if defined(CACHE_MAINTAIN) && CACHE_MAINTAIN
    DCACHE_InvalidateByRange(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);
#endif

    memset(s_nor_program_buffer, 0xFFU, sizeof(s_nor_program_buffer));
    memcpy(s_nor_read_buffer, (void *)(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE),
           sizeof(s_nor_read_buffer));

    if (memcmp(s_nor_program_buffer, s_nor_read_buffer, sizeof(s_nor_program_buffer)))
    {
        PRINTF("Erase data -  read out data value incorrect !\r\n ");
        return -1;
    }
    else
    {
        PRINTF("Erase data - successfully. \r\n");
    }

    for (i = 0; i < 0xFFU; i++)
    {
        s_nor_program_buffer[i] = i;
    }

    status =
        flexspi_nor_flash_page_program(EXAMPLE_FLEXSPI, EXAMPLE_SECTOR * SECTOR_SIZE, (void *)s_nor_program_buffer);
    if (status != kStatus_Success)
    {
        PRINTF("Page program failure !\r\n");
        return -1;
    }

#if defined(CACHE_MAINTAIN) && CACHE_MAINTAIN
    DCACHE_InvalidateByRange(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);
#endif

    memcpy(s_nor_read_buffer, (void *)(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE),
           sizeof(s_nor_read_buffer));

    if (memcmp(s_nor_read_buffer, s_nor_program_buffer, sizeof(s_nor_program_buffer)) != 0)
    {
        PRINTF("Program data -  read out data value incorrect !\r\n ");
        return -1;
    }
    else
    {
        PRINTF("Program data - successfully. \r\n");
    }

    while (1)
    {
    }
}
