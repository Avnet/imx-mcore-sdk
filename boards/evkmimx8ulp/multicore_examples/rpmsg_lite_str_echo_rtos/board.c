/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
#include "fsl_lpi2c.h"
#endif /* SDK_I2C_BASED_COMPONENT_USED */
#include "fsl_flexspi.h"
#include "fsl_upower.h"
#include "fsl_reset.h"
#include "fsl_cache.h"
#if defined(BOARD_USE_TPM) && BOARD_USE_TPM
#include "fsl_tpm.h"
#endif /* BOARD_USE_TPM */
#if defined(BOARD_USE_PCA6416A) && BOARD_USE_PCA6416A
#include "fsl_pca6416a.h"
#endif /* BOARD_USE_PCA6416A */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_FLEXSPI_DLL_LOCK_RETRY (10)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq;

    CLOCK_SetIpSrc(BOARD_DEBUG_UART_IP_NAME, BOARD_DEBUG_UART_CLKSRC);
    uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    RESET_PeripheralReset(BOARD_DEBUG_UART_RESET);

    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void BOARD_ConfigMPU(void)
{
    uint8_t attr;

    /* Disable MPU */
    ARM_MPU_Disable();

    /* Attr0: device. */
    ARM_MPU_SetMemAttr(0U, ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE, 0U));

    /* Attr1: non cacheable. */
    ARM_MPU_SetMemAttr(1U, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));

    /* Attr2: non transient, write through, read allocate. */
    attr = ARM_MPU_ATTR_MEMORY_(0U, 0U, 1U, 0U);
    ARM_MPU_SetMemAttr(2U, ARM_MPU_ATTR(attr, attr));

    /* Attr3: non transient, write back, read/write allocate. */
    attr = ARM_MPU_ATTR_MEMORY_(0U, 1U, 1U, 1U);
    ARM_MPU_SetMemAttr(3U, ARM_MPU_ATTR(attr, attr));

    /* NOTE1: All SRAM is non-cacheable regardless of MPU setting. */
    /* NOTE2: [0x0, 0x1FFFFFFF] cannot support write back. */
    /* Region 0 (FlexSPI0): [0x0, 0x1FFFFFFF], non-shareable, read/write, any privileged, executable. Attr 2 (write
     * through). */
    ARM_MPU_SetRegion(0U, ARM_MPU_RBAR(0U, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x1FFFFFFFU, 2U));

    /* NOTE: All SRAM is non-cacheable regardless of MPU setting. */
    /* Region 1 (Peripherals): [0x20000000, 0x3FFFFFFF], non-shareable, read/write, any privileged, executable. Attr 0
     * (device). */
    ARM_MPU_SetRegion(1U, ARM_MPU_RBAR(0x20000000U, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x3FFFFFFF, 0U));

    /* Region 2 (FlexSPI1,2): [0x40000000, 0x7FFFFFFF], non-shareable, read/write, any privileged, executable. Attr 3
     * (write back). */
    ARM_MPU_SetRegion(2U, ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x7FFFFFFF, 3U));

    /* NOTE: DDR is used as shared memory for A/M core communication, set it to non-cacheable. */
    /* Region 3 (DDR): [0x80000000, 0xDFFFFFFF], outer shareable, read/write, any privileged, executable. Attr 1
     * (non-cacheable). */
    ARM_MPU_SetRegion(3U, ARM_MPU_RBAR(0x80000000, ARM_MPU_SH_OUTER, 0U, 1U, 0U), ARM_MPU_RLAR(0xDFFFFFFF, 1U));

    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Code Cache already enabled in ROM, here enable System Cache */
    CACHE64_EnableWriteBuffer(CACHE64_CTRL1, true);
    CACHE64_EnableCache(CACHE64_CTRL1);
}

static status_t flexspi_hyper_ram_write_mcr(FLEXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = regAddr;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = 3;
    flashXfer.data          = mrVal;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

static status_t flexspi_hyper_ram_get_mcr(FLEXSPI_Type *base, uint8_t regAddr, uint32_t *mrVal)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Read data */
    flashXfer.deviceAddress = regAddr;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = 2;
    flashXfer.data          = mrVal;
    flashXfer.dataSize      = 2;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

static status_t flexspi_hyper_ram_reset(FLEXSPI_Type *base)
{
    flexspi_transfer_t flashXfer;
    status_t status;

    /* Write data */
    flashXfer.deviceAddress = 0x0U;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = 4;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status == kStatus_Success)
    {
        /* for loop of 50000 is about 1ms (@200 MHz CPU) */
        for (uint32_t i = 2000000U; i > 0; i--)
        {
            __NOP();
        }
    }
    return status;
}

/* Initialize psram. */
status_t BOARD_InitPsRam(void)
{
    flexspi_device_config_t deviceconfig = {
        .flexspiRootClk       = 392000000, /* 392MHZ SPI serial clock, DDR serial clock 196M */
        .isSck2Enabled        = false,
        .flashSize            = 0x2000, /* 64Mb/KByte */
        .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
        .CSInterval           = 5,
        .CSHoldTime           = 3,
        .CSSetupTime          = 3,
        .dataValidTime        = 1,
        .columnspace          = 0,
        .enableWordAddress    = false,
        .AWRSeqIndex          = 1,
        .AWRSeqNumber         = 1,
        .ARDSeqIndex          = 0,
        .ARDSeqNumber         = 1,
        .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
        .AHBWriteWaitInterval = 0,
        .enableWriteMask      = true,
    };

    uint32_t customLUT[64] = {
        /* Read Data */
        [0] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x20, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
        [1] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07, kFLEXSPI_Command_READ_DDR,
                              kFLEXSPI_8PAD, 0x04),

        /* Write Data */
        [4] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0xA0, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
        [5] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07, kFLEXSPI_Command_WRITE_DDR,
                              kFLEXSPI_8PAD, 0x04),

        /* Read Register */
        [8] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0x40, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
        [9] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_RWDS_DDR, kFLEXSPI_8PAD, 0x07, kFLEXSPI_Command_READ_DDR,
                              kFLEXSPI_8PAD, 0x04),

        /* Write Register */
        [12] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0xC0, kFLEXSPI_Command_RADDR_DDR, kFLEXSPI_8PAD, 0x20),
        [13] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR, kFLEXSPI_8PAD, 0x08, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD,
                               0x00),

        /* reset */
        [16] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_8PAD, 0xFF, kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_8PAD, 0x03),

    };

    uint32_t mr0mr1[1];
    uint32_t mr4mr8[1];
    uint32_t mr0Val[1];
    uint32_t mr4Val[1];
    uint32_t mr8Val[1];
    flexspi_config_t config;
    status_t status = kStatus_Success;

    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_FLEXSPI1);

    /* 392MHz * 1U / 1U = 392MHz */
    CLOCK_SetIpSrcDiv(kCLOCK_Flexspi1, kCLOCK_Pcc1PlatIpSrcPll0Pfd3, 0U, 0U);
    RESET_PeripheralReset(kRESET_Flexspi1);

    /* Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /* Init FLEXSPI. */
    config.rxSampleClock = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    /*Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch    = true;
    config.ahbConfig.enableAHBBufferable  = true;
    config.ahbConfig.enableAHBCachable    = true;
    config.ahbConfig.enableReadAddressOpt = true;
    for (uint8_t i = 1; i < FSL_FEATURE_FLEXSPI_AHB_BUFFER_COUNT - 1; i++)
    {
        config.ahbConfig.buffer[i].bufferSize = 0;
    }
    /* FlexSPI1 has total 2KB RX buffer.
     * Set GPU/Display master to use AHB Rx Buffer0.
     */
    config.ahbConfig.buffer[0].masterIndex    = 2;    /* DMA0 */
    config.ahbConfig.buffer[0].bufferSize     = 1024; /* Allocate 1KB bytes for DMA0 */
    config.ahbConfig.buffer[0].enablePrefetch = true;
    config.ahbConfig.buffer[0].priority       = 7; /* Set DMA0 to highest priority. */
    /* All other masters use last buffer with 1KB bytes. */
    config.ahbConfig.buffer[FSL_FEATURE_FLEXSPI_AHB_BUFFER_COUNT - 1].bufferSize = 1024;
    config.enableCombination                                                     = true;
    FLEXSPI_Init(BOARD_FLEXSPI_PSRAM, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(BOARD_FLEXSPI_PSRAM, &deviceconfig, kFLEXSPI_PortA1);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(BOARD_FLEXSPI_PSRAM, 0, customLUT, ARRAY_SIZE(customLUT));

    /* Do software reset. */
    FLEXSPI_SoftwareReset(BOARD_FLEXSPI_PSRAM);

    /* Reset hyper ram. */
    status = flexspi_hyper_ram_reset(BOARD_FLEXSPI_PSRAM);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_hyper_ram_get_mcr(BOARD_FLEXSPI_PSRAM, 0x0, mr0mr1);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_hyper_ram_get_mcr(BOARD_FLEXSPI_PSRAM, 0x4, mr4mr8);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Enable RBX, burst length set to 1K. - MR8 */
    mr8Val[0] = (mr4mr8[0] & 0xFF00U) >> 8U;
    mr8Val[0] = mr8Val[0] | 0x0F;
    status    = flexspi_hyper_ram_write_mcr(BOARD_FLEXSPI_PSRAM, 0x8, mr8Val);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set LC code to 0x04(LC=7, maximum frequency 200M) - MR0. */
    mr0Val[0] = mr0mr1[0] & 0x00FFU;
    mr0Val[0] = (mr0Val[0] & ~0x3CU) | (4U << 2U);
    status    = flexspi_hyper_ram_write_mcr(BOARD_FLEXSPI_PSRAM, 0x0, mr0Val);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set WLC code to 0x01(WLC=7, maximum frequency 200M) - MR4. */
    mr4Val[0] = mr4mr8[0] & 0x00FFU;
    mr4Val[0] = (mr4Val[0] & ~0xE0U) | (1U << 5U);
    status    = flexspi_hyper_ram_write_mcr(BOARD_FLEXSPI_PSRAM, 0x4, mr4Val);
    if (status != kStatus_Success)
    {
        return status;
    }

    return status;
}

void BOARD_DeinitXip(FLEXSPI_Type *base)
{
    /* Wait until FLEXSPI is not busy */
    while (!((base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK) && (base->STS0 & FLEXSPI_STS0_SEQIDLE_MASK)))
    {
    }
    /* Disable module during the reset procedure */
    base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
}

void BOARD_InitXip(FLEXSPI_Type *base)
{
    uint32_t status;
    uint32_t lastStatus;
    uint32_t retry;

    /* Enable FLEXSPI module */
    base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

    base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
    while (base->MCR0 & FLEXSPI_MCR0_SWRESET_MASK)
    {
    }

    /* Need to wait DLL locked if DLL enabled */
    if (0U != (base->DLLCR[0] & FLEXSPI_DLLCR_DLLEN_MASK))
    {
        lastStatus = base->STS2;
        retry      = BOARD_FLEXSPI_DLL_LOCK_RETRY;
        /* Wait slave delay line locked and slave reference delay line locked. */
        do
        {
            status = base->STS2;
            if ((status & (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK)) ==
                (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK))
            {
                /* Locked */
                retry = 100;
                break;
            }
            else if (status == lastStatus)
            {
                /* Same delay cell number in calibration */
                retry--;
            }
            else
            {
                retry      = BOARD_FLEXSPI_DLL_LOCK_RETRY;
                lastStatus = status;
            }
        } while (retry > 0);
        /* According to ERR011377, need to delay at least 100 NOPs to ensure the DLL is locked. */
        for (; retry > 0U; retry--)
        {
            __NOP();
        }
    }
}

/* BOARD_SetFlexspiClock run in RAM used to configure FlexSPI clock source and divider when XIP. */
void BOARD_SetFlexspiClock(FLEXSPI_Type *base, uint32_t src, uint8_t divValue, uint8_t fracValue)
{
    uint32_t pccReg;

    if (base == FLEXSPI0)
    {
        pccReg = PCC_REG(kCLOCK_Flexspi0);
        if ((PCC_PCS_VAL(pccReg) != src) || PCC_PCD_VAL(pccReg) != divValue || PCC_FRAC_VAL(pccReg) != fracValue)
        {
            if (BOARD_IS_XIP_FLEXSPI0())
            {
                BOARD_DeinitXip(base);
            }
            pccReg &= ~(PCC_CLKCFG_PCD_MASK | PCC_CLKCFG_FRAC_MASK | PCC_CLKCFG_PCS_MASK);
            pccReg |= PCC_CLKCFG_PCD(divValue) | PCC_CLKCFG_FRAC(fracValue) | PCC_CLKCFG_PCS(src);
            /*
             * If clock is already enabled, first disable it, then set the clock
             * source and re-enable it.
             */
            PCC_REG(kCLOCK_Flexspi0) = pccReg & ~PCC_CLKCFG_CGC_MASK;
            PCC_REG(kCLOCK_Flexspi0) = pccReg;
            if (BOARD_IS_XIP_FLEXSPI0())
            {
                BOARD_InitXip(base);
            }
        }
    }
    else if (base == FLEXSPI1)
    {
        pccReg = PCC_REG(kCLOCK_Flexspi1);
        if ((PCC_PCS_VAL(pccReg) != src) || PCC_PCD_VAL(pccReg) != divValue || PCC_FRAC_VAL(pccReg) != fracValue)
        {
            /* FLEXSPI1 not for CM33 XIP. */
            pccReg &= ~(PCC_CLKCFG_PCD_MASK | PCC_CLKCFG_FRAC_MASK | PCC_CLKCFG_PCS_MASK);
            pccReg |= PCC_CLKCFG_PCD(divValue) | PCC_CLKCFG_FRAC(fracValue) | PCC_CLKCFG_PCS(src);
            /*
             * If clock is already enabled, first disable it, then set the clock
             * source and re-enable it.
             */
            PCC_REG(kCLOCK_Flexspi1) = pccReg & ~PCC_CLKCFG_CGC_MASK;
            PCC_REG(kCLOCK_Flexspi1) = pccReg;
        }
    }
    else
    {
        assert(false);
    }
}

/* This function is used to change FlexSPI clock to a stable source before clock sources(Such as PLL and Main clock)
 * updating in case XIP(execute code on FLEXSPI memory.) */
void BOARD_FlexspiClockSafeConfig(void)
{
    /* Move FLEXSPI clock source from main clock to FRO192M / 2 to avoid instruction/data fetch issue in XIP when
     * updating PLL and main clock.
     */
    BOARD_SetFlexspiClock(FLEXSPI0, 6U, 1U, 0U);
}

#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_LPI2C_Init(LPI2C_Type *base, uint32_t clkSrc_Hz)
{
    lpi2c_master_config_t lpi2cConfig = {0};

    /*
     * lpi2cConfig.debugEnable = false;
     * lpi2cConfig.ignoreAck = false;
     * lpi2cConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * lpi2cConfig.baudRate_Hz = 100000U;
     * lpi2cConfig.busIdleTimeout_ns = 0;
     * lpi2cConfig.pinLowTimeout_ns = 0;
     * lpi2cConfig.sdaGlitchFilterWidth_ns = 0;
     * lpi2cConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&lpi2cConfig);
    LPI2C_MasterInit(base, &lpi2cConfig, clkSrc_Hz);
}

status_t BOARD_LPI2C_Send(LPI2C_Type *base,
                          uint8_t deviceAddress,
                          uint32_t subAddress,
                          uint8_t subAddressSize,
                          uint8_t *txBuff,
                          uint8_t txBuffSize,
                          uint32_t flags)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = flags;
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = txBuff;
    xfer.dataSize       = txBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

status_t BOARD_LPI2C_Receive(LPI2C_Type *base,
                             uint8_t deviceAddress,
                             uint32_t subAddress,
                             uint8_t subAddressSize,
                             uint8_t *rxBuff,
                             uint8_t rxBuffSize,
                             uint32_t flags)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = flags;
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Read;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = rxBuff;
    xfer.dataSize       = rxBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

void BOARD_Accel_I2C_Init(void)
{
    BOARD_LPI2C_Init(BOARD_ACCEL_I2C_BASEADDR, BOARD_ACCEL_I2C_CLOCK_FREQ);
}

status_t BOARD_Accel_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff, uint32_t flags)
{
    uint8_t data = (uint8_t)txBuff;

    return BOARD_LPI2C_Send(BOARD_ACCEL_I2C_BASEADDR, deviceAddress, subAddress, subaddressSize, &data, 1, flags);
}

status_t BOARD_Accel_I2C_Receive(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subaddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize,
                                 uint32_t flags)
{
    return BOARD_LPI2C_Receive(BOARD_ACCEL_I2C_BASEADDR, deviceAddress, subAddress, subaddressSize, rxBuff, rxBuffSize,
                               flags);
}

void BOARD_Codec_I2C_Init(void)
{
    BOARD_LPI2C_Init(BOARD_CODEC_I2C_BASEADDR, BOARD_CODEC_I2C_CLOCK_FREQ);
}

status_t BOARD_Codec_I2C_Send(uint8_t deviceAddress,
                              uint32_t subAddress,
                              uint8_t subAddressSize,
                              const uint8_t *txBuff,
                              uint8_t txBuffSize,
                              uint32_t flags)
{
    return BOARD_LPI2C_Send(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
                            txBuffSize, flags);
}

status_t BOARD_Codec_I2C_Receive(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize,
                                 uint32_t flags)
{
    return BOARD_LPI2C_Receive(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize,
                               flags);
}

void BOARD_Display_I2C_Init(void)
{
    BOARD_LPI2C_Init(BOARD_DISPLAY_I2C_BASEADDR, BOARD_DISPLAY_I2C_CLOCK_FREQ);
}

status_t BOARD_Display_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
    return BOARD_LPI2C_Send(BOARD_DISPLAY_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
                            txBuffSize, 0);
}

status_t BOARD_Display_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    return BOARD_LPI2C_Receive(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize,
                               0);
}

#if defined(BOARD_USE_PCA6416A) && BOARD_USE_PCA6416A
pca6416a_handle_t g_pca6416aHandle;

void BOARD_PCA6416A_I2C_Init(void)
{
    BOARD_LPI2C_Init(BOARD_PCA6416A_I2C, BOARD_PCA6416A_I2C_CLOCK_FREQ);
}

status_t BOARD_PCA6416A_I2C_Send(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 const uint8_t *txBuff,
                                 uint8_t txBuffSize,
                                 uint32_t flags)
{
    return BOARD_LPI2C_Send(BOARD_PCA6416A_I2C, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
                            txBuffSize, flags);
}

status_t BOARD_PCA6416A_I2C_Receive(uint8_t deviceAddress,
                                    uint32_t subAddress,
                                    uint8_t subAddressSize,
                                    uint8_t *rxBuff,
                                    uint8_t rxBuffSize,
                                    uint32_t flags)
{
    return BOARD_LPI2C_Receive(BOARD_PCA6416A_I2C, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize,
                               flags);
}

void BOARD_InitPCA6416A(pca6416a_handle_t *handle)
{
    BOARD_PCA6416A_I2C_Init();

    static const pca6416a_config_t config = {
        .i2cAddr         = BOARD_PCA6416A_I2C_ADDR,
        .I2C_SendFunc    = BOARD_PCA6416A_I2C_Send,
        .I2C_ReceiveFunc = BOARD_PCA6416A_I2C_Receive,
    };

    PCA6416A_Init(handle, &config);
}

/* Set I2C0 PCA6416 IO9(PTH9_MIPI_SWITH) to high to select MIPI DSI panel path for uboot(running on a35) */
void BOARD_InitMipiDsiPins(void)
{
    BOARD_InitPCA6416A(&g_pca6416aHandle);
    PCA6416A_SetPins(&g_pca6416aHandle, (1U << BOARD_PCA6416A_MIPI_SWITCH));
    PCA6416A_SetDirection(&g_pca6416aHandle, (1U << BOARD_PCA6416A_MIPI_SWITCH), kPCA6416A_Output);
}
#endif /* BOARD_USE_PCA6416A. */

#if defined(BOARD_USE_TPM) && BOARD_USE_TPM
/* Set TPM0_CH2 to full duty cycle to enable backlight at highest brightness for uboot(running on a35) */
void BOARD_EnableMipiDsiBacklight(void)
{
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t pwmChannelConfig = {
        .chnlNumber       = (tpm_chnl_t)TPM0_CH2,
        .level            = kTPM_HighTrue,
        .dutyCyclePercent = FULL_DUTY_CYCLE,
    };

    TPM_GetDefaultConfig(&tpmInfo);
    TPM_Init(TPM0, (void *)&tpmInfo);
    TPM_SetupPwm(TPM0, (void *)&pwmChannelConfig, 1, kTPM_EdgeAlignedPwm, CLOCK_GetTpmClkFreq(0U), TPM0_CH2_PWM_FREQ);
    TPM_StartTimer(TPM0, kTPM_SystemClock);
}
#endif /* BOARD_USE_TPM. */
#endif /* SDK_I2C_BASED_COMPONENT_USED */
