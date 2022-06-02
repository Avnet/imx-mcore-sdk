/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_flexcan_edma.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#include "fsl_dmamux.h"
#endif
#include "fsl_reset.h"
#include "fsl_upower.h"
#include "fsl_trdc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_CAN           CAN0
#define TX_MESSAGE_BUFFER_NUM (9)

#define EXAMPLE_CAN_DMA            (DMA0)
#define EXAMPLE_CAN_DMA_CHANNEL    0
#define FLEXCAN_DMA_REQUEST_SOURCE kDmaRequestMux0CAN

#define EXAMPLE_CAN_CLOCK_NAME   (kCLOCK_Flexcan)
#define EXAMPLE_CAN_CLOCK_SOURCE (kCLOCK_Pcc1BusIpSrcSysOscDiv2)
#define EXAMPLE_CAN_CLK_FREQ     (CLOCK_GetIpFreq(EXAMPLE_CAN_CLOCK_NAME))
/* Set USE_IMPROVED_TIMING_CONFIG macro to use api to calculates the improved CAN / CAN FD timing values. */
#define USE_IMPROVED_TIMING_CONFIG (1U)
/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool txComplete = false;
volatile bool rxComplete = false;
flexcan_handle_t flexcanHandle;
flexcan_edma_handle_t flexcanEdmaHandle;
edma_handle_t flexcanRxFifoEdmaHandle;
flexcan_mb_transfer_t txXfer;
flexcan_fifo_transfer_t rxFifoXfer;
flexcan_frame_t txFrame;
AT_NONCACHEABLE_SECTION(flexcan_frame_t rxFrame);

/*******************************************************************************
 * Code
 ******************************************************************************/
static void APP_SetTrdcConfig(void)
{
    TRDC_Init(TRDC);

    /* Set DMA (master id is 2) bus attribute to privileged, otherwise it will cause bus errors when reading/writing
     * flexcan memory( flexcan in supervisor mode). */
    trdc_non_processor_domain_assignment_t pDomainAssignment;
    TRDC_GetDefaultNonProcessorDomainAssignment(&pDomainAssignment);
    pDomainAssignment.privilegeAttr = kTRDC_ForcePrivilege;
    TRDC_SetNonProcessorDomainAssignment(TRDC, 2U, &pDomainAssignment);
    TRDC_SetDacGlobalValid(TRDC);
}

/*!
 * @brief FlexCAN Call Back function
 */
static FLEXCAN_CALLBACK(flexcan_callback)
{
    /* Process FlexCAN Tx event. */
    if ((kStatus_FLEXCAN_TxIdle == status) && (TX_MESSAGE_BUFFER_NUM == result))
    {
        txComplete = true;
    }
}

/*!
 * @brief FlexCAN DMA Call Back function
 */
static void flexcan_dma_callback(CAN_Type *base, flexcan_edma_handle_t *handle, status_t status, void *userData)
{
    /* Process FlexCAN Rx event. */
    if (kStatus_FLEXCAN_RxFifoIdle == status)
    {
        rxComplete = true;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    edma_config_t edmaConfig;
    flexcan_config_t flexcanConfig;
    flexcan_rx_fifo_config_t rxFifoConfig;
    uint32_t rxFifoFilter[] = {
        FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x321, 0, 0), FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x321, 1, 0),
        FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x123, 0, 0), FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x123, 1, 0)};
    uint8_t i;

    /* Initialize board hardware. */
    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    APP_SetTrdcConfig();

    UPOWER_PowerOnMemPart(0U, (uint32_t)(kUPOWER_MP1_DMA0));
    CLOCK_EnableClock(kCLOCK_Dma0Ch0);

    CLOCK_SetIpSrc(EXAMPLE_CAN_CLOCK_NAME, EXAMPLE_CAN_CLOCK_SOURCE);
    RESET_PeripheralReset(kRESET_Flexcan);
    CLOCK_SetIpSrc(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcSysOscDiv2);
    RESET_PeripheralReset(kRESET_Lpi2c0);

    pca6416a_handle_t handle;
    BOARD_InitPCA6416A(&handle);
    PCA6416A_SetDirection(&handle, 1 << 4U, kPCA6416A_Output);
    PCA6416A_ClearPins(&handle, 1 << 4U);

    LOG_INFO("\r\n==FlexCAN loopback edma example -- Start.==\r\n\r\n");

    /* Init FlexCAN module. */
    /*
     * flexcanConfig.clkSrc                 = kFLEXCAN_ClkSrc0;
     * flexcanConfig.baudRate               = 1000000U;
     * flexcanConfig.baudRateFD             = 2000000U;
     * flexcanConfig.maxMbNum               = 16;
     * flexcanConfig.enableLoopBack         = false;
     * flexcanConfig.enableSelfWakeup       = false;
     * flexcanConfig.enableIndividMask      = false;
     * flexcanConfig.disableSelfReception   = false;
     * flexcanConfig.enableListenOnlyMode   = false;
     * flexcanConfig.enableDoze             = false;
     */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);

#if defined(EXAMPLE_CAN_CLK_SOURCE)
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

    flexcanConfig.enableLoopBack = true;

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
    if (FLEXCAN_CalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.baudRate, EXAMPLE_CAN_CLK_FREQ,
                                              &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
        LOG_INFO("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif

    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
    /* Configure DMA. */
    DMAMUX_Init(EXAMPLE_CAN_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_CAN_DMAMUX, EXAMPLE_CAN_DMA_CHANNEL, EXAMPLE_CAN_DMA_REQUEST);
    DMAMUX_EnableChannel(EXAMPLE_CAN_DMAMUX, EXAMPLE_CAN_DMA_CHANNEL);
#endif

    /*
     * edmaConfig.enableRoundRobinArbitration = false;
     * edmaConfig.enableHaltOnError = true;
     * edmaConfig.enableContinuousLinkMode = false;
     * edmaConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&edmaConfig);
    EDMA_Init(EXAMPLE_CAN_DMA, &edmaConfig);

    /* Create EDMA handle. */
    EDMA_CreateHandle(&flexcanRxFifoEdmaHandle, EXAMPLE_CAN_DMA, EXAMPLE_CAN_DMA_CHANNEL);
#if defined(FSL_FEATURE_EDMA_HAS_CHANNEL_MUX) && FSL_FEATURE_EDMA_HAS_CHANNEL_MUX
    EDMA_SetChannelMux(EXAMPLE_CAN_DMA, EXAMPLE_CAN_DMA_CHANNEL, FLEXCAN_DMA_REQUEST_SOURCE);
#endif

    /* Setup Tx Message Buffer. */
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    /* Setup Rx FIFO. */
    rxFifoConfig.idFilterTable = rxFifoFilter;
    rxFifoConfig.idFilterType  = kFLEXCAN_RxFifoFilterTypeA;
    rxFifoConfig.idFilterNum   = sizeof(rxFifoFilter) / sizeof(rxFifoFilter[0]);
    rxFifoConfig.priority      = kFLEXCAN_RxFifoPrioHigh;
    FLEXCAN_SetRxFifoConfig(EXAMPLE_CAN, &rxFifoConfig, true);

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Create FlexCAN EDMA handle structure and set call back function. */
    FLEXCAN_TransferCreateHandleEDMA(EXAMPLE_CAN, &flexcanEdmaHandle, flexcan_dma_callback, NULL,
                                     &flexcanRxFifoEdmaHandle);

    /* Send first message through Tx Message Buffer. */
    txFrame.format    = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type      = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.id        = FLEXCAN_ID_STD(0x123);
    txFrame.length    = 8U;
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x11) | CAN_WORD0_DATA_BYTE_2(0x11) |
                        CAN_WORD0_DATA_BYTE_3(0x11);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x11) | CAN_WORD1_DATA_BYTE_5(0x11) | CAN_WORD1_DATA_BYTE_6(0x11) |
                        CAN_WORD1_DATA_BYTE_7(0x11);

    txXfer.frame = &txFrame;
    txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
    (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
    while (!txComplete)
    {
    }
    txComplete = false;
    LOG_INFO("Send Msg1 to Rx FIFO: word0 = 0x%x, word1 = 0x%x.\r\n", txFrame.dataWord0, txFrame.dataWord1);

    /* Send second message through Tx Message Buffer. */
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x22) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x22) |
                        CAN_WORD0_DATA_BYTE_3(0x22);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x22) | CAN_WORD1_DATA_BYTE_5(0x22) | CAN_WORD1_DATA_BYTE_6(0x22) |
                        CAN_WORD1_DATA_BYTE_7(0x22);
    (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
    while (!txComplete)
    {
    }
    txComplete = false;
    LOG_INFO("Send Msg2 to Rx FIFO: word0 = 0x%x, word1 = 0x%x.\r\n", txFrame.dataWord0, txFrame.dataWord1);

    /* Send third message through Tx Message Buffer. */
    txXfer.frame      = &txFrame;
    txXfer.mbIdx      = (uint8_t)TX_MESSAGE_BUFFER_NUM;
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x33) | CAN_WORD0_DATA_BYTE_1(0x33) | CAN_WORD0_DATA_BYTE_2(0x33) |
                        CAN_WORD0_DATA_BYTE_3(0x33);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x33) | CAN_WORD1_DATA_BYTE_5(0x33) | CAN_WORD1_DATA_BYTE_6(0x33) |
                        CAN_WORD1_DATA_BYTE_7(0x33);
    (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
    while (!txComplete)
    {
    }
    txComplete = false;
    LOG_INFO("Send Msg3 to Rx FIFO: word0 = 0x%x, word1 = 0x%x.\r\n", txFrame.dataWord0, txFrame.dataWord1);

    /* Send fourth message through Tx Message Buffer. */
    txXfer.frame      = &txFrame;
    txXfer.mbIdx      = (uint8_t)TX_MESSAGE_BUFFER_NUM;
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x44) | CAN_WORD0_DATA_BYTE_1(0x44) | CAN_WORD0_DATA_BYTE_2(0x44) |
                        CAN_WORD0_DATA_BYTE_3(0x44);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x44) | CAN_WORD1_DATA_BYTE_5(0x44) | CAN_WORD1_DATA_BYTE_6(0x44) |
                        CAN_WORD1_DATA_BYTE_7(0x44);
    (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
    while (!txComplete)
    {
    }
    txComplete = false;
    LOG_INFO("Send Msg4 to Rx FIFO: word0 = 0x%x, word1 = 0x%x.\r\n\r\n", txFrame.dataWord0, txFrame.dataWord1);

    /* Receive data through Rx FIFO. */
    rxFifoXfer.frame = &rxFrame;
    for (i = 0; i < 4; i++)
    {
        (void)FLEXCAN_TransferReceiveFifoEDMA(EXAMPLE_CAN, &flexcanEdmaHandle, &rxFifoXfer);
        while (!rxComplete)
        {
        }
        rxComplete = false;

        LOG_INFO("Receive Msg%d from FIFO: word0 = 0x%x, word1 = 0x%x, ID Filter Hit%d.\r\n", i + 1, rxFrame.dataWord0,
                 rxFrame.dataWord1, rxFrame.idhit);
    }

    LOG_INFO("\r\n==FlexCAN loopback EDMA example -- Finish.==\r\n");

    while (true)
    {
        __WFI();
    }
}
