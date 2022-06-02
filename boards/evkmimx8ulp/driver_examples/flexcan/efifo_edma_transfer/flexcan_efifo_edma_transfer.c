/*
 * Copyright 2021 NXP
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
#define TX_MESSAGE_BUFFER_NUM (0U)

#define EXAMPLE_CAN_DMA            (DMA0)
#define EXAMPLE_CAN_DMA_CHANNEL    0
#define FLEXCAN_DMA_REQUEST_SOURCE kDmaRequestMux0CAN

#define EXAMPLE_CAN_CLOCK_NAME   (kCLOCK_Flexcan)
#define EXAMPLE_CAN_CLOCK_SOURCE (kCLOCK_Pcc1BusIpSrcSysOscDiv2)
#define EXAMPLE_CAN_CLK_FREQ     (CLOCK_GetIpFreq(EXAMPLE_CAN_CLOCK_NAME))
/* Set USE_IMPROVED_TIMING_CONFIG macro to use api to calculates the improved CAN / CAN FD timing values. */
#define USE_IMPROVED_TIMING_CONFIG (1U)
#define DLC                        kFLEXCAN_64BperFrame
#define BYTES_IN_MB                kFLEXCAN_64BperMB
/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF
#ifndef RX_MESSAGE_COUNT
#define RX_MESSAGE_COUNT (4U)
#endif
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
flexcan_fd_frame_t txFrame;
AT_NONCACHEABLE_SECTION(flexcan_fd_frame_t rxFrame[RX_MESSAGE_COUNT]);
/* Config fifo filters to make it accept std frame with ID 0x123 ~ 0x 126. */
uint32_t rxEnFifoFilter[] = {FLEXCAN_ENHANCED_RX_FIFO_STD_MASK_AND_FILTER(0x123, 0, 0x3F, 0),
                             FLEXCAN_ENHANCED_RX_FIFO_STD_MASK_AND_FILTER(0x124, 0, 0x3F, 0),
                             FLEXCAN_ENHANCED_RX_FIFO_STD_MASK_AND_FILTER(0x125, 0, 0x3F, 0),
                             FLEXCAN_ENHANCED_RX_FIFO_STD_MASK_AND_FILTER(0x126, 0, 0x3F, 0)};

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
    flexcan_enhanced_rx_fifo_config_t rxEhFifoConfig;
#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
    uint8_t node_type;
#endif
    uint32_t i;

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

    LOG_INFO("FlexCAN Enhanced Rx FIFO edma example.\r\n");
#if (defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
    LOG_INFO("Loopback mode, Message buffer %d used for Tx, Enhanced Rx FIFO used for Rx.\r\n", TX_MESSAGE_BUFFER_NUM);
#else
    LOG_INFO("Board to board mode.\r\n");
    LOG_INFO("Node B Enhanced Rx FIFO used for Rx.\r\n");
    LOG_INFO("Node A Message buffer %d used for Tx.\r\n", TX_MESSAGE_BUFFER_NUM);

    do
    {
        LOG_INFO("Please select local node as A or B:\r\n");
        LOG_INFO("Note: Node B should start first.\r\n");
        LOG_INFO("Node:");
        node_type = GETCHAR();
        LOG_INFO("%c", node_type);
        LOG_INFO("\r\n");
    } while ((node_type != 'A') && (node_type != 'B') && (node_type != 'a') && (node_type != 'b'));
#endif

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

#if (defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
    flexcanConfig.enableLoopBack = true;
#endif

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
    if (FLEXCAN_FDCalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.baudRate, flexcanConfig.baudRateFD,
                                                EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
        LOG_INFO("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif

    FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);

#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
    if ((node_type == 'A') || (node_type == 'a') || (node_type == 'T') || (node_type == 't'))
    {
#endif
        /* Setup Tx Message Buffer. */
        FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
        /* Send messages through Tx Message Buffer. */
        txFrame.dataWord[0] = 0;
        txFrame.dataWord[1] = 0x55;
        txFrame.format      = (uint8_t)kFLEXCAN_FrameFormatStandard;
        txFrame.type        = (uint8_t)kFLEXCAN_FrameTypeData;
        txFrame.id          = FLEXCAN_ID_STD(0x123);
        txFrame.brs         = 1U;
        txFrame.length      = (uint8_t)DLC;
        txXfer.framefd      = &txFrame;
        txXfer.mbIdx        = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
    }
    else
    {
#endif
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

        /* Setup Enhanced Rx FIFO. */
        rxEhFifoConfig.idFilterTable     = rxEnFifoFilter;
        rxEhFifoConfig.idFilterPairNum   = sizeof(rxEnFifoFilter) / sizeof(rxEnFifoFilter[0]) / 2U;
        rxEhFifoConfig.extendIdFilterNum = 0;
        /*!< To reduce the complexity of DMA software configuration, the fifoWatermark value must be set to 1 when using
            DMA handling the Enhanced Rx FIFO. Because a DMA transfer cannot be dynamically changed, Number of words
           read per transfer (dmaPerReadLength) should be programmed so that the Enhanced Rx FIFO element can store the
           largest CAN message present on the CAN bus. */
        rxEhFifoConfig.fifoWatermark    = 0U;
        rxEhFifoConfig.dmaPerReadLength = kFLEXCAN_19WordPerRead;
        rxEhFifoConfig.priority         = kFLEXCAN_RxFifoPrioHigh;
        FLEXCAN_SetEnhancedRxFifoConfig(EXAMPLE_CAN, &rxEhFifoConfig, true);
        rxFifoXfer.framefd  = &rxFrame[0];
        rxFifoXfer.frameNum = RX_MESSAGE_COUNT;
        /* Create FlexCAN EDMA handle structure and set call back function. */
        FLEXCAN_TransferCreateHandleEDMA(EXAMPLE_CAN, &flexcanEdmaHandle, flexcan_dma_callback, NULL,
                                         &flexcanRxFifoEdmaHandle);
#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
        LOG_INFO("Start to Wait data from Node A.\r\n\r\n");
    }
#endif

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    while (true)
    {
#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
        if ((node_type == 'A') || (node_type == 'a'))
        {
#endif
            LOG_INFO("Press any key to trigger %d transmission.\r\n\r\n", RX_MESSAGE_COUNT);
            GETCHAR();
            for (i = 0; i < RX_MESSAGE_COUNT; i++)
            {
                txFrame.id = FLEXCAN_ID_STD(0x123U + i);
                (void)FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

                while (!txComplete)
                {
                };
                txComplete = false;
                LOG_INFO("Send Msg%d to Enhanced Rx FIFO: word0 = 0x%x, word1 = 0x%x, id = 0x%x.\r\n", i,
                         txFrame.dataWord[0], txFrame.dataWord[1], 0x123U + i);
                txFrame.dataWord[0]++;
            }
            LOG_INFO("\r\n");
#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
        }
        else
        {
#endif
            /* Receive data through Enhanced Rx FIFO. */
            if (FLEXCAN_TransferReceiveEnhancedFifoEDMA(EXAMPLE_CAN, &flexcanEdmaHandle, &rxFifoXfer) !=
                kStatus_Success)
            {
                LOG_INFO("Receive CAN message from Enhanced Rx FIFO failed!\r\n");
                return -1;
            }
            else
            {
                while (!rxComplete)
                {
                }
                rxComplete = false;
                for (i = 0; i < RX_MESSAGE_COUNT; i++)
                {
                    LOG_INFO(
                        "Receive Msg%d from Enhanced Rx FIFO: word0 = 0x%x, word1 = 0x%x, ID Filter Hit: %d, Time "
                        "stamp: %d.\r\n",
                        i, rxFrame[i].dataWord[0], rxFrame[i].dataWord[1], rxFrame[i].idhit, rxFrame[i].timestamp);
                }
                LOG_INFO("\r\n");
            }
#if !(defined(ENABLE_LOOPBACK) && ENABLE_LOOPBACK)
            LOG_INFO("Wait for the next %d messages!\r\n\r\n", RX_MESSAGE_COUNT);
        }
#endif
    }
}
