/*
 * Copyright 2020-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_sentinel.h"

#ifdef FSL_RTOS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.sentinel"
#endif

#define SENTINEL_CMD_HEADER(cmd, size) ((0x17U << 24U) | ((cmd) << 16U) | ((size) << 8U) | 0x6U)

/* TODO: Remove it and use the definition in SoC header file. */
#define SENTINEL_BASE (0x27000000UL)
#define SENTINEL_MU   ((MU_Type *)(SENTINEL_BASE + 0x40000U))

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Initialize MU interface for Sentinel access.
 */
void SENTINEL_Init(void)
{
    SENTINEL_MU->CIER0 = 0U;    /* Disable all module interrupts. */
    SENTINEL_MU->CSSR0 = 0xFFU; /* Clear all sticky pending status. */
    SENTINEL_MU->FCR   = 0U;    /* Clear all flags. */
    SENTINEL_MU->GIER  = 0U;    /* Disable all General interrupts. */
    SENTINEL_MU->GCR   = 0U;    /* Clear all general interrupt requests. */
    SENTINEL_MU->GSR   = 0xFU;  /* Clear all general interrupt status. */
    SENTINEL_MU->TCR   = 0U;    /* Disable all transmit interrupts. */
    SENTINEL_MU->RCR   = 0U;    /* Disable all receive interrupts. */
}

/*!
 * @brief Deinitialize MU interface for Sentinel access.
 */
void SENTINEL_Deinit(void)
{
}

/*!
 * @brief Send message to Sentinel.
 *
 * @param cmd Command to send to Sentinel.
 * @param param Command parameters pointer. Each parameter is a 32bit word.
 * @param paramCount Command parameter count.
 */
void SENTINEL_SendMessage(uint8_t cmd, uint32_t *param, uint32_t paramCount)
{
    uint32_t word = SENTINEL_CMD_HEADER(cmd, paramCount + 1U);
    uint32_t i;

    /* First send command header */
    while ((SENTINEL_MU->TSR & MU_TSR_TE0_MASK) == 0U)
    {
        /* Wait TR empty */
    }
    SENTINEL_MU->TR[0] = word;

    for (i = 1U; param && paramCount > 0U; paramCount--, i = (i + 1U) % 4U)
    {
        word = *param++;
        while ((SENTINEL_MU->TSR & (uint32_t)(MU_TSR_TE0_MASK << i)) == 0U)
        {
            /* Wait transmit register empty */
        }
        SENTINEL_MU->TR[i] = word;
    }
}

/*!
 * @brief Receive message from Sentinel.
 *
 * @param pCmd Pointer to save received command from Sentinel.
 * @param pParam Pointer to save command parameters. Each parameter is a 32bit word.
 * @param pParamCount Inout pointer to command parameter count.
 */
void SENTINEL_ReceiveMessage(uint8_t *pCmd, uint32_t *pParam, uint32_t *pParamCount)
{
    uint32_t word;
    uint32_t msgSize;
    uint32_t i;
    uint32_t paramCount = pParamCount ? *pParamCount : 0U;

    while ((SENTINEL_MU->RSR & MU_RSR_RF0_MASK) == 0U)
    {
        /* Wait receive register full */
    }

    word = SENTINEL_MU->RR[0];

    if (pCmd)
    {
        *pCmd = (word >> 16U) & 0xFFU;
    }
    else
    {
    }

    msgSize = (word >> 8U) & 0xFFU;
    assert(msgSize >= 1U);

    for (i = 1U, msgSize--; msgSize > 0U; msgSize--, i = (i + 1U) % 4U)
    {
        while ((SENTINEL_MU->RSR & (uint32_t)(MU_RSR_RF0_MASK << i)) == 0U)
        {
            /* Wait receive register full */
        }

        word = SENTINEL_MU->RR[i];
        if (pParam && paramCount)
        {
            *pParam++ = word;
            paramCount--;
        }
        else
        {
        }
    }

    if (pParamCount)
    {
        /* Save received param count. */
        *pParamCount -= paramCount;
    }
    else
    {
    }
}

/*!
 * @brief Send command to Sentinel and receive response.
 *
 * @param cmd Command to send to Sentinel.
 * @param cmdParam Command parameters pointer. Each parameter is a 32bit word.
 * @param cmdParamCount Command parameter count.
 * @param pRespParam Pointer to save response parameters. Each parameter is a 32bit word.
 * @param pRespParamCount Inout pointer to response parameter count.
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_Command(
    uint8_t cmd, uint32_t *cmdParam, uint32_t cmdParamCount, uint32_t *pRespParam, uint32_t *pRespParamCount)
{
    uint8_t respCmd;
    uint32_t respCode    = 0U;
    uint32_t respCodeLen = 1U;

#ifdef FSL_RTOS_FREE_RTOS
    /* Suspends the scheduler to make sure there's only one rpc call ongoing at a time. */
    vTaskSuspendAll();
#endif

    SENTINEL_SendMessage(cmd, cmdParam, cmdParamCount);

    if (pRespParam && pRespParamCount && *pRespParamCount > 0U)
    {
        SENTINEL_ReceiveMessage(&respCmd, pRespParam, pRespParamCount);
        respCode = *pRespParam;
    }
    else
    {
        SENTINEL_ReceiveMessage(&respCmd, &respCode, &respCodeLen);
    }
    assert(cmd == respCmd);
    assert(respCodeLen == 1U);

#ifdef FSL_RTOS_FREE_RTOS
    (void)xTaskResumeAll();
#endif

    return respCode;
}

/*!
 * @brief Ping Sentinel to see if it is alive.
 *
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_Ping(void)
{
    return SENTINEL_Command(SENTINEL_CMD_PING, NULL, 0U, NULL, NULL);
}

/*!
 * @brief Get Sentinel firmware version.
 *
 * @param pVersion Pointer to save firmware version.
 * @param pCommitSha Pointer to save first 4 bytes of the git commit ID.
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_GetFirmwareVersion(uint32_t *pVersion, uint32_t *pCommitSha)
{
    uint32_t respParam[3];
    uint32_t respParamCount = ARRAY_SIZE(respParam);

    SENTINEL_Command(SENTINEL_CMD_GET_FW_VERSION, NULL, 0U, respParam, &respParamCount);
    assert(respParamCount > 0U);

    if (respParam[0] == 0U) /* Successful */
    {
        assert(respParamCount == ARRAY_SIZE(respParam));
        if (pVersion)
        {
            *pVersion = respParam[1];
        }
        else
        {
        }
        if (pCommitSha)
        {
            *pCommitSha = respParam[2];
        }
        else
        {
        }
    }
    else
    {
    }

    return respParam[0];
}

/*!
 * @brief Get Sentinel firmware status.
 *
 * @param pStatus Pointer to save firmware status. 0: No firmware in place. 1: Firmware authenticated and operational.
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_GetFirmwareStatus(uint8_t *pStatus)
{
    uint32_t respParam[2];
    uint32_t respParamCount = ARRAY_SIZE(respParam);

    SENTINEL_Command(SENTINEL_CMD_GET_FW_STATUS, NULL, 0U, respParam, &respParamCount);
    assert(respParamCount > 0);

    if (respParam[0] == 0U) /* Successful */
    {
        assert(respParamCount == ARRAY_SIZE(respParam));
        if (pStatus)
        {
            *pStatus = respParam[1] & 0xFFU;
        }
        else
        {
        }
    }
    else
    {
    }

    return respParam[0];
}

/*!
 * @brief Request Sentinel to enter power down.
 *
 * @param preserveContext If save TRDC and XRTC states before power down.
 * @param trdcBufAddr Base address in memory for TRDC context preserve
 * @param trdcBufSize Length of buffer in memory for TRDC context preserve
 * @param xrdcBufAddr Base address in memory for XRDC context preserve
 * @param xrdcBufSize Length of buffer in memory for XRDC context preserve
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_SetPowerDown(
    bool preserveContext, uint32_t trdcBufAddr, uint32_t trdcBufSize, uint32_t xrdcBufAddr, uint32_t xrdcBufSize)
{
    uint32_t cmdParam[3];
    uint32_t respParam[1];
    uint32_t respParamCount = ARRAY_SIZE(respParam);

    if (preserveContext)
    {
        cmdParam[0] = trdcBufAddr;
        cmdParam[1] = xrdcBufAddr;
        cmdParam[2] = (trdcBufSize << 16) | xrdcBufSize;
    }
    else
    {
        cmdParam[0] = 0;
        cmdParam[1] = 0;
        cmdParam[2] = 0;
    }

    SENTINEL_Command(SENTINEL_CMD_POWER_DOWN, cmdParam, 3U, respParam, &respParamCount);
    assert(respParamCount > 0);

    if (respParam[0] == 0U) /* Successful */
    {
        assert(respParamCount == ARRAY_SIZE(respParam));
    }
    else
    {
    }

    return respParam[0];
}
