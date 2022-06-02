/*
 * Copyright 2020-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_SENTINEL_H_
#define _FSL_SENTINEL_H_

#include "fsl_common.h"

/*!
 * @addtogroup sentinel
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief sentinel command definition. */
#define SENTINEL_CMD_PING              (0x01U)
#define SENTINEL_CMD_DUMP_DEBUG_BUFFER (0x21U)
#define SENTINEL_CMD_OEM_CNTN_AUTH     (0x87U)
#define SENTINEL_CMD_VERIFY_IMAGE      (0x88U)
#define SENTINEL_CMD_RELEASE_CONTAINER (0x89U)
#define SENTINEL_CMD_GET_FW_VERSION    (0x9DU)
#define SENTINEL_CMD_GET_FW_STATUS     (0xC5U)
#define SENTINEL_CMD_POWER_DOWN        (0xD1U)

/*! @name Driver version */
/*@{*/
/*! @brief sentinel driver version 2.0.1. */
#define FSL_SENTINEL_DRIVER_VERSION (MAKE_VERSION(2, 0, 1))
/*@}*/

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /*__cplusplus */

/*!
 * @brief Initialize MU interface for Sentinel access.
 */
void SENTINEL_Init(void);

/*!
 * @brief Deinitialize MU interface for Sentinel access.
 */
void SENTINEL_Deinit(void);

/*!
 * @brief Send message to Sentinel.
 *
 * @param cmd Command to send to Sentinel.
 * @param param Command parameters pointer. Each parameter is a 32bit word.
 * @param paramCount Command parameter count.
 */
void SENTINEL_SendMessage(uint8_t cmd, uint32_t *param, uint32_t paramCount);

/*!
 * @brief Receive message from Sentinel.
 *
 * @param pCmd Pointer to save received command from Sentinel.
 * @param pParam Pointer to save command parameters. Each parameter is a 32bit word.
 * @param pParamCount Inout pointer to command parameter count.
 */
void SENTINEL_ReceiveMessage(uint8_t *pCmd, uint32_t *pParam, uint32_t *pParamCount);

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
    uint8_t cmd, uint32_t *cmdParam, uint32_t cmdParamCount, uint32_t *pRespParam, uint32_t *pRespParamCount);

/*!
 * @brief Ping Sentinel to see if it is alive.
 *
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_Ping(void);

/*!
 * @brief Get Sentinel firmware version.
 *
 * @param pVersion Pointer to save firmware version.
 * @param pCommitSha Pointer to save first 4 bytes of the git commit ID.
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_GetFirmwareVersion(uint32_t *pVersion, uint32_t *pCommitSha);

/*!
 * @brief Get Sentinel firmware status.
 *
 * @param pStatus Pointer to save firmware status. 0: No firmware in place. 1: Firmware authenticated and operational.
 * @return 0 for success, otherwise return error code.
 */
uint32_t SENTINEL_GetFirmwareStatus(uint8_t *pStatus);

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
    bool preserveContext, uint32_t trdcBufAddr, uint32_t trdcBufSize, uint32_t xrdcBufAddr, uint32_t xrdcBufSize);

#if defined(__cplusplus)
}
#endif /*__cplusplus */

/*! @} */

#endif /* _FSL_SENTINEL_H_ */
