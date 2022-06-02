/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i3c.h"

#include "fsl_reset.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_MASTER             I3C0
#define EXAMPLE_I2C_BAUDRATE       400000
#define I3C_MASTER_CLOCK_FREQUENCY CLOCK_GetI3cClkFreq(0)
#define I3C_MASTER_SLAVE_ADDR_7BIT 0x6A
#define WAIT_TIME                  1000
#define I3C_DATA_LENGTH            1
#define LSM6DSO_WHOAMI 0x6c

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
    i3c_master_config_t masterConfig;
    i3c_master_transfer_t masterXfer;
    status_t result        = kStatus_Success;
    uint8_t who_am_i_value = 0x00;

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    CLOCK_SetIpSrc(kCLOCK_I3c0, kCLOCK_Pcc0BusIpSrcSysOscDiv2);
    RESET_PeripheralReset(kRESET_I3c0);

    PRINTF("\r\nI3C master read sensor data example.\r\n");

    PRINTF("\r\nStart to do I3C master transfer in I2C mode.\r\n");

    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud          = EXAMPLE_I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud  = 4000000U;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = 1500000U;
    masterConfig.enableOpenDrainStop          = false;
    I3C_MasterInit(EXAMPLE_MASTER, &masterConfig, I3C_MASTER_CLOCK_FREQUENCY);

    memset(&masterXfer, 0, sizeof(masterXfer));

    uint8_t deviceAddress     = 0x0FU;
    masterXfer.slaveAddress   = I3C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction      = kI3C_Read;
    masterXfer.busType        = kI3C_TypeI2C;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = &who_am_i_value;
    masterXfer.dataSize       = I3C_DATA_LENGTH;
    masterXfer.flags          = kI3C_TransferDefaultFlag;

    result = I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);
    if (result != kStatus_Success)
    {
        return -1;
    }

    if (who_am_i_value == LSM6DSO_WHOAMI)
    {
        PRINTF("\r\nSuccess to read WHO_AM_I register value from LSDM6DSO on board, the value is 0x%02X. \r\n",
               who_am_i_value);
    }

    while (1)
    {
    }
}
