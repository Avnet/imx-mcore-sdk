/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _POWER_MODE_SWITCH_H_
#define _POWER_MODE_SWITCH_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Power mode definition used in application. */
typedef enum _app_power_mode
{
    kAPP_PowerModeRun = 'A',     /* Normal RUN mode */
    kAPP_PowerModeWait,          /* WAIT mode. */
    kAPP_PowerModeStop,          /* STOP mode. */
    kAPP_PowerModeSleep,         /* Sleep mode. */
    kAPP_PowerModeDeepSleep,     /* Deep Sleep mode. */
    kAPP_PowerModePowerDown,     /* Power Down mode. */
    kAPP_PowerModeDeepPowerDown, /* Deep Power Down mode */
} app_power_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__cplusplus)
}
#endif

#endif /* _POWER_MODE_SWITCH_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
