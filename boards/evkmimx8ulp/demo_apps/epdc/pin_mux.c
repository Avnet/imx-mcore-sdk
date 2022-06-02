/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v6.0
processor: MCIMX7U5xxxxx
package_id: MCIMX7U5CVP05
mcu_data: ksdk2_0
processor_version: 6.0.1
board: MCIMX7ULP-EVK-REV-B
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitDebugConsolePins();
    BOARD_InitPCA6414I2CPins();
    BOARD_InitEpdcPins();
    BOARD_InitEpdcI2CPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: cm4}
- pin_list:
  - {pin_num: AB19, peripheral: LPUART0, signal: lpuart_rx, pin_signal: PTA19, PE: PE_1_pull_enabled, PS: PS_1_pull_up}
  - {pin_num: AC19, peripheral: LPUART0, signal: lpuart_tx, pin_signal: PTA18, PE: PE_1_pull_enabled, PS: PS_1_pull_up}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitDebugConsolePins(void)
{ /*!< Function assigned for the core: Cortex-M4[cm4] */
    IOMUXC_SetPinMux(BOARD_UART1_TX_PIN_FUNCTION_ID, 0U);
    IOMUXC_SetPinConfig(BOARD_UART1_TX_PIN_FUNCTION_ID, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(BOARD_UART1_RX_PIN_FUNCTION_ID, 0U);
    IOMUXC_SetPinConfig(BOARD_UART1_RX_PIN_FUNCTION_ID, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
}

void BOARD_InitPCA6414I2CPins(void)
{ /*!< Function assigned for the core: Cortex-M4[cm4] */
    IOMUXC_SetPinMux(IOMUXC_PTA8_LPI2C0_SCL, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTA9_LPI2C0_SDA, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA8_LPI2C0_SCL, IOMUXC_PCR_ODE_MASK);
    IOMUXC_SetPinConfig(IOMUXC_PTA9_LPI2C0_SDA, IOMUXC_PCR_ODE_MASK);
}

void BOARD_InitEpdcPins(void)
{
    IOMUXC_SetPinMux(IOMUXC_PTF23_EPDC0_D0, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF22_EPDC0_D1, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF21_EPDC0_D2, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF20_EPDC0_D3, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF19_EPDC0_D4, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF18_EPDC0_D5, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF17_EPDC0_D6, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF16_EPDC0_D7, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF24_EPDC0_SDCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF25_EPDC0_GDSP, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF26_EPDC0_SDLE, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF27_EPDC0_SDCE0, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF30_EPDC0_SDCE2, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTF0_EPDC0_SDOE, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTE19_EPDC0_GDCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTE20_EPDC0_GDOE, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTE21_EPDC0_GDRL, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTE17_EPDC0_PWRSTAT, 0U);

    /*
     * Here use EPDC register EPDC_GPIO to control PWRWAKE pin.
     */
    IOMUXC_SetPinMux(IOMUXC_PTE18_EPDC0_PWRWAKE, 0U);

    /* Panel front light */
    IOMUXC_SetPinMux(IOMUXC_PTA3_PTA3, 0U);
}

void BOARD_InitEpdcI2CPins(void)
{ /*!< Function assigned for the core: Cortex-M4[cm4] */
    IOMUXC_SetPinMux(IOMUXC_PTA12_LPI2C1_SCL, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTA13_LPI2C1_SDA, 0U);

    IOMUXC_SetPinConfig(IOMUXC_PTA12_LPI2C1_SCL, IOMUXC_PCR_ODE_MASK);
    IOMUXC_SetPinConfig(IOMUXC_PTA13_LPI2C1_SDA, IOMUXC_PCR_ODE_MASK);
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
