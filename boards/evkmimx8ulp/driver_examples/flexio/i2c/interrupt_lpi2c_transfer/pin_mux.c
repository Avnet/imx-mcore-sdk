/*
 * Copyright 2021 NXP.
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
product: Pins v8.0
processor: MCIMX7U5xxxxx
package_id: MCIMX7U5DVP07
mcu_data: ksdk2_0
processor_version: 8.0.1
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
    BOARD_InitPins();
    BOARD_I2C_ConfigurePins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'false', prefix: BOARD_, coreID: cm4}
- pin_list:
  - {pin_num: AB19, peripheral: LPUART0, signal: lpuart_rx, pin_signal: PTA19, PE: PE_1_pull_enabled, PS: PS_1_pull_up}
  - {pin_num: AC19, peripheral: LPUART0, signal: lpuart_tx, pin_signal: PTA18, PE: PE_1_pull_enabled, PS: PS_1_pull_up}
  - {pin_num: AG18, peripheral: FXIO0, signal: 'fxio_d, 0', pin_signal: PTA16, ODE: ODE_0_Push_pull}
  - {pin_num: AD19, peripheral: FXIO0, signal: 'fxio_d, 1', pin_signal: PTA17, ODE: ODE_0_Push_pull}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {                                /*!< Function assigned for the core: Cortex-M4[cm4] */
    IOMUXC_SetPinMux(IOMUXC_PTB0_FXIO0_D16, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB0_FXIO0_D16,
                        IOMUXC_PCR_PS_MASK |
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_SRE_MASK |
                        IOMUXC_PCR_DSE_MASK |
                        IOMUXC_PCR_ODE_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTB1_FXIO0_D17, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB1_FXIO0_D17,
                        IOMUXC_PCR_PS_MASK |
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_SRE_MASK |
                        IOMUXC_PCR_DSE_MASK |
                        IOMUXC_PCR_ODE_MASK);

    IOMUXC_SetPinMux(BOARD_UART1_TX_PIN_FUNCTION_ID, 0U);
    IOMUXC_SetPinConfig(BOARD_UART1_TX_PIN_FUNCTION_ID,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(BOARD_UART1_RX_PIN_FUNCTION_ID, 0U);
    IOMUXC_SetPinConfig(BOARD_UART1_RX_PIN_FUNCTION_ID,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
}


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_I2C_ConfigurePins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: cm4}
- pin_list:
  - {pin_num: AC8, peripheral: LPI2C3, signal: lpi2c_scl, pin_signal: PTB12, ODE: ODE_1_Open_drain}
  - {pin_num: AD8, peripheral: LPI2C3, signal: lpi2c_sda, pin_signal: PTB13, ODE: ODE_1_Open_drain}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_I2C_ConfigurePins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_I2C_ConfigurePins(void) {                       /*!< Function assigned for the core: Cortex-M4[cm4] */
    IOMUXC_SetPinMux(IOMUXC_PTA8_LPI2C0_SCL, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA8_LPI2C0_SCL,
                        IOMUXC_PCR_PS_MASK |
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_SRE_MASK |
                        IOMUXC_PCR_DSE_MASK |
                        IOMUXC_PCR_ODE_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA9_LPI2C0_SDA, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA9_LPI2C0_SDA,
                        IOMUXC_PCR_PS_MASK |
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_SRE_MASK |
                        IOMUXC_PCR_DSE_MASK |
                        IOMUXC_PCR_ODE_MASK);
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
