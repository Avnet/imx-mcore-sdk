/*
 * Copyright 2021 NXP
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
product: Pins v10.0
processor: MIMX8UD7xxx10
package_id: MIMX8UD7DVP10
mcu_data: ksdk2_0
processor_version: 0.0.0
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
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33}
- pin_list:
  - {pin_num: AH2, peripheral: LPUART1, signal: lpuart_tx, pin_signal: PTA10, PS: UP, PE: ENABLED, SRE: STANDARD, ODE: PUSH_PULL, DSE: STANDARD, LK: UNLOCK, IBE: DISABLED,
    OBE: DISABLED, DFE: DISABLED, DFCS: IPGCLK, DFD: ZERO, INV: NotInvert}
  - {pin_num: AH3, peripheral: LPUART1, signal: lpuart_rx, pin_signal: PTA11, PS: UP, PE: ENABLED}
  - {pin_num: AH4, peripheral: LPSPI1, signal: 'lpspi_pcs, 0', pin_signal: PTA15}
  - {pin_num: AH6, peripheral: LPSPI1, signal: lpspi_sin, pin_signal: PTA16}
  - {pin_num: AD8, peripheral: LPSPI1, signal: lpspi_sout, pin_signal: PTA17}
  - {pin_num: AF6, peripheral: LPSPI1, signal: lpspi_sck, pin_signal: PTA18}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {                                /*!< Function assigned for the core: Cortex-M33[cm33] */
    IOMUXC_SetPinMux(IOMUXC_PTA10_LPUART1_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA10_LPUART1_TX,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA11_LPUART1_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA11_LPUART1_RX,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA15_LPSPI1_PCS0, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTA16_LPSPI1_SIN, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTA17_LPSPI1_SOUT, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTA18_LPSPI1_SCK, 0U);
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
