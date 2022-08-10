/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_rgpio.h"
#if defined(BOARD_USE_PCA6416A) && BOARD_USE_PCA6416A
#include "fsl_pca6416a.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME        "MIMX8ULP-EVK"
#define MANUFACTURER_NAME "NXP"

/* The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_BAUDRATE 115200u
#define BOARD_DEBUG_UART_BASEADDR LPUART1_BASE
#define BOARD_DEBUG_UART_INSTANCE 1U
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetLpuartClkFreq(BOARD_DEBUG_UART_INSTANCE)
#define BOARD_DEBUG_UART_IP_NAME  kCLOCK_Lpuart1
#define BOARD_DEBUG_UART_CLKSRC   kCLOCK_Pcc1BusIpSrcCm33Bus
#define BOARD_DEBUG_UART_RESET    kRESET_Lpuart1
#define BOARD_UART_IRQ            LPUART1_IRQn
#define BOARD_UART_IRQ_HANDLER    LPUART1_IRQHandler

/* Board accelerator sensor configuration */
#define BOARD_ACCEL_I2C_BASEADDR   LPI2C0
#define BOARD_ACCEL_I2C_CLOCK_FREQ CLOCK_GetLpi2cClkFreq(0)

#define BOARD_CODEC_I2C_BASEADDR   LPI2C0
#define BOARD_CODEC_I2C_CLOCK_FREQ CLOCK_GetLpi2cClkFreq(0)
#define BOARD_CODEC_I2C_INSTANCE   0U

/* Board mipi to hdmi bridge ic(IT6161) */
#define BOARD_DISPLAY_I2C_BASEADDR   LPI2C0
#define BOARD_DISPLAY_I2C_CLOCK_FREQ CLOCK_GetLpi2cClkFreq(0)

#define BOARD_FLEXSPI_PSRAM FLEXSPI1

#define BOARD_SW8_GPIO        GPIOB
#define BOARD_SW8_GPIO_PIN    12U
#define BOARD_SW8_IRQ         GPIOB_INT0_IRQn
#define BOARD_SW8_IRQ_HANDLER GPIOB_INT0_IRQHandler
#define BOARD_SW8_NAME        "SW8"

#define BOARD_SW7_GPIO        GPIOB
#define BOARD_SW7_GPIO_PIN    13U
#define BOARD_SW7_IRQ         GPIOB_INT0_IRQn
#define BOARD_SW7_IRQ_HANDLER GPIOB_INT0_IRQHandler
#define BOARD_SW7_NAME        "SW7"

#define LED_INIT()
#define LED_ON()
#define LED_TOGGLE()

#define VDEV0_VRING_BASE (0xAFF00000U)
#define VDEV1_VRING_BASE (0xAFF10000U)

/* MIPI panel. */
/* RST pin. */
#define BOARD_MIPI_RST_GPIO GPIOC
#define BOARD_MIPI_RST_PIN  23
/* Backlight pin. */
#define BOARD_MIPI_BL_GPIO GPIOA
#define BOARD_MIPI_BL_PIN  3
/* TE pin. */
#define BOARD_MIPI_TE_GPIO GPIOE
#define BOARD_MIPI_TE_PIN  17

#define BOARD_IS_XIP_FLEXSPI0()                                                                                 \
    ((((uint32_t)BOARD_InitDebugConsole >= 0x04000000U) && ((uint32_t)BOARD_InitDebugConsole < 0x0C000000U)) || \
     (((uint32_t)BOARD_InitDebugConsole >= 0x14000000U) && ((uint32_t)BOARD_InitDebugConsole < 0x1C000000U)))

/* PCA6416A (U27) */
#define BOARD_PCA6416A_I2C            LPI2C0
#define BOARD_PCA6416A_I2C_ADDR       (0x20U)
#define BOARD_PCA6416A_I2C_CLOCK_FREQ (CLOCK_GetLpi2cClkFreq(0))

#define BOARD_PCA6416A_CPU_POWER_MODE0 0U
#define BOARD_PCA6416A_CPU_POWER_MODE1 1U
#define BOARD_PCA6416A_CPU_POWER_MODE2 2U
#define BOARD_PCA6416A_CAN0_STDBY      4U
#define BOARD_PCA6416A_TOUCH_RESET     5U
#define BOARD_PCA6416A_CAMERA_RESET    7U
#define BOARD_PCA6416A_CAMERA_PWDN     (8U + 0U)
#define BOARD_PCA6416A_MIPI_SWITCH     (8U + 1U)
#define BOARD_PCA6416A_EPDC_SWITCH     (8U + 2U)
#define BOARD_PCA6416A_BATT_ADC_ENABLE (8U + 3U)
#define BOARD_PCA6416A_CHG_OK          (8U + 4U)
#define BOARD_PCA6416A_AC_OK           (8U + 5U)

#define TPM0_CH2 (2UL)
/* 500 Hz */
#define TPM0_CH2_PWM_FREQ (500UL)
#define FULL_DUTY_CYCLE   (100UL)

/* IT6161(U10) */
#define BOARD_IT6161_I2C            LPI2C0
#define BOARD_IT6161_I2C_ADDR       (0x6CU)
#define BOARD_IT6161_I2C_CLOCK_FREQ (CLOCK_GetLpi2cClkFreq(0))

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);
void BOARD_ConfigMPU(void);
status_t BOARD_InitPsRam(void);
void BOARD_FlexspiClockSafeConfig(void);
AT_QUICKACCESS_SECTION_CODE(
    void BOARD_SetFlexspiClock(FLEXSPI_Type *base, uint32_t src, uint8_t divValue, uint8_t fracValue));
AT_QUICKACCESS_SECTION_CODE(void BOARD_DeinitXip(FLEXSPI_Type *base));
AT_QUICKACCESS_SECTION_CODE(void BOARD_InitXip(FLEXSPI_Type *base));
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_LPI2C_Init(LPI2C_Type *base, uint32_t clkSrc_Hz);
status_t BOARD_LPI2C_Send(LPI2C_Type *base,
                          uint8_t deviceAddress,
                          uint32_t subAddress,
                          uint8_t subaddressSize,
                          uint8_t *txBuff,
                          uint8_t txBuffSize,
                          uint32_t flags);
status_t BOARD_LPI2C_Receive(LPI2C_Type *base,
                             uint8_t deviceAddress,
                             uint32_t subAddress,
                             uint8_t subaddressSize,
                             uint8_t *rxBuff,
                             uint8_t rxBuffSize,
                             uint32_t flags);
void BOARD_Accel_I2C_Init(void);
status_t BOARD_Accel_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff, uint32_t flags);
status_t BOARD_Accel_I2C_Receive(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subaddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize,
                                 uint32_t flags);
void BOARD_Codec_I2C_Init(void);
status_t BOARD_Codec_I2C_Send(uint8_t deviceAddress,
                              uint32_t subAddress,
                              uint8_t subAddressSize,
                              const uint8_t *txBuff,
                              uint8_t txBuffSize,
                              uint32_t flags);
status_t BOARD_Codec_I2C_Receive(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize,
                                 uint32_t flags);

void BOARD_Display_I2C_Init(void);
status_t BOARD_Display_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize);
status_t BOARD_Display_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize);

#if defined(BOARD_USE_PCA6416A) && BOARD_USE_PCA6416A
void BOARD_PCA6416A_I2C_Init(void);
status_t BOARD_PCA6416A_I2C_Send(uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 const uint8_t *txBuff,
                                 uint8_t txBuffSize,
                                 uint32_t flags);
status_t BOARD_PCA6416A_I2C_Receive(uint8_t deviceAddress,
                                    uint32_t subAddress,
                                    uint8_t subAddressSize,
                                    uint8_t *rxBuff,
                                    uint8_t rxBuffSize,
                                    uint32_t flags);

/* PCA6416A U27. */
extern pca6416a_handle_t g_pca6416aHandle;

void BOARD_InitPCA6416A(pca6416a_handle_t *handle);

/* MIPI DSI */
void BOARD_InitMipiDsiPins(void);
void BOARD_EnableMipiDsiBacklight(void);
#endif /* BOARD_USE_PCA6416A */

#endif /* SDK_I2C_BASED_COMPONENT_USED */
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
