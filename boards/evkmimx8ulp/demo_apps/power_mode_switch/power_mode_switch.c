/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "fsl_rgpio.h"
#include "fsl_lptmr.h"
#include "fsl_upower.h"
#include "fsl_mu.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "board.h"
#include "lpm.h"
#include "app_srtm.h"
#include "power_mode_switch.h"
#include "fsl_wuu.h"
#include "fsl_rtd_cmc.h"
#include "fsl_sentinel.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"

#include "fsl_iomuxc.h"
#include "fsl_lpuart.h"
#include "fsl_reset.h"
/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
#define APP_DEBUG_UART_BAUDRATE       (115200U)             /* Debug console baud rate. */
#define APP_DEBUG_UART_DEFAULT_CLKSRC kCLOCK_IpSrcSircAsync /* SCG SIRC clock. */

/* LPTMR0 is WUU internal module 0. */
#define WUU_MODULE_SYSTICK WUU_MODULE_LPTMR0
/* Allow systick to be a wakeup source in Power Down mode. */
#define SYSTICK_WUU_WAKEUP (false)

#define APP_LPTMR1_IRQ_PRIO (5U)
#define WUU_WAKEUP_PIN_IDX     (24U) /* WUU0_P24 used for RTD Button2 (SW8) */
#define WUU_WAKEUP_PIN_TYPE    kWUU_ExternalPinFallingEdge
#define APP_WAKEUP_BUTTON_NAME "RTD BUTTON2 (SW8)"

typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceLptmr, /*!< Wakeup by LPTMR.        */
    kAPP_WakeupSourcePin    /*!< Wakeup by external pin. */
} app_wakeup_source_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
extern void APP_PowerPreSwitchHook(lpm_power_mode_t targetMode);
extern void APP_PowerPostSwitchHook(lpm_power_mode_t targetMode, bool result);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t s_wakeupTimeout;            /* Wakeup timeout. (Unit: Second) */
static app_wakeup_source_t s_wakeupSource; /* Wakeup source.                 */
static SemaphoreHandle_t s_wakeupSig;
static const char *s_modeNames[] = {"RUN", "WAIT", "STOP", "Sleep", "Deep Sleep", "Power Down", "Deep Power Down"};

/*******************************************************************************
 * Function Code
 ******************************************************************************/
static uint32_t iomuxBackup[25 + 16 + 24]; /* Backup 25 PTA, 16 PTB and 24 PTC IOMUX registers */
static uint32_t gpioICRBackup[25 + 16 + 24];


static void APP_Suspend(void)
{
    uint32_t i;
    uint32_t setting;
    uint32_t backupIndex;
    bool flash_target;

    if ((((uint32_t)SystemInit >> 24) == 0x4) || (((uint32_t)SystemInit >> 24) == 0x14))
    {
        flash_target = true;
    }
    else
    {
        flash_target = false;
    }

    backupIndex = 0;

    /* Backup PTA IOMUXC and GPIOA ICR registers then disable */
    for (i = 0; i <= 24; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY0[i];

        gpioICRBackup[backupIndex] = GPIOA->ICR[i];

        GPIOA->ICR[i] = 0; /* disable interrupts */

        /* Skip PTA20 ~ 23: JTAG pins for debug purpose only */
        if ((i != 20) && (i != 21) && (i != 22) && (i != 23))
        {
            IOMUXC0->PCR0_IOMUXCARRAY0[i] = 0;
        }
        backupIndex++;
    }

    /* Backup PTB IOMUXC and GPIOB ICR registers then disable */
    for (i = 0; i <= 15; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY1[i];

        gpioICRBackup[backupIndex] = GPIOB->ICR[i];

        GPIOB->ICR[i] = 0; /* disable interrupts */

        /* If it's wakeup source, need to set as WUU0_P24 */
        if ((i == 12) && (WUU0->PE2 & WUU_PE2_WUPE24_MASK))
        {
            /* Disable interrupt temperarily to prevent glitch
             * interrupt during switching IOMUXC pin selection
             */
            setting = WUU0->PE2 & WUU_PE2_WUPE24_MASK;
            WUU0->PE2 &= !WUU_PE2_WUPE24_MASK;

            IOMUXC0->PCR0_IOMUXCARRAY1[i] = IOMUXC0_PCR0_IOMUXCARRAY0_MUX(13);

            WUU0->PE2 |= setting;
        }
        else
        {
            IOMUXC0->PCR0_IOMUXCARRAY1[i] = 0;
        }
        backupIndex++;
    }

    /* Backup PTC IOMUXC and GPIOC ICR registers then disable */
    for (i = 0; i <= 23; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY2[i];

        gpioICRBackup[backupIndex] = GPIOC->ICR[i];

        GPIOC->ICR[i] = 0; /* disable interrupts */

        /* Skip PTC0 ~ 10: FlexSPI0 pins if run on flash */
        if ((i > 10) || !flash_target)
        {
            IOMUXC0->PCR0_IOMUXCARRAY2[i] = 0;
        }
        backupIndex++;
    }

    /* Cleare any potential interrupts before enter Power Down */
    WUU0->PF = WUU0->PF;
}

static void APP_Resume(bool resume)
{
    uint32_t i;
    uint32_t backupIndex;

    backupIndex = 0;

    /* Restore PTA IOMUXC and GPIOA ICR registers */
    for (i = 0; i <= 24; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY0[i] = iomuxBackup[backupIndex];
        GPIOA->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    /* Restore PTB IOMUXC and GPIOB ICR registers */
    for (i = 0; i <= 15; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY1[i] = iomuxBackup[backupIndex];
        GPIOB->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    /* Restore PTC IOMUXC and GPIOC ICR registers */
    for (i = 0; i <= 23; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY2[i] = iomuxBackup[backupIndex];
        GPIOC->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    EnableIRQ(WUU0_IRQn);
}

void APP_PowerPreSwitchHook(lpm_power_mode_t targetMode)
{
    uint32_t setting;

    if ((LPM_PowerModeRun != targetMode))
    {
        /* Wait for debug console output finished. */
        while (!(kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)))
        {
        }
        DbgConsole_Deinit();
        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set to pinmux to analog.
         * Debug console TX pin: Set to pinmux to analog.
         */
        IOMUXC_SetPinMux(BOARD_UART1_TX_PIN_FUNCTION_ID, 0);
        IOMUXC_SetPinConfig(BOARD_UART1_TX_PIN_FUNCTION_ID, 0);
        IOMUXC_SetPinMux(BOARD_UART1_RX_PIN_FUNCTION_ID, 0);
        IOMUXC_SetPinConfig(BOARD_UART1_RX_PIN_FUNCTION_ID, 0);

        if (LPM_PowerModePowerDown == targetMode)
        {
            APP_Suspend();
        }
        else if (LPM_PowerModeDeepPowerDown == targetMode)
        {
            /* If PTB12 is wakeup source, set to WUU0_P24 */
            if ((WUU0->PE2 & WUU_PE2_WUPE24_MASK) != 0)
            {
                /* Disable interrupt temperarily to prevent glitch
                 * interrupt during switching IOMUXC pin selection
                 */
                setting = WUU0->PE2 & WUU_PE2_WUPE24_MASK;
                WUU0->PE2 &= !WUU_PE2_WUPE24_MASK;

                IOMUXC0->PCR0_IOMUXCARRAY1[12] = IOMUXC0_PCR0_IOMUXCARRAY0_MUX(13);

                WUU0->PE2 |= setting;
            }

            /* Cleare any potential interrupts before enter Deep Power Down */
            WUU0->PF = WUU0->PF;
        }
    }
}

void APP_PowerPostSwitchHook(lpm_power_mode_t targetMode, bool result)
{
    if (LPM_PowerModeRun != targetMode)
    {
        if (LPM_PowerModePowerDown == targetMode)
        {
            APP_Resume(result);
        }
        /*
         * Debug console RX pin was set to disable for current leakage, need to re-configure pinmux.
         * Debug console TX pin was set to disable for current leakage, need to re-configure pinmux.
         */
        IOMUXC_SetPinMux(BOARD_UART1_TX_PIN_FUNCTION_ID, 0U);
        IOMUXC_SetPinConfig(BOARD_UART1_TX_PIN_FUNCTION_ID, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
        IOMUXC_SetPinMux(BOARD_UART1_RX_PIN_FUNCTION_ID, 0U);
        IOMUXC_SetPinConfig(BOARD_UART1_RX_PIN_FUNCTION_ID, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);

        BOARD_InitDebugConsole();
    }
    PRINTF("== Power switch %s ==\r\n", result ? "OK" : "FAIL");
}

/* WUU0 interrupt handler. */
void APP_WUU0_IRQHandler(void)
{
    bool wakeup = false;

    if (WUU_GetInternalWakeupModuleFlag(WUU0, WUU_MODULE_LPTMR1))
    {
        /* Woken up by LPTMR, then clear LPTMR flag. */
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
        wakeup = true;
    }

    if (WUU_GetExternalWakeupPinFlag(WUU0, WUU_WAKEUP_PIN_IDX))
    {
        /* Woken up by external pin. */
        WUU_ClearExternalWakeupPinFlag(WUU0, WUU_WAKEUP_PIN_IDX);
        wakeup = true;
    }

    if (WUU_GetInternalWakeupModuleFlag(WUU0, WUU_MODULE_SYSTICK))
    {
        /* Woken up by Systick LPTMR, then clear LPTMR flag. */
        LPTMR_ClearStatusFlags(SYSTICK_BASE, kLPTMR_TimerCompareFlag);
    }

    if (wakeup)
    {
        xSemaphoreGiveFromISR(s_wakeupSig, NULL);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

/* LPTMR1 interrupt handler. */
void LPTMR1_IRQHandler(void)
{
    bool wakeup = false;

    if (kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR1))
    {
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
        wakeup = true;
    }

    if (wakeup)
    {
        xSemaphoreGiveFromISR(s_wakeupSig, NULL);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

static void APP_IRQDispatcher(IRQn_Type irq, void *param)
{
    switch (irq)
    {
        case WUU0_IRQn:
            APP_WUU0_IRQHandler();
            break;
        case GPIOB_INT0_IRQn:
            if ((1U << APP_PIN_IDX(APP_PIN_RTD_BTN2)) &
                RGPIO_GetPinsInterruptFlags(BOARD_SW8_GPIO, kRGPIO_InterruptOutput2))
            {
                /* Flag will be cleared by app_srtm.c */
                xSemaphoreGiveFromISR(s_wakeupSig, NULL);
                portYIELD_FROM_ISR(pdTRUE);
            }
            break;
        default:
            break;
    }
}

/* Get input from user about wakeup timeout. */
static uint8_t APP_GetWakeupTimeout(void)
{
    uint8_t timeout = 0U;
    uint8_t c;

    while (1)
    {
        PRINTF("Select the wake up timeout in seconds.\r\n");
        PRINTF("The allowed range is 1s ~ 999s.\r\n");
        PRINTF("Eg. enter 5 to wake up in 5 seconds.\r\n");
        PRINTF("\r\nWaiting for input timeout value...\r\n\r\n");

        do
        {
            c = GETCHAR();
            if ((c >= '0') && (c <= '9'))
            {
                PRINTF("%c", c);
                timeout = timeout * 10U + c - '0';
            }
            else if ((c == '\r') || (c == '\n'))
            {
                break;
            }
            else
            {
                PRINTF("%c\r\nWrong value!\r\n", c);
                timeout = 0U;
            }
        } while (timeout != 0U && timeout < 100U);

        if (timeout > 0U)
        {
            PRINTF("\r\n");
            break;
        }
    }

    return timeout;
}

/* Get wakeup source by user input. */
static app_wakeup_source_t APP_GetWakeupSource(void)
{
    uint8_t ch;

    while (1)
    {
        PRINTF("Select the wake up source:\r\n");
        PRINTF("Press T for LPTMR - Low Power Timer\r\n");
        PRINTF("Press S for switch/button %s. \r\n", APP_WAKEUP_BUTTON_NAME);

        PRINTF("\r\nWaiting for key press..\r\n\r\n");

        ch = GETCHAR();

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }

        if (ch == 'T')
        {
            return kAPP_WakeupSourceLptmr;
        }
        else if (ch == 'S')
        {
            return kAPP_WakeupSourcePin;
        }
        else
        {
            PRINTF("Wrong value!\r\n");
        }
    }
}

/* Get wakeup timeout and wakeup source. */
static void APP_GetWakeupConfig(void)
{
    /* Get wakeup source by user input. */
    s_wakeupSource = APP_GetWakeupSource();

    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        /* Wakeup source is LPTMR, user should input wakeup timeout value. */
        s_wakeupTimeout = APP_GetWakeupTimeout();
        PRINTF("Will wakeup in %d seconds.\r\n", s_wakeupTimeout);
    }
    else
    {
        PRINTF("Press %s to wake up.\r\n", APP_WAKEUP_BUTTON_NAME);
    }
}

static void APP_SetWakeupConfig(lpm_power_mode_t targetMode)
{
    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        LPTMR_SetTimerPeriod(LPTMR1, (1000UL * s_wakeupTimeout / 16U));
        LPTMR_StartTimer(LPTMR1);
        LPTMR_EnableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
    }

    /* To avoid conflicting access of WUU with SRTM dispatcher, we put the WUU setting into SRTM dispatcher context.*/
    /* If targetMode is PD/DPD, setup WUU. */
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        if (kAPP_WakeupSourceLptmr == s_wakeupSource)
        {
            /* Set WUU LPTMR1 module wakeup source. */
            APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, kWUU_InternalModuleDMATrigger);
        }
        else
        {
            /* Set PORT and WUU wakeup pin. */
            APP_SRTM_SetWakeupPin(APP_PIN_RTD_BTN2, (uint16_t)WUU_WAKEUP_PIN_TYPE | 0x100);
        }
    }
    else
    {
        /* Set PORT pin. */
        if (kAPP_WakeupSourcePin == s_wakeupSource)
        {
            APP_SRTM_SetWakeupPin(APP_PIN_RTD_BTN2, (uint16_t)WUU_WAKEUP_PIN_TYPE);
        }
    }
}

static void APP_ClearWakeupConfig(lpm_power_mode_t targetMode)
{
    if (kAPP_WakeupSourcePin == s_wakeupSource)
    {
        APP_SRTM_SetWakeupPin(APP_PIN_RTD_BTN2, (uint16_t)kWUU_ExternalPinDisable);
    }
    else if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, false);
    }
}

/* Power Mode Switch task */
void PowerModeSwitchTask(void *pvParameters)
{
    lptmr_config_t lptmrConfig;
    lpm_power_mode_t targetPowerMode;
    uint32_t freq = 0U;
    uint8_t ch;

    /* As IRQ handler main entry locates in app_srtm.c to support services, here need an entry to handle application
     * IRQ events.
     */
    APP_SRTM_SetIRQHandler(APP_IRQDispatcher, NULL);
    /* Add Systick as Power Down wakeup source, depending on SYSTICK_WUU_WAKEUP value. */
    APP_SRTM_SetWakeupModule(WUU_MODULE_SYSTICK, SYSTICK_WUU_WAKEUP);

    /* Setup LPTMR. */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1; /* Use RTC 1KHz as clock source. */
    lptmrConfig.bypassPrescaler      = false;
    lptmrConfig.value                = kLPTMR_Prescale_Glitch_3; /* Divide clock source by 16. */
    LPTMR_Init(LPTMR1, &lptmrConfig);
    NVIC_SetPriority(LPTMR1_IRQn, APP_LPTMR1_IRQ_PRIO);

    EnableIRQ(LPTMR1_IRQn);

    for (;;)
    {
        freq = CLOCK_GetFreq(kCLOCK_Cm33CorePlatClk);
        PRINTF("\r\n####################  Power Mode Switch Task ####################\n\r\n");
        PRINTF("    Build Time: %s--%s \r\n", __DATE__, __TIME__);
        PRINTF("    Core Clock: %dHz \r\n", freq);
        PRINTF("\r\nSelect the desired operation \n\r\n");
        PRINTF("Press  %c to enter: Normal RUN mode\r\n", kAPP_PowerModeRun);
        PRINTF("Press  %c to enter: WAIT mode\r\n", kAPP_PowerModeWait);
        PRINTF("Press  %c to enter: STOP mode\r\n", kAPP_PowerModeStop);
        PRINTF("Press  %c to enter: Sleep mode\r\n", kAPP_PowerModeSleep);
        PRINTF("Press  %c to enter: Deep Sleep mode\r\n", kAPP_PowerModeDeepSleep);
        PRINTF("Press  %c to enter: Power Down mode\r\n", kAPP_PowerModePowerDown);
        PRINTF("Press  %c to enter: Deep Power Down\r\n", kAPP_PowerModeDeepPowerDown);
        PRINTF("\r\nWaiting for power mode select..\r\n\r\n");

        /* Wait for user response */
        do
        {
            ch = GETCHAR();
        } while ((ch == '\r') || (ch == '\n'));

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }
        targetPowerMode = (lpm_power_mode_t)(ch - 'A');

        if (targetPowerMode <= LPM_PowerModeDeepPowerDown)
        {
            if (!LPM_SetPowerMode(targetPowerMode))
            {
                PRINTF("Some task doesn't allow to enter mode %s\r\n", s_modeNames[targetPowerMode]);
            }
            else /* Idle task will handle the low power state. */
            {
                APP_GetWakeupConfig();
                APP_SetWakeupConfig(targetPowerMode);
                xSemaphoreTake(s_wakeupSig, portMAX_DELAY);
                /* The call might be blocked by SRTM dispatcher task. Must be called after power mode reset. */
                APP_ClearWakeupConfig(targetPowerMode);
            }
        }
        else
        {
            PRINTF("Invalid command %c[0x%x]\r\n", ch, ch);
        }

        PRINTF("\r\nNext loop\r\n");
    }
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc Failed!!!\r\n");
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t irqMask;
    lpm_power_mode_t targetPowerMode;
    lpm_power_mode_t targetMode;
    upwr_pwm_param_t param;
    bool result;

    targetMode = LPM_GetPowerMode();

    /* Workround for PD/DPD exit fail if sleep more than 1 second */
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        param.R              = 0;
        param.B.DPD_ALLOW    = 0;
        param.B.DSL_DIS      = 1; /* 1= uPower won't go Deep Sleep */
        param.B.SLP_ALLOW    = 0;
        param.B.DSL_BGAP_OFF = 0;
        param.B.DPD_BGAP_ON  = 0;

        UPOWER_SetPwrMgmtParam(&param);
    }

    irqMask = DisableGlobalIRQ();

    /* Only when no context switch is pending and no task is waiting for the scheduler
     * to be unsuspended then enter low power entry.
     */
    if (eTaskConfirmSleepModeStatus() != eAbortSleep)
    {
        targetPowerMode = LPM_GetPowerMode();
        if (targetPowerMode != LPM_PowerModeRun)
        {
            /* Only wait when target power mode is not running */
            APP_PowerPreSwitchHook(targetPowerMode);
            result = LPM_WaitForInterrupt((uint64_t)1000 * xExpectedIdleTime / configTICK_RATE_HZ);
            APP_PowerPostSwitchHook(targetPowerMode, result);
        }
    }
    EnableGlobalIRQ(irqMask);
}

/* Called in PowerModeSwitchTask */
static bool APP_LpmListener(lpm_power_mode_t curMode, lpm_power_mode_t newMode, void *data)
{
    PRINTF("WorkingTask %d: Transfer from %s to %s\r\n", (uint32_t)data, s_modeNames[curMode], s_modeNames[newMode]);

    /* Do necessary preparation for this mode change */

    return true; /* allow this switch */
}

/*!
 * @brief simulating working task.
 */
static void WorkingTask(void *pvParameters)
{
    LPM_RegisterPowerListener(APP_LpmListener, pvParameters);

    for (;;)
    {
        /* Use App task logic to replace vTaskDelay */
        PRINTF("Task %d is working now\r\n", (uint32_t)pvParameters);
        vTaskDelay(portMAX_DELAY);
    }
}

/*! @brief Main function */
int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(kCLOCK_Sai0, kCLOCK_Cm33SaiClkSrcPll1Pfd2Div);

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    RESET_PeripheralReset(kRESET_Sai0);
    RESET_PeripheralReset(kRESET_Lpi2c0);
    RESET_PeripheralReset(kRESET_Lpi2c1);
    RESET_PeripheralReset(kRESET_Tpm0);

    APP_SRTM_Init();

    APP_SRTM_StartCommunication();

    LPM_Init();

    s_wakeupSig = xSemaphoreCreateBinary();

    xTaskCreate(PowerModeSwitchTask, "Main Task", 512U, NULL, tskIDLE_PRIORITY + 1U, NULL);
    xTaskCreate(WorkingTask, "Working Task", configMINIMAL_STACK_SIZE, (void *)1, tskIDLE_PRIORITY + 2U, NULL);

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Application should never reach this point. */
    for (;;)
    {
    }
}
