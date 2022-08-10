/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "fsl_lpi2c_freertos.h"

#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_pwm_adapter.h"
#include "srtm_pwm_service.h"
#include "srtm_message.h"
#include "srtm_rpmsg_endpoint.h"
#include "srtm_i2c_service.h"
#include "srtm_sai_edma_adapter.h"
#include "srtm_io_service.h"
#include "srtm_keypad_service.h"
#include "srtm_lfcl_service.h"
#include "srtm_rtc_service.h"
#include "srtm_rtc_adapter.h"

#include "app_srtm.h"
#include "board.h"
#include "fsl_mu.h"
#include "fsl_debug_console.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"
#include "fsl_upower.h"
#include "fsl_iomuxc.h"
#include "rsc_table.h"
#include "fsl_bbnsm.h"
#include "fsl_sentinel.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum
{
    APP_SRTM_StateRun = 0x0U,
    APP_SRTM_StateLinkedUp,
    APP_SRTM_StateReboot,
    APP_SRTM_StateShutdown,
} app_srtm_state_t;

typedef struct
{
    uint16_t ioId;
    TimerHandle_t timer; /* GPIO glitch detect timer */
    srtm_io_event_t event;
    bool wakeup;
    bool overridden; /* Means the CA35 pin configuration is overridden by CM33 wakeup pin. */
    uint8_t index;
    uint8_t value;
} app_io_t;

/* NOTE: CM33 DRIVERS DON'T SUPPORT SAVE CONTEXT FOR RESUME, BUT CA35 LINUX DRIVERS DO.
 * WHEN CM33 CORE RUNS INTO VLLS MODE, MOST PERIPHERALS STATE WILL BE LOST. HERE PROVIDES
 * AN EXAMPLE TO SAVE DEVICE STATE BY APPLICATION IN A SUSPEND CONTEXT LOCATING IN TCM
 * WHICH CAN KEEP DATA IN VLLS MODE.
 */
typedef struct
{
    struct
    {
        app_io_t data[APP_IO_NUM];
    } io;
    struct
    {
        uint32_t CR;
    } mu;
} app_suspend_ctx_t;

typedef enum
{
    CORE_ACT  = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x0U),
    CORE_STDB = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x1U),
    CORE_PD   = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x3U),
} core_low_power_mode_t; /* A35 core0/1 low power mode */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static srtm_status_t APP_SRTM_I2C_Read(srtm_i2c_adapter_t adapter,
                                       uint32_t base_addr,
                                       srtm_i2c_type_t type,
                                       uint16_t slaveAddr,
                                       uint8_t *buf,
                                       uint8_t len,
                                       uint16_t flags);

static srtm_status_t APP_SRTM_I2C_Write(srtm_i2c_adapter_t adapter,
                                        uint32_t base_addr,
                                        srtm_i2c_type_t type,
                                        uint16_t slaveAddr,
                                        uint8_t *buf,
                                        uint8_t len,
                                        uint16_t flags);

static srtm_status_t APP_SRTM_I2C_SwitchChannel(srtm_i2c_adapter_t adapter,
                                                uint32_t base_addr,
                                                srtm_i2c_type_t type,
                                                uint16_t slaveAddr,
                                                srtm_i2c_switch_channel channel);

static srtm_status_t APP_IO_ConfIEvent(
    srtm_service_t service, srtm_peercore_t core, uint16_t ioId, srtm_io_event_t event, bool wakeup);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const uint16_t wuuPins[] = {
    0x0000U, /* WUU_P0 PTA0 */
    0x0003U, /* WUU_P1 PTA3 */
    0x0004U, /* WUU_P2 PTA4 */
    0x0006U, /* WUU_P3 PTA6 */
    0x0007U, /* WUU_P4 PTA7 */
    0x0008U, /* WUU_P5 PTA8 */
    0x0009U, /* WUU_P6 PTA9 */
    0x000AU, /* WUU_P7 PTA10 */
    0x000BU, /* WUU_P8 PTA11 */
    0x000CU, /* WUU_P9 PTA12 */
    0x000DU, /* WUU_P10 PTA13 */
    0x000EU, /* WUU_P11 PTA14 */
    0x000FU, /* WUU_P12 PTA15 */
    0x0010U, /* WUU_P13 PTA16 */
    0x0011U, /* WUU_P14 PTA17 */
    0x0012U, /* WUU_P15 PTA18 */
    0x0018U, /* WUU_P16 PTA24 */

    0x0100U, /* WUU_P17 PTB0 */
    0x0101U, /* WUU_P18 PTB1 */
    0x0102U, /* WUU_P19 PTB2 */
    0x0103U, /* WUU_P20 PTB3 */
    0x0104U, /* WUU_P21 PTB4 */
    0x0105U, /* WUU_P22 PTB5 */
    0x0106U, /* WUU_P23 PTB6 */
    0x010CU, /* WUU_P24 PTB12 */
    0x010DU, /* WUU_P25 PTB13 */
    0x010EU, /* WUU_P26 PTB14 */
    0x010FU, /* WUU_P27 PTB15 */
};

static const srtm_io_event_t rtdBtn1KeyEvents[] = {
    SRTM_IoEventNone,        /* SRTM_KeypadEventNone */
    SRTM_IoEventRisingEdge,  /* SRTM_KeypadEventPress */
    SRTM_IoEventFallingEdge, /* SRTM_KeypadEventRelease */
    SRTM_IoEventEitherEdge   /* SRTM_KeypadEventPressOrRelease */
};

static const srtm_io_event_t rtdBtn2KeyEvents[] = {
    SRTM_IoEventNone,        /* SRTM_KeypadEventNone */
    SRTM_IoEventRisingEdge,  /* SRTM_KeypadEventPress */
    SRTM_IoEventFallingEdge, /* SRTM_KeypadEventRelease */
    SRTM_IoEventEitherEdge   /* SRTM_KeypadEventPressOrRelease */
};

static const srtm_io_event_t wuuPinModeEvents[] = {
    SRTM_IoEventNone,        /* kWUU_ExternalPinDisable */
    SRTM_IoEventRisingEdge,  /* kWUU_ExternalPinRisingEdge */
    SRTM_IoEventFallingEdge, /* kWUU_ExternalPinFallingEdge */
    SRTM_IoEventEitherEdge   /* kWUU_ExternalPinAnyEdge */
};

static srtm_dispatcher_t disp;
static srtm_peercore_t core;
static srtm_sai_adapter_t saiAdapter;
static srtm_service_t audioService;
static srtm_service_t pwmService;
static srtm_service_t rtcService;
static srtm_rtc_adapter_t rtcAdapter;
static srtm_service_t i2cService;
static srtm_service_t ioService;
static srtm_service_t keypadService;
static SemaphoreHandle_t monSig;
static volatile app_srtm_state_t srtmState;
static struct rpmsg_lite_instance *rpmsgHandle;
static app_rpmsg_monitor_t rpmsgMonitor;
static void *rpmsgMonitorParam;
static TimerHandle_t linkupTimer;
static TimerHandle_t refreshS400WdgTimer;
static TimerHandle_t rtcAlarmEventTimer; /* It is used to send alarm event to acore after acore(acore entered power down
                                            mode) is waken by rtc alarm(Avoid losting a rtc alarm event) */
static app_irq_handler_t irqHandler;
static void *irqHandlerParam;

static HAL_PWM_HANDLE_DEFINE(pwmHandle0);

static HAL_RTC_HANDLE_DEFINE(rtcHandle);

/*
 * AD: Application Domain
 * LP: Low Power
 * Low Power Modes for Application Domain is indroduced in AD_PMCTRLi of CMC1:
 * Active,
 * Sleep,
 * Deep Sleep,
 * Partial Active,
 * Power Down(PD),
 * Deep Power Down(DPD),
 * Hold
 */
enum AD_LPMode
{
    AD_ACT,
    AD_PD,  /* Application Domain enter Power Down Mode when linux execute suspend command(echo mem > /sys/power/state,
               suspend to ram) */
    AD_DPD, /* Application Domian enter Deep Power Down Mode when linux execute poweroff command */
};

static enum AD_LPMode AD_CurrentMode   = AD_ACT;
static enum AD_LPMode AD_WillEnterMode = AD_ACT;

/* pwmHandles must strictly follow TPM instances. If you don't provide service for some TPM instance,
 * set the corresponding handle to NULL. */
static hal_pwm_handle_t pwmHandles[2] = {(hal_pwm_handle_t)pwmHandle0, NULL};

static struct _i2c_bus platform_i2c_buses[] = {
    {.bus_id         = 0,
     .base_addr      = LPI2C0_BASE,
     .type           = SRTM_I2C_TYPE_LPI2C,
     .switch_idx     = I2C_SWITCH_NONE,
     .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED},
    {.bus_id         = 1,
     .base_addr      = LPI2C1_BASE,
     .type           = SRTM_I2C_TYPE_LPI2C,
     .switch_idx     = I2C_SWITCH_NONE,
     .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED},
};

static struct _srtm_i2c_adapter i2c_adapter = {.read          = APP_SRTM_I2C_Read,
                                               .write         = APP_SRTM_I2C_Write,
                                               .switchchannel = APP_SRTM_I2C_SwitchChannel,
                                               .bus_structure = {
                                                   .buses      = platform_i2c_buses,
                                                   .bus_num    = sizeof(platform_i2c_buses) / sizeof(struct _i2c_bus),
                                                   .switch_num = 0,
                                               }};

static RGPIO_Type *const gpios[] = RGPIO_BASE_PTRS;

static app_suspend_ctx_t suspendContext;

static MU_Type mu0_mua;
/*******************************************************************************
 * Code
 ******************************************************************************/
void MU0_MUA_Save(void)
{
    mu0_mua.RCR = MU0_MUA->RCR;
}

void MU0_MUA_Restore(void)
{
    MU0_MUA->RCR = mu0_mua.RCR;
}

/* Real Time Domain save context */
void rtdCtxSave(void)
{
    MU0_MUA_Save();
}

/* Real Time Domain restore context */
void rtdCtxRestore(void)
{
    MU0_MUA_Restore();
}

static uint8_t APP_IO_GetWUUPin(uint16_t ioId)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(wuuPins); i++)
    {
        if (wuuPins[i] == ioId)
        {
            break;
        }
    }

    return i;
}

static uint8_t APP_IO_GetIoIndex(uint16_t ioId)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(suspendContext.io.data); i++)
    {
        if (suspendContext.io.data[i].ioId == ioId)
        {
            break;
        }
    }

    return i;
}

static uint8_t APP_Keypad_GetInputIndex(uint8_t keyIdx)
{
    uint8_t i;

    for (i = 0; i < APP_IO_NUM; i++)
    {
        if (suspendContext.io.data[i].index == keyIdx)
        {
            break;
        }
    }

    return i;
}

static srtm_io_event_t APP_Keypad_GetIoEvent(uint8_t keyIdx, srtm_keypad_event_t event)
{
    switch (keyIdx)
    {
        case APP_KEYPAD_INDEX_VOL_PLUS:
            return rtdBtn1KeyEvents[event]; /* Map vol+ to RTD button1 */
        case APP_KEYPAD_INDEX_VOL_MINUS:
            return rtdBtn2KeyEvents[event]; /* Map vol- to RTD button2 */
        default:
            assert(false);
            break;
    }

    return SRTM_IoEventNone;
}

void APP_PowerOnACore(void)
{
    UPOWER_PowerOnADInPDMode();
}

/* WUU interrupt handler. */
void WUU0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(WUU0_IRQn, irqHandlerParam);
    }
}

static void APP_HandleGPIOHander(uint8_t gpioIdx)
{
    BaseType_t reschedule = pdFALSE;
    RGPIO_Type *gpio      = gpios[gpioIdx];

    if (APP_GPIO_IDX(APP_PIN_IT6161_INT) == gpioIdx &&
        (1U << APP_PIN_IDX(APP_PIN_IT6161_INT)) & RGPIO_GetPinsInterruptFlags(gpio, APP_GPIO_INT_SEL))
    {
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, 1U << APP_PIN_IDX(APP_PIN_IT6161_INT));
        /* Ignore the interrrupt of gpio(set interrupt trigger type of gpio after A35 send command to set interrupt
         * trigger type) */
        APP_IO_ConfIEvent(NULL, NULL, APP_PIN_IT6161_INT, SRTM_IoEventNone, false);
        if ((AD_CurrentMode == AD_PD) && suspendContext.io.data[APP_INPUT_IT6161_INT].wakeup)
        {
            /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
            APP_PowerOnACore();
        }
        xTimerStartFromISR(suspendContext.io.data[APP_INPUT_IT6161_INT].timer, &reschedule);
    }

    if (APP_GPIO_IDX(APP_PIN_TOUCH_INT) == gpioIdx &&
        (1U << APP_PIN_IDX(APP_PIN_TOUCH_INT)) & RGPIO_GetPinsInterruptFlags(gpio, APP_GPIO_INT_SEL))
    {
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, 1U << APP_PIN_IDX(APP_PIN_TOUCH_INT));
        /* Ignore the interrrupt of gpio(set interrupt trigger type of gpio after A35 send command to set interrupt
         * trigger type) */
        APP_IO_ConfIEvent(NULL, NULL, APP_PIN_TOUCH_INT, SRTM_IoEventNone, false);
        if ((AD_CurrentMode == AD_PD) && suspendContext.io.data[APP_INPUT_TOUCH_INT].wakeup)
        {
            /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
            APP_PowerOnACore();
        }
        xTimerStartFromISR(suspendContext.io.data[APP_INPUT_TOUCH_INT].timer, &reschedule);
    }

    if (APP_GPIO_IDX(APP_PIN_RTD_BTN1) == gpioIdx &&
        (1U << APP_PIN_IDX(APP_PIN_RTD_BTN1)) & RGPIO_GetPinsInterruptFlags(gpio, APP_GPIO_INT_SEL))
    {
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, (1U << APP_PIN_IDX(APP_PIN_RTD_BTN1)));
        RGPIO_SetPinInterruptConfig(gpio, APP_PIN_IDX(APP_PIN_RTD_BTN1), APP_GPIO_INT_SEL,
                                    kRGPIO_InterruptOrDMADisabled);
        suspendContext.io.data[APP_INPUT_RTD_BTN1].value = RGPIO_PinRead(gpio, APP_PIN_IDX(APP_PIN_RTD_BTN1));
        if ((AD_CurrentMode == AD_PD) && suspendContext.io.data[APP_INPUT_RTD_BTN1].wakeup)
        {
            /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
            APP_PowerOnACore();
        }
        xTimerStartFromISR(suspendContext.io.data[APP_INPUT_RTD_BTN1].timer, &reschedule);
    }

    if (APP_GPIO_IDX(APP_PIN_RTD_BTN2) == gpioIdx &&
        (1U << APP_PIN_IDX(APP_PIN_RTD_BTN2)) & RGPIO_GetPinsInterruptFlags(gpio, APP_GPIO_INT_SEL))
    {
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, (1U << APP_PIN_IDX(APP_PIN_RTD_BTN2)));
        RGPIO_SetPinInterruptConfig(gpio, APP_PIN_IDX(APP_PIN_RTD_BTN2), APP_GPIO_INT_SEL,
                                    kRGPIO_InterruptOrDMADisabled);
        suspendContext.io.data[APP_INPUT_RTD_BTN2].value = RGPIO_PinRead(gpio, APP_PIN_IDX(APP_PIN_RTD_BTN2));
        if ((AD_CurrentMode == AD_PD) && suspendContext.io.data[APP_INPUT_RTD_BTN2].wakeup)
        {
            /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
            APP_PowerOnACore();
        }
        xTimerStartFromISR(suspendContext.io.data[APP_INPUT_RTD_BTN2].timer, &reschedule);
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

void GPIOA_INT0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOA_INT0_IRQn, irqHandlerParam);
    }
    APP_HandleGPIOHander(0U);
}

void GPIOA_INT1_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOA_INT1_IRQn, irqHandlerParam);
    }
    APP_HandleGPIOHander(0U);
}

void GPIOB_INT0_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOB_INT0_IRQn, irqHandlerParam);
    }
    APP_HandleGPIOHander(1U);
}

void GPIOB_INT1_IRQHandler(void)
{
    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(GPIOB_INT1_IRQn, irqHandlerParam);
    }
    APP_HandleGPIOHander(1U);
}

static void rtcAlarmEventTimer_Callback(TimerHandle_t xTimer)
{
    uint32_t status = MU_GetCoreStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterRunFlag) /* A Core in run mode */
    {
        /* Send rpmsg to A Core when Application Domain in active mode(Make sure that DDR is working) */
        SRTM_RtcAdapter_NotifyAlarm(
            rtcAdapter); /* The function SRTM_RtcAdapter_NotifyAlarm will clear alarm interrupt flag */

        xTimerStop(rtcAlarmEventTimer, portMAX_DELAY);
    }
    else
    {
        xTimerStart(rtcAlarmEventTimer, portMAX_DELAY);
    }
}

void BBNSM_IRQHandler(void)
{
    BaseType_t reschedule = pdFALSE;
    uint32_t status       = BBNSM_GetStatusFlags(BBNSM);

    /* If application has handler */
    if (irqHandler)
    {
        irqHandler(BBNSM_IRQn, irqHandlerParam);
    }

    /*
     * Process RTC alarm if present.
     * BBNSM IRQ enable is done in RTC service initialization. So rtcAdapter must be ready.
     */
    if (status & kBBNSM_RTC_AlarmInterruptFlag)
    {
        if (AD_CurrentMode == AD_PD) /* Application Domain is in Power Down Mode */
        {
            /* Power on A Core (A35) */
            APP_PowerOnACore();
            /* Send rtc alarm event in timer */
            xTimerStartFromISR(rtcAlarmEventTimer, &reschedule);
        }
        else if (AD_CurrentMode == AD_ACT)
        {
            /* Send rpmsg to A Core when Application Domain in active mode(Make sure that DDR is working) */
            SRTM_RtcAdapter_NotifyAlarm(
                rtcAdapter); /* The function SRTM_RtcAdapter_NotifyAlarm will clear alarm interrupt flag */
        }
    }

    if (status & kBBNSM_EMG_OFF_InterruptFlag)
    {
        /* Clear emergency power off interrupt flag */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_EMG_OFF_InterruptFlag);
    }

    if (status & kBBNSM_PWR_OFF_InterruptFlag)
    {
        if (AD_CurrentMode == AD_DPD ||
            AD_CurrentMode == AD_PD) /* Application Domain is in Deep Power Down Mode/Power Down Mode */
        {
            /* Power on A Core (A35) */
            APP_PowerOnACore();
        }
        /* Clear BBNSM button off interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_OFF_InterruptFlag);
        if (keypadService && srtmState == APP_SRTM_StateLinkedUp) /* keypad service is created and linux is ready */
        {
        }
    }
    else if (status & kBBNSM_PWR_ON_InterruptFlag)
    {
        /* Clear BBNSM button on interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_ON_InterruptFlag);
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

static uint16_t ioIdTable[APP_IO_NUM] = {APP_PIN_RTD_BTN1, APP_PIN_RTD_BTN2, APP_PIN_PTA19, APP_PIN_PTB5, APP_PIN_PTA5, APP_PIN_PTA6};

#define PIN_FUNC_ID_SIZE (5)
static uint32_t pinFuncId[APP_IO_NUM][PIN_FUNC_ID_SIZE] = {
    {IOMUXC_PTB13_PTB13},
    {IOMUXC_PTB12_PTB12},
    {IOMUXC_PTA19_PTA19},
    {IOMUXC_PTB5_PTB5},
    {IOMUXC_PTA5_PTA5},
    {IOMUXC_PTA6_PTA6},
};

static uint32_t inputMask[APP_IO_NUM] = {
    IOMUXC_PCR_IBE_MASK,
    IOMUXC_PCR_IBE_MASK,
    IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK,
    IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK,
    IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK,
    IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK,
};

static uint32_t outputMask[APP_IO_NUM] = {
    IOMUXC_PCR_IBE_MASK,
    IOMUXC_PCR_IBE_MASK,
    IOMUXC_PCR_OBE_MASK,
    IOMUXC_PCR_OBE_MASK,
    IOMUXC_PCR_OBE_MASK,
    IOMUXC_PCR_OBE_MASK,
};

static int getPinFuncIdIndex(uint16_t ioId)
{
    int index = 0;

    for (index = APP_GPIO_START; index < APP_IO_NUM; index++)
    {
        if (ioId == ioIdTable[index])
            break;
    }
    assert(index != APP_IO_NUM);

    return index;
}

/*
 * @brief Set pad control register
 * @param asInput    use gpio as input, unless use as output
 */
static void APP_IO_SetPinConfig(uint16_t ioId, bool asInput)
{
    int index = 0;

    index = getPinFuncIdIndex(ioId);
    IOMUXC_SetPinConfig(pinFuncId[index][0], pinFuncId[index][1], pinFuncId[index][2], pinFuncId[index][3],
                        pinFuncId[index][4], asInput ? (inputMask[index]) : (outputMask[index]));
}

static srtm_status_t APP_IO_ConfOutput(uint16_t ioId, srtm_io_value_t ioValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 2U); /* We only support GPIOA and GPIOB */
    assert(pinIdx < 32U);

    APP_IO_SetPinConfig(ioId, false);
    RGPIO_PinWrite(gpios[gpioIdx], pinIdx, (uint8_t)ioValue);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_SetOutput(srtm_service_t service,
                                      srtm_peercore_t core,
                                      uint16_t ioId,
                                      srtm_io_value_t ioValue)
{
    uint8_t index = APP_IO_GetIoIndex(ioId);

    assert(index < APP_IO_NUM);

    suspendContext.io.data[index].value = (uint8_t)ioValue;

    return APP_IO_ConfOutput(ioId, ioValue);
}

static srtm_status_t APP_IO_GetInput(srtm_service_t service,
                                     srtm_peercore_t core,
                                     uint16_t ioId,
                                     srtm_io_value_t *pIoValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 2U); /* We only support GPIOA and GPIOB */
    assert(pinIdx < 32U);
    assert(pIoValue);

    *pIoValue = RGPIO_PinRead(gpios[gpioIdx], pinIdx) ? SRTM_IoValueHigh : SRTM_IoValueLow;

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_ConfInput(uint8_t inputIdx, srtm_io_event_t event, bool wakeup)
{
    uint16_t ioId   = suspendContext.io.data[inputIdx].ioId;
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);
    uint8_t wuuIdx  = APP_IO_GetWUUPin(ioId);
    wuu_external_wakeup_pin_config_t config;

    assert(gpioIdx < 2U); /* Only support GPIOA, GPIOB */
    assert(pinIdx < 32U);
    assert(wuuIdx <= ARRAY_SIZE(wuuPins)); /* When wuuIdx == ARRAY_SIZE(wuuPins),
                                              it means there's no WUU pin for ioId. */
    config.event = kWUU_ExternalPinInterrupt;
    config.mode  = kWUU_ExternalPinActiveAlways;

    APP_IO_SetPinConfig(ioId, true);
    switch (event)
    {
        case SRTM_IoEventRisingEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptRisingEdge);
            if (wakeup)
            {
                assert(wuuIdx < ARRAY_SIZE(wuuPins));
                config.edge = kWUU_ExternalPinRisingEdge;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
            }
            break;
        case SRTM_IoEventFallingEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptFallingEdge);
            if (wakeup)
            {
                assert(wuuIdx < ARRAY_SIZE(wuuPins));
                config.edge = kWUU_ExternalPinFallingEdge;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
            }
            break;
        case SRTM_IoEventEitherEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptEitherEdge);
            if (wakeup)
            {
                assert(wuuIdx < ARRAY_SIZE(wuuPins));
                config.edge = kWUU_ExternalPinAnyEdge;
                WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
            }
            break;
        case SRTM_IoEventLowLevel:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptLogicZero);
            /* Power level cannot trigger wakeup */
            assert(!wakeup);
            break;
        case SRTM_IoEventHighLevel:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptLogicOne);
            /* Power level cannot trigger wakeup */
            assert(!wakeup);
            break;
        default:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
            break;
    }
    if (!wakeup && wuuIdx < ARRAY_SIZE(wuuPins))
    {
        config.edge = kWUU_ExternalPinDisable;
        WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
    }

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_ConfIEvent(
    srtm_service_t service, srtm_peercore_t core, uint16_t ioId, srtm_io_event_t event, bool wakeup)
{
    uint8_t inputIdx = APP_IO_GetIoIndex(ioId);

    assert(inputIdx < APP_IO_NUM);

    suspendContext.io.data[inputIdx].event  = event;
    suspendContext.io.data[inputIdx].wakeup = wakeup;

    return APP_IO_ConfInput(inputIdx, event, wakeup);
}

static srtm_status_t APP_IO_ConfKEvent(
    srtm_service_t service, srtm_peercore_t core, uint8_t keyIdx, srtm_keypad_event_t event, bool wakeup)
{
    uint8_t inputIdx = APP_Keypad_GetInputIndex(keyIdx);

    assert(inputIdx < APP_IO_NUM);

    suspendContext.io.data[inputIdx].event  = APP_Keypad_GetIoEvent(keyIdx, event);
    suspendContext.io.data[inputIdx].wakeup = wakeup;

    return APP_IO_ConfInput(inputIdx, suspendContext.io.data[inputIdx].event, wakeup);
}

static void APP_SRTM_PollLinkup(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (srtmState == APP_SRTM_StateRun)
    {
        if (rpmsg_lite_is_link_up(rpmsgHandle))
        {
            srtmState = APP_SRTM_StateLinkedUp;
            xSemaphoreGive(monSig);
        }
        else
        {
            /* Start timer to poll linkup status. */
            xTimerStart(linkupTimer, portMAX_DELAY);
        }
    }
}

static void APP_RefreshS400WdgTimerCallback(TimerHandle_t xTimer)
{
    SENTINEL_Ping();
    PRINTF("\r\n %s: %d ping s400 wdg timer ok\r\n", __func__, __LINE__);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);
}

static void APP_LinkupTimerCallback(TimerHandle_t xTimer)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_PollLinkup, NULL, NULL);

    if (proc)
    {
        SRTM_Dispatcher_PostProc(disp, proc);
    }
}

static void APP_VolPlusTimerCallback(TimerHandle_t xTimer)
{
    uint8_t gpioIdx = APP_GPIO_IDX(APP_PIN_RTD_BTN1);
    uint8_t pinIdx  = APP_PIN_IDX(APP_PIN_RTD_BTN1);
    srtm_keypad_value_t value;

    if (RGPIO_PinRead(gpios[gpioIdx], pinIdx) == suspendContext.io.data[APP_INPUT_RTD_BTN1].value)
    {
        value = suspendContext.io.data[APP_INPUT_RTD_BTN1].value ? SRTM_KeypadValueReleased : SRTM_KeypadValuePressed;
        /* No glitch, a valid user operation */
        if (AD_CurrentMode == AD_ACT)
        {
            /* When A Core(CA35) is running, notify the event to A Core(CA35). */
            SRTM_KeypadService_NotifyKeypadEvent(keypadService, APP_KEYPAD_INDEX_VOL_PLUS, value);
        }
    }

    /* Restore pin detection interrupt */
    APP_IO_ConfInput(APP_INPUT_RTD_BTN1, suspendContext.io.data[APP_INPUT_RTD_BTN1].event, false);
}

static void APP_VolMinusTimerCallback(TimerHandle_t xTimer)
{
    uint8_t gpioIdx = APP_GPIO_IDX(APP_PIN_RTD_BTN2);
    uint8_t pinIdx  = APP_PIN_IDX(APP_PIN_RTD_BTN2);
    srtm_keypad_value_t value;

    if (RGPIO_PinRead(gpios[gpioIdx], pinIdx) == suspendContext.io.data[APP_INPUT_RTD_BTN2].value)
    {
        value = suspendContext.io.data[APP_INPUT_RTD_BTN2].value ? SRTM_KeypadValueReleased : SRTM_KeypadValuePressed;
        /* No glitch, a valid user operation */
        if (AD_CurrentMode == AD_ACT)
        {
            /* When A Core(CA35) is running, notify the event to A Core(CA35). */
            SRTM_KeypadService_NotifyKeypadEvent(keypadService, APP_KEYPAD_INDEX_VOL_MINUS, value);
        }
    }

    /* Restore pin detection interrupt */
    APP_IO_ConfInput(APP_INPUT_RTD_BTN2, suspendContext.io.data[APP_INPUT_RTD_BTN2].event, false);
}

static void APP_It6161IntPinTimerCallback(TimerHandle_t xTimer)
{
    /* When A Core(CA35) is running, notify the event to A Core(CA35). */
    if (AD_CurrentMode == AD_ACT)
    {
        SRTM_IoService_NotifyInputEvent(ioService, APP_PIN_IT6161_INT);
    }
}

static void APP_TouchIntPinTimerCallback(TimerHandle_t xTimer)
{
    if (AD_CurrentMode == AD_ACT)
    {
        /* When A Core(CA35) is running, notify the event to A Core(CA35). */
        SRTM_IoService_NotifyInputEvent(ioService, APP_PIN_TOUCH_INT);
    }
}

static void APP_SRTM_NotifyPeerCoreReady(struct rpmsg_lite_instance *rpmsgHandle, bool ready)
{
    /* deinit and init app task(str_echo/pingpong rpmsg) in APP_SRTM_StateReboot only */
    if (rpmsgMonitor && (srtmState == APP_SRTM_StateReboot))
    {
        rpmsgMonitor(rpmsgHandle, ready, rpmsgMonitorParam);
    }
}

static void APP_SRTM_Linkup(void)
{
    srtm_channel_t chan;
    srtm_rpmsg_endpoint_config_t rpmsgConfig;

    /* Inform upower that m33 is using the ddr */
    UPOWER_SetRtdUseDdr(true);

    /* Create SRTM peer core */
    core = SRTM_PeerCore_Create(PEER_CORE_ID);
    /* Set peer core state to activated */
    SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Activated);

    /* Common RPMsg channel config */
    rpmsgConfig.localAddr   = RL_ADDR_ANY;
    rpmsgConfig.peerAddr    = RL_ADDR_ANY;
    rpmsgConfig.rpmsgHandle = rpmsgHandle;

    /* Create and add SRTM I2C channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_I2C_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM AUDIO channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_AUDIO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);
    assert((audioService != NULL) && (saiAdapter != NULL));
    SRTM_AudioService_BindChannel(audioService, saiAdapter, chan);

    /* Create and add SRTM Keypad channel to peer core */
    rpmsgConfig.epName = APP_SRTM_KEYPAD_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM IO channel to peer core */
    rpmsgConfig.epName = APP_SRTM_IO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM PWM channel to peer core */
    rpmsgConfig.epName = APP_SRTM_PWM_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM RTC channel to peer core */
    rpmsgConfig.epName = APP_SRTM_RTC_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM Life Cycle channel to peer core */
    rpmsgConfig.epName = APP_SRTM_LFCL_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    SRTM_Dispatcher_AddPeerCore(disp, core);
}

static void APP_SRTM_InitPeerCore(void)
{
    copyResourceTable();

    rpmsgHandle = rpmsg_lite_remote_init((void *)RPMSG_LITE_SRTM_SHMEM_BASE, RPMSG_LITE_SRTM_LINK_ID, RL_NO_FLAGS);
    assert(rpmsgHandle);

    /* save context, such as: MU0_MUA[RCR] */
    rtdCtxSave();

    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, true);

    if (rpmsg_lite_is_link_up(rpmsgHandle))
    {
        APP_SRTM_Linkup();
    }
    else
    {
        /* Start timer to poll linkup status. */
        xTimerStart(linkupTimer, portMAX_DELAY);
    }
}

static void APP_SRTM_GpioReset(void)
{
    int32_t i;

    /* First disable all GPIO interrupts configured by CA35 */
    for (i = APP_INPUT_GPIO_CONTROL_BY_ACORE_START; i <= APP_INPUT_GPIO_CONTROL_BY_ACORE_END; i++)
    {
        if (suspendContext.io.data[i].timer)
        {
            xTimerStop(suspendContext.io.data[i].timer, portMAX_DELAY);
        }
        if (suspendContext.io.data[i].overridden)
        {
            /* The IO is configured by CM33 instead of CA35, don't reset HW configuration. */
            suspendContext.io.data[i].event  = SRTM_IoEventNone;
            suspendContext.io.data[i].wakeup = false;
        }
        else
        {
            APP_IO_ConfIEvent(NULL, NULL, suspendContext.io.data[i].ioId, SRTM_IoEventNone, false);
        }
    }

    /* Output pin value doesn't change. */

    /* Then reset IO service */
    SRTM_IoService_Reset(ioService, core);
    SRTM_KeypadService_Reset(keypadService, core);
}

static void APP_SRTM_ResetServices(void)
{
    /* When CA35 resets, we need to avoid async event to send to CA35. Audio and IO services have async events. */
    SRTM_AudioService_Reset(audioService, core);
    SRTM_RtcService_Reset(rtcService, core);
    APP_SRTM_GpioReset();
}

static void APP_SRTM_DeinitPeerCore(void)
{
    /* Stop linkupTimer if it's started. */
    xTimerStop(linkupTimer, portMAX_DELAY);

    /* Notify application for the peer core disconnection. */
    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, false);

    if (core)
    {
        /* Need to let services know peer core is now down. */
        APP_SRTM_ResetServices();

        SRTM_Dispatcher_RemovePeerCore(disp, core);
        SRTM_PeerCore_Destroy(core);
        core = NULL;
    }

    if (rpmsgHandle)
    {
        rpmsg_lite_deinit(rpmsgHandle);
        rpmsgHandle = NULL;
    }

    /* Inform upower that m33 is not using the ddr(it's ready to reset ddr of lpavd) */
    UPOWER_SetRtdUseDdr(false);
}

static void APP_SRTM_InitAudioDevice(void)
{
    edma_config_t dmaConfig;

    /* Initialize DMA0 for SAI */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DMA0, &dmaConfig);

    /* Initialize DMAMUX for SAI */
    EDMA_SetChannelMux(DMA0, APP_SAI_TX_DMA_CHANNEL, kDmaRequestMux0SAI0Tx);
    EDMA_SetChannelMux(DMA0, APP_SAI_RX_DMA_CHANNEL, kDmaRequestMux0SAI0Rx);
}

static void APP_SRTM_InitAudioService(void)
{
    srtm_sai_edma_config_t saiTxConfig;
    srtm_sai_edma_config_t saiRxConfig;

    APP_SRTM_InitAudioDevice();

    /*  Set SAI DMA IRQ Priority. */
    NVIC_SetPriority(APP_DMA_IRQN(APP_SAI_TX_DMA_CHANNEL), APP_SAI_TX_DMA_IRQ_PRIO);
    NVIC_SetPriority(APP_DMA_IRQN(APP_SAI_RX_DMA_CHANNEL), APP_SAI_RX_DMA_IRQ_PRIO);
    NVIC_SetPriority(SAI0_IRQn, APP_SAI_IRQ_PRIO);

    /* Create SAI EDMA adapter */
    SAI_GetClassicI2SConfig(&saiTxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo, kSAI_Channel0Mask);
    saiTxConfig.config.syncMode           = kSAI_ModeSync; /* Tx in Sync mode */
    saiTxConfig.config.fifo.fifoWatermark = FSL_FEATURE_SAI_FIFO_COUNT - 1;
    saiTxConfig.mclk                      = CLOCK_GetIpFreq(kCLOCK_Sai0);
    saiTxConfig.stopOnSuspend             = true; /* Audio data is in DRAM which is not accessable in A core suspend. */
    saiTxConfig.threshold  = UINT32_MAX;          /* Every period transmitted triggers periodDone message to A core. */
    saiTxConfig.dmaChannel = APP_SAI_TX_DMA_CHANNEL;

    SAI_GetClassicI2SConfig(&saiRxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo, kSAI_Channel0Mask);
    saiRxConfig.config.syncMode           = kSAI_ModeAsync; /* Rx in async mode */
    saiRxConfig.config.fifo.fifoWatermark = 1;
    saiRxConfig.mclk                      = saiTxConfig.mclk;
    saiRxConfig.stopOnSuspend             = true; /* Audio data is in DRAM which is not accessable in A core suspend. */
    saiRxConfig.threshold  = UINT32_MAX;          /* Every period received triggers periodDone message to A core. */
    saiRxConfig.dmaChannel = APP_SAI_RX_DMA_CHANNEL;

    saiAdapter = SRTM_SaiEdmaAdapter_Create(SAI0, DMA0, &saiTxConfig, &saiRxConfig);
    assert(saiAdapter);

    /* Create and register audio service */
    audioService = SRTM_AudioService_Create(saiAdapter, NULL);
    SRTM_Dispatcher_RegisterService(disp, audioService);
}

static void APP_SRTM_InitPwmDevice(void)
{
    HAL_PwmInit(pwmHandles[0], 0U, CLOCK_GetTpmClkFreq(0U));
}

static void APP_SRTM_InitPwmService(void)
{
    srtm_pwm_adapter_t pwmAdapter;

    APP_SRTM_InitPwmDevice();
    pwmAdapter = SRTM_PwmAdapter_Create(pwmHandles, ARRAY_SIZE(pwmHandles));
    assert(pwmAdapter);

    /* Create and register pwm service */
    pwmService = SRTM_PwmService_Create(pwmAdapter);
    SRTM_Dispatcher_RegisterService(disp, pwmService);
}

static void APP_SRTM_InitI2CDevice(void)
{
    lpi2c_master_config_t masterConfig;

    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = LPI2C0_BAUDRATE;
    LPI2C_MasterInit(LPI2C0, &masterConfig, I2C_SOURCE_CLOCK_FREQ_LPI2C0);
    masterConfig.baudRate_Hz = LPI2C1_BAUDRATE;
    LPI2C_MasterInit(LPI2C1, &masterConfig, I2C_SOURCE_CLOCK_FREQ_LPI2C1);
}

static void APP_SRTM_InitI2CService(void)
{
    APP_SRTM_InitI2CDevice();
    i2cService = SRTM_I2CService_Create(&i2c_adapter);
    SRTM_Dispatcher_RegisterService(disp, i2cService);
}

static void APP_SRTM_InitIoKeyDevice(void)
{
    uint32_t i;

    rgpio_pin_config_t gpioConfig = {
        kRGPIO_DigitalInput,
        0U,
    };

    /* Init input configuration */
    for (i = APP_INPUT_GPIO_START; i <= APP_INPUT_GPIO_END; i++)
    {
        RGPIO_PinInit(gpios[APP_GPIO_IDX(suspendContext.io.data[i].ioId)], APP_PIN_IDX(suspendContext.io.data[i].ioId),
                      &gpioConfig);
        if (!suspendContext.io.data[i].overridden)
        {
            APP_IO_ConfInput(i, suspendContext.io.data[i].event, suspendContext.io.data[i].wakeup);
        }
    }
}

static void APP_SRTM_InitIoKeyService(void)
{
    /* Init IO structure used in the application. */
    /* Keypad */
    suspendContext.io.data[APP_INPUT_RTD_BTN1].index = APP_KEYPAD_INDEX_VOL_PLUS;  /* use RTD BUTTON1 as vol+ button */
    suspendContext.io.data[APP_INPUT_RTD_BTN2].index = APP_KEYPAD_INDEX_VOL_MINUS; /* use RTD BUTTON2 as vol- button */

    /* GPIO ID */
    suspendContext.io.data[APP_INPUT_RTD_BTN1].ioId = APP_PIN_RTD_BTN1;
    suspendContext.io.data[APP_INPUT_RTD_BTN2].ioId = APP_PIN_RTD_BTN2;
    suspendContext.io.data[APP_INPUT_PTA19].ioId    = APP_PIN_PTA19;
    suspendContext.io.data[APP_INPUT_PTB5].ioId     = APP_PIN_PTB5;
    suspendContext.io.data[APP_OUTPUT_PTA5].ioId    = APP_PIN_PTA5;
    suspendContext.io.data[APP_OUTPUT_PTA6].ioId    = APP_PIN_PTA6;

    APP_SRTM_InitIoKeyDevice();

    /* Enable interrupt for GPIO. */
    NVIC_SetPriority(GPIOA_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOA_INT1_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOB_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOB_INT1_IRQn, APP_GPIO_IRQ_PRIO);
    EnableIRQ(GPIOA_INT0_IRQn);
    EnableIRQ(GPIOA_INT1_IRQn);
    EnableIRQ(GPIOB_INT0_IRQn);
    EnableIRQ(GPIOB_INT1_IRQn);

    ioService = SRTM_IoService_Create();
    SRTM_IoService_RegisterPin(ioService, APP_PIN_PTA19, APP_IO_SetOutput, APP_IO_GetInput, APP_IO_ConfIEvent, NULL);
    SRTM_IoService_RegisterPin(ioService, APP_PIN_PTB5, APP_IO_SetOutput, APP_IO_GetInput, APP_IO_ConfIEvent, NULL);
    SRTM_IoService_RegisterPin(ioService, APP_PIN_PTA5, APP_IO_SetOutput, APP_IO_GetInput, APP_IO_ConfIEvent, NULL);
    SRTM_IoService_RegisterPin(ioService, APP_PIN_PTA6, APP_IO_SetOutput, APP_IO_GetInput, APP_IO_ConfIEvent, NULL);
    SRTM_Dispatcher_RegisterService(disp, ioService);

    keypadService = SRTM_KeypadService_Create();
    SRTM_KeypadService_RegisterKey(keypadService, APP_KEYPAD_INDEX_VOL_PLUS, APP_IO_ConfKEvent, NULL);
    SRTM_KeypadService_RegisterKey(keypadService, APP_KEYPAD_INDEX_VOL_MINUS, APP_IO_ConfKEvent, NULL);
    SRTM_Dispatcher_RegisterService(disp, keypadService);
}

static void APP_SRTM_A35ResetHandler(void)
{
    portBASE_TYPE taskToWake = pdFALSE;

    /* disable interrupt */
    MU_DisableInterrupts(MU0_MUA, kMU_ResetAssertInterruptEnable);

    srtmState = APP_SRTM_StateReboot;

    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    if (pdPASS == xSemaphoreGiveFromISR(monSig, &taskToWake))
    {
        portYIELD_FROM_ISR(taskToWake);
    }
}

static inline uint32_t CMC_ADGetSystemResetInterruptFlag(CMC_AD_Type *base)
{
    return base->SRIF;
}

static inline uint32_t CMC_ADGetAD_PSDORF(CMC_AD_Type *base)
{
    return base->AD_PSDORF;
}

static void CMC_ADClrAD_PSDORF(CMC_AD_Type *base, uint32_t flag)
{
    base->AD_PSDORF = flag; /* Write 1 to clear flag */
}

/*
 * MU Interrrupt RPMsg handler
 */
#ifdef MU0_A_IRQHandler
#undef MU0_A_IRQHandler
#endif

int32_t MU0_A_IRQHandler(void)
{
    uint32_t status = MU_GetStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterPowerDownInterruptFlag) /* PD/DPD mode */
    {
        PRINTF("AD entered PD(linux suspend to ram)/DPD(linux shutdown) mode\r\n");
        MU_ClearStatusFlags(MU0_MUA, (uint32_t)kMU_OtherSideEnterPowerDownInterruptFlag);

        if (AD_WillEnterMode == AD_DPD)
        {
            AD_CurrentMode = AD_WillEnterMode; /* AD entered Deep Power Down Mode */
        }
        else
        {
            /* Relase A Core */
            MU_BootOtherCore(
                MU0_MUA,
                (mu_core_boot_mode_t)0); /* Delete the code after linux supported sending suspend rpmsg to M Core */
            AD_CurrentMode = AD_PD;      /* AD entered Power Down Mode */
        }
        AD_WillEnterMode = AD_ACT;
    }

    return RPMsg_MU0_A_IRQHandler();
}

void CMC1_IRQHandler(void)
{
    uint32_t srs       = CMC_ADGetSystemResetInterruptFlag(CMC_AD);
    uint32_t ad_psdorf = CMC_ADGetAD_PSDORF(CMC_AD);

    if (srs & CMC_AD_SRS_WDOG_AD(1) || (AD_CurrentMode == AD_DPD))
    {
        DisableIRQ(CMC1_IRQn);
        APP_SRTM_A35ResetHandler();
    }

    if (ad_psdorf & CMC_AD_AD_PSDORF_AD_PERIPH(1) && (AD_CurrentMode == AD_PD))
    {
        /* Need clear AD_PERIPH flag after wakup by wuu */
        CMC_ADClrAD_PSDORF(CMC_AD, CMC_AD_AD_PSDORF_AD_PERIPH(1));
        /* Restore context(such as: MU0_MUA[RCR]) */
        rtdCtxRestore();
        PRINTF("\r\nAD resume from Power Down Mode\r\n");

        /* hold A core for next reboot */
        MU_HoldOtherCoreReset(MU0_MUA);
    }

    PRINTF("\r\nAD entered Active mode\r\n");
    AD_CurrentMode = AD_ACT;
}

static void APP_SRTM_InitRtcDevice(void)
{
    HAL_RtcInit(rtcHandle, 0);
    NVIC_ClearPendingIRQ(BBNSM_IRQn);
    NVIC_SetPriority(BBNSM_IRQn, APP_BBNSM_IRQ_PRIO);
    EnableIRQ(BBNSM_IRQn);
}

static void APP_SRTM_InitRtcService(void)
{
    APP_SRTM_InitRtcDevice();
    rtcAdapter = SRTM_RtcAdapter_Create(rtcHandle);
    assert(rtcAdapter);

    rtcService = SRTM_RtcService_Create(rtcAdapter);
    SRTM_Dispatcher_RegisterService(disp, rtcService);
}

static srtm_status_t APP_SRTM_LfclEventHandler(
    srtm_service_t service, srtm_peercore_t core, srtm_lfcl_event_t event, void *eventParam, void *userParam)
{
    switch (event)
    {
        case SRTM_Lfcl_Event_ShutdownReq: /* Notify M Core that Application Domain will enter Deep Power Down Mode */
            AD_WillEnterMode = AD_DPD;
            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            PRINTF("\r\nAD will enter Deep Power Down Mode\r\n");
            break;
        case SRTM_Lfcl_Event_SuspendReq: /* Notify M Core that Application Domain will enter Power Down Mode(Currently
                                            linux driver not support sending the request to M Core) */
            AD_WillEnterMode = AD_PD;
            /* Save context(such as: MU0_MUA[RCR]) */
            rtdCtxSave();
            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            PRINTF("\r\nAD will enter Power Down Mode\r\n");
            break;
        case SRTM_Lfcl_Event_Running: /* Notify M Core that Application Domain entered Active Mode(Currently linux
                                         driver not support sending the request to M Core) */
            AD_CurrentMode = AD_ACT;
            PRINTF("\r\nAD will enter active mode\r\n");
            break;
        default:
            PRINTF("\r\n%s: %d unsupported event: 0x%x\r\n", __func__, __LINE__, event);
            break;
    }

    return SRTM_Status_Success;
}

static void APP_SRTM_InitLfclService(void)
{
    srtm_service_t service;

    /* Create and register Life Cycle service */
    service = SRTM_LfclService_Create();
    SRTM_LfclService_Subscribe(service, APP_SRTM_LfclEventHandler, NULL);
    SRTM_Dispatcher_RegisterService(disp, service);
}

static void APP_SRTM_InitServices(void)
{
    APP_SRTM_InitI2CService();
    APP_SRTM_InitAudioService();
    APP_SRTM_InitIoKeyService();
    APP_SRTM_InitPwmService();
    APP_SRTM_InitRtcService();
    APP_SRTM_InitLfclService();
}

bool is_lowPowerBootType(void)
{
    /*
     * BOOTCFG[0](BT0_CFG0)
     * 0: No Low Power Boot
     * 1: Boot from M33 with A35 on demand
     */
    return CMC_RTD->MR[0] & CMC_MR_BOOTCFG(0);
}

void handshake_with_uboot(void)
{
    TickType_t xTicksToWait = pdMS_TO_TICKS(APP_HANDSHAKE_WITH_UBOOT_TIMEOUT_MS);
    TickType_t currTick     = xTaskGetTickCount();

    /*
     * Wait MU0_MUA FSR F0 flag is set by uboot
     *
     * handshake procedure as follows:
     * a35(set flag F0 of MU0_MUB) --- ready to do MU communication(also indicates MIPI DSI panel ready) ---> m33
     * a35 <--------------- ACK ----------------------------------------------------------------------------> m33 (get
     * flag F0 of MU0_MUA,  and set flag F0 of MU0_MUA) a35(clear flag F0 of MU0_MUB)
     * -----------------------------------------------------------------------> m33 a35
     * <------------------------------------------------------------------------------------------------> m33 (get flag
     * F0 of MU0_MUA and clear flag F0 of MU0_MUA)
     *
     * (uboot will set MU0_MUB FCR F0 flag in board_init(), board/freescale/imx8ulp_evk/imx8ulp_evk.c,
     * after uboot set MU0_MUB FCR F0 flag, the flag will be shown in MU0_MUA FSR)
     */
    /* enable clock of MU0_MUA before accessing registers of MU0_MUA */
    MU_Init(MU0_MUA);
    while (true)
    {
        if (MU_GetFlags(MU0_MUA) & APP_MU0_MUB_F0_INIT_SRTM_COMMUNICATION_FLG)
        {
            /* Set FCR F0 flag of MU0_MUA to notify uboot to clear FCR F0 flag of MU0_MUB */
            MU_SetFlags(MU0_MUA, APP_MU0_MUB_F0_INIT_SRTM_COMMUNICATION_FLG);
            break;
        }
        vTaskDelay(APP_WAIT_MU0_MUB_F0_FLG_FROM_UBOOT_TICK);
        if (currTick + xTicksToWait < xTaskGetTickCount())
        {
            PRINTF("\r\n %s: %d handshake with uboot timeout\r\n", __func__, __LINE__);
            return;
        }
    }

    /*
     * Wait uboot to clear the FCR F0 flag of MU0_MUB
     * Clear FCR F0 flag of MU0_MUA after uboot have cleared the FCR
     * F0 flag of MU0_MUB
     */
    currTick = xTaskGetTickCount(); /* update currTick */
    while (true)
    {
        if ((MU_GetFlags(MU0_MUA) & APP_MU0_MUB_F0_INIT_SRTM_COMMUNICATION_FLG) == 0)
        {
            MU_SetFlags(MU0_MUA, 0);
            break;
        }
        vTaskDelay(APP_WAIT_MU0_MUB_F0_FLG_FROM_UBOOT_TICK);
        if (currTick + xTicksToWait < xTaskGetTickCount())
        {
            PRINTF("\r\n %s: %d handshake with uboot timeout\r\n", __func__, __LINE__);
            MU_SetFlags(MU0_MUA, 0); /* clear flag */
            return;
        }
    }
}

static void SRTM_MonitorTask(void *pvParameters)
{
    app_srtm_state_t state = APP_SRTM_StateShutdown;

    /* Initialize services and add to dispatcher */
    APP_SRTM_InitServices();

    /* Start SRTM dispatcher */
    SRTM_Dispatcher_Start(disp);

    /* Monitor peer core state change */
    while (true)
    {
        xSemaphoreTake(monSig, portMAX_DELAY);

        if (state == srtmState)
        {
            continue;
        }

        switch (srtmState)
        {
            case APP_SRTM_StateRun:
                assert(state == APP_SRTM_StateShutdown);
                PRINTF("Start SRTM communication\r\n");
                SRTM_Dispatcher_Stop(disp);

                BOARD_InitMipiDsiPins();
                BOARD_EnableMipiDsiBacklight();

                if (!is_lowPowerBootType())
                {
                    handshake_with_uboot();
                }

                APP_SRTM_InitPeerCore();
                SRTM_Dispatcher_Start(disp);
                state = APP_SRTM_StateRun;
                break;

            case APP_SRTM_StateLinkedUp:
                if (state == APP_SRTM_StateRun)
                {
                    PRINTF("Handle Peer Core Linkup\r\n");
                    SRTM_Dispatcher_Stop(disp);
                    APP_SRTM_Linkup();
                    SRTM_Dispatcher_Start(disp);
                }
                break;

            case APP_SRTM_StateReboot:
                assert(state == APP_SRTM_StateRun);

                PRINTF("Handle Peer Core Reboot\r\n");

                SRTM_Dispatcher_Stop(disp);
                /* Remove peer core from dispatcher */
                APP_SRTM_DeinitPeerCore();

                /* Initialize io and tpm for uboot */
                BOARD_InitMipiDsiPins();
                BOARD_EnableMipiDsiBacklight();

                /* enable clock of MU0_MUA before accessing registers of MU0_MUA */
                MU_Init(MU0_MUA);

                /* Relase core */
                MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);

                handshake_with_uboot();

                /* Initialize peer core and add to dispatcher */
                APP_SRTM_InitPeerCore();

                /* Restore srtmState to Run. */
                srtmState = APP_SRTM_StateRun;

                SRTM_Dispatcher_Start(disp);

                NVIC_ClearPendingIRQ(CMC1_IRQn);
                EnableIRQ(CMC1_IRQn);

                /* hold A core for next reboot */
                MU_HoldOtherCoreReset(MU0_MUA);

                /* Do not need to change state. It's still Run. */
                break;

            default:
                assert(false);
                break;
        }
    }
}

static void SRTM_DispatcherTask(void *pvParameters)
{
    SRTM_Dispatcher_Run(disp);
}

static void APP_SRTM_InitPeriph(bool resume)
{
}

void APP_SRTM_Init(void)
{
    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    APP_SRTM_InitPeriph(false);

    monSig = xSemaphoreCreateBinary();
    assert(monSig);

    /* Create a rtc alarm event timer to send rtc alarm event to A Core after A Core is waken by rtc alarm(avoid losting
     * rtc alarm event) */
    rtcAlarmEventTimer = xTimerCreate("rtcAlarmEventTimer", APP_MS2TICK(APP_RTC_ALM_EVT_TIMER_PERIOD_MS), pdFALSE, NULL,
                                      rtcAlarmEventTimer_Callback);
    assert(rtcAlarmEventTimer);

    /* Note: Create a task to refresh S400(sentinel) watchdog timer to keep S400 alive, the task will be removed after
     * the bug is fixed in soc A1 */
    SENTINEL_Init();
    refreshS400WdgTimer = xTimerCreate("refreshS400WdgTimer", APP_MS2TICK(APP_REFRESH_S400_WDG_TIMER_PERIOD_MS),
                                       pdFALSE, NULL, APP_RefreshS400WdgTimerCallback);
    assert(refreshS400WdgTimer);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);

    linkupTimer =
        xTimerCreate("Linkup", APP_MS2TICK(APP_LINKUP_TIMER_PERIOD_MS), pdFALSE, NULL, APP_LinkupTimerCallback);
    assert(linkupTimer);

    suspendContext.io.data[APP_INPUT_RTD_BTN1].timer =
        xTimerCreate("Vol+", APP_MS2TICK(50), pdFALSE, NULL, APP_VolPlusTimerCallback);
    assert(suspendContext.io.data[APP_INPUT_RTD_BTN1].timer);
    suspendContext.io.data[APP_INPUT_RTD_BTN2].timer =
        xTimerCreate("Vol-", APP_MS2TICK(50), pdFALSE, NULL, APP_VolMinusTimerCallback);
    assert(suspendContext.io.data[APP_INPUT_RTD_BTN2].timer);

    suspendContext.io.data[APP_INPUT_IT6161_INT].timer =
        xTimerCreate("It6161Int", APP_MS2TICK(50), pdFALSE, NULL, APP_It6161IntPinTimerCallback);
    assert(suspendContext.io.data[APP_INPUT_IT6161_INT].timer);
    suspendContext.io.data[APP_INPUT_TOUCH_INT].timer =
        xTimerCreate("TouchInt", APP_MS2TICK(50), pdFALSE, NULL, APP_TouchIntPinTimerCallback);
    assert(suspendContext.io.data[APP_INPUT_TOUCH_INT].timer);

    /* Create SRTM dispatcher */
    disp = SRTM_Dispatcher_Create();

    NVIC_SetPriority(CMC1_IRQn, APP_CMC1_IRQ_PRIO);
    EnableIRQ(CMC1_IRQn);

    MU_Init(MU0_MUA);
    MU_EnableInterrupts(MU0_MUA, kMU_OtherSideEnterPowerDownInterruptEnable);

    /* hold A core for next reboot */
    MU_HoldOtherCoreReset(MU0_MUA);

    xTaskCreate(SRTM_MonitorTask, "SRTM monitor", 256U, NULL, APP_SRTM_MONITOR_TASK_PRIO, NULL);
    xTaskCreate(SRTM_DispatcherTask, "SRTM dispatcher", 512U, NULL, APP_SRTM_DISPATCHER_TASK_PRIO, NULL);
}

void APP_SRTM_StartCommunication(void)
{
    srtmState = APP_SRTM_StateRun;
    xSemaphoreGive(monSig);
}

void APP_SRTM_Suspend(void)
{
}

void APP_SRTM_Resume(bool resume)
{
    if (resume)
    {
    }
}

void APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor_t monitor, void *param)
{
    rpmsgMonitor      = monitor;
    rpmsgMonitorParam = param;
}

static srtm_status_t APP_SRTM_I2C_SwitchChannel(srtm_i2c_adapter_t adapter,
                                                uint32_t base_addr,
                                                srtm_i2c_type_t type,
                                                uint16_t slaveAddr,
                                                srtm_i2c_switch_channel channel)
{
    uint8_t txBuff[1];
    assert(channel < SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED);
    txBuff[0] = 1 << (uint8_t)channel;
    return adapter->write(adapter, base_addr, type, slaveAddr, txBuff, sizeof(txBuff),
                          SRTM_I2C_FLAG_NEED_STOP); // APP_SRTM_I2C_Write
}

static srtm_status_t APP_SRTM_I2C_Write(srtm_i2c_adapter_t adapter,
                                        uint32_t base_addr,
                                        srtm_i2c_type_t type,
                                        uint16_t slaveAddr,
                                        uint8_t *buf,
                                        uint8_t len,
                                        uint16_t flags)
{
    status_t retVal   = kStatus_Fail;
    uint32_t needStop = (flags & SRTM_I2C_FLAG_NEED_STOP) ? kLPI2C_TransferDefaultFlag : kLPI2C_TransferNoStopFlag;

    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            retVal = BOARD_LPI2C_Send((LPI2C_Type *)base_addr, slaveAddr, 0, 0, buf, len, needStop);
            break;
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

static srtm_status_t APP_SRTM_I2C_Read(srtm_i2c_adapter_t adapter,
                                       uint32_t base_addr,
                                       srtm_i2c_type_t type,
                                       uint16_t slaveAddr,
                                       uint8_t *buf,
                                       uint8_t len,
                                       uint16_t flags)
{
    status_t retVal   = kStatus_Fail;
    uint32_t needStop = (flags & SRTM_I2C_FLAG_NEED_STOP) ? kLPI2C_TransferDefaultFlag : kLPI2C_TransferNoStopFlag;

    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            retVal = BOARD_LPI2C_Receive((LPI2C_Type *)base_addr, slaveAddr, 0, 0, buf, len, needStop);
            break;
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

uint8_t APP_Read_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex)
{
    uint8_t value;
    SRTM_I2C_RequestBusWrite(i2cService, busID, slaveAddr, &regIndex, 1, 0);
    SRTM_I2C_RequestBusRead(i2cService, busID, slaveAddr, &value, 1);
    return value;
}

uint8_t APP_Write_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex, uint8_t value)
{
    uint8_t write_content[2];
    write_content[0] = regIndex;
    write_content[1] = value;
    SRTM_I2C_RequestBusWrite(i2cService, busID, slaveAddr, write_content, 2, 1);
    return value;
}

void APP_SRTM_HandlePeerReboot(void)
{
    if (srtmState != APP_SRTM_StateShutdown)
    {
        srtmState = APP_SRTM_StateReboot;
        xSemaphoreGive(monSig);
    }
}

void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param)
{
    irqHandler      = handler;
    irqHandlerParam = param;
}

static void APP_SRTM_DoSetWakeupModule(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    uint32_t module                          = (uint32_t)param1;
    wuu_internal_wakeup_module_event_t event = (wuu_internal_wakeup_module_event_t)(uint32_t)param2;

    WUU_SetInternalWakeUpModulesConfig(WUU0, module, event);
}

void APP_SRTM_SetWakeupModule(uint32_t module, bool enable)
{
    srtm_procedure_t proc;

    proc = SRTM_Procedure_Create(APP_SRTM_DoSetWakeupModule, (void *)module, (void *)(uint32_t)enable);
    assert(proc);

    SRTM_Dispatcher_CallProc(disp, proc, SRTM_WAIT_FOR_EVER);
    SRTM_Procedure_Destroy(proc);
}

static void APP_SRTM_DoSetWakeupPin(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    uint16_t ioId    = (uint32_t)param1;
    uint16_t event   = (uint32_t)param2;
    uint8_t inputIdx = APP_IO_GetIoIndex(ioId);
    bool wakeup      = (bool)(event >> 8);
    uint8_t pinMode  = (uint8_t)event;

    assert(pinMode < ARRAY_SIZE(wuuPinModeEvents));

    if (wuuPinModeEvents[pinMode] != SRTM_IoEventNone)
    {
        APP_IO_ConfInput(inputIdx, wuuPinModeEvents[pinMode], wakeup);
    }
    else
    {
        /* Restore CA35 settings */
        APP_IO_ConfInput(inputIdx, suspendContext.io.data[inputIdx].event, suspendContext.io.data[inputIdx].wakeup);
    }
}

void APP_SRTM_SetWakeupPin(uint16_t ioId, uint16_t event)
{
    srtm_procedure_t proc;

    proc = SRTM_Procedure_Create(APP_SRTM_DoSetWakeupPin, (void *)(uint32_t)ioId, (void *)(uint32_t)event);
    assert(proc);

    SRTM_Dispatcher_CallProc(disp, proc, SRTM_WAIT_FOR_EVER);
    SRTM_Procedure_Destroy(proc);
}
