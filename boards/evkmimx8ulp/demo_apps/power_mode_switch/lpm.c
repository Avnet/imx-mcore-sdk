/*
 * Copyright 2017-2021, NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "clock_config.h"
#include "board.h"
#include "fsl_lptmr.h"
#include "fsl_mu.h"
#include "fsl_rtd_cmc.h"
#include "fsl_upower.h"
#include "fsl_sentinel.h"

#include "lpm.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SYSTICK_SOURCE_CLOCK   (CLOCK_GetRtcOscFreq() / 32)
#define SYSTICK_TICKLESS_CLOCK (CLOCK_GetRtcOscFreq() / 32)

#define FLEXSPI_LUT_KEY_VAL (0x5AF05AF0UL)
#define CUSTOM_LUT_LENGTH   (64U)

void SysTick_Handler(void);

/* Define the count per tick of the systick(LPTMR) in run mode. For accuracy purpose,
 * please make SYSTICK_SOURCE_CLOCK times of configTICK_RATE_HZ.
 */
#define SYSTICK_COUNT_PER_TICK (SYSTICK_SOURCE_CLOCK / configTICK_RATE_HZ)

struct _lpm_power_mode_listener
{
    lpm_power_mode_callback_t callback;
    void *data;
    struct _lpm_power_mode_listener *next;
};

typedef struct _lpm_power_mode_listener lpm_power_mode_listener_t;

typedef struct _lpm_nvic_context
{
    uint32_t PriorityGroup;
    uint32_t ISER[8];
    uint8_t IPR[240]; /* ULP CM33 max IRQn is 191 */
    uint8_t SHPR[12];
    uint32_t ICSR;
    uint32_t VTOR;
    uint32_t AIRCR;
    uint32_t SCR;
    uint32_t CCR;
    uint32_t SHCSR;
    uint32_t MMFAR;
    uint32_t BFAR;
    uint32_t CPACR;
    uint32_t NSACR;

} lpm_nvic_context_t;

static lpm_power_mode_t s_curMode;

/* Save latest address of __vecotr_table */
volatile uint32_t s_vector_table_addr;

/* Store Stack Pointer during RTD in Power Down */
volatile uint32_t s_psp;
volatile uint32_t s_msp;
volatile uint32_t s_control;

static SemaphoreHandle_t s_mutex;
#if configSUPPORT_STATIC_ALLOCATION
static StaticSemaphore_t s_staticMutex;
#endif
static lpm_power_mode_listener_t *s_listenerHead;
static lpm_power_mode_listener_t *s_listenerTail;

/* Indicate if execution target is flash */
static bool lpm_flash_target;

/* RTD Power Down Power mode */
static ps_rtd_pwr_mode_cfgs_t rtd_pwr_mode_cfgs = {
    /* PD */
    [PD_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x1c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x901),
            .pad_cfg        = PAD_CFG(0x3, 0x0, 0x0),
            .mon_cfg        = MON_CFG(0x01e80a00, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x0, 0x2, 0x2, 0x0),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* DPD */
    [DPD_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x1c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x909),
            .pad_cfg        = PAD_CFG(0xf, 0x0, 0x1),
            .mon_cfg        = MON_CFG(0x01e80a00, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x0, 0x2, 0x2, 0x0),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* DSL */
    [DSL_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x1c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x901),
            .pad_cfg        = PAD_CFG(0x0, 0x0, 0x0),
            .mon_cfg        = MON_CFG(0x0deb3a00, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x0, 0x2, 0x2, 0x0),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* SLP */
    [SLP_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x1c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x901),
            .pad_cfg        = PAD_CFG(0x0, 0x0, 0x0),
            .mon_cfg        = MON_CFG(0x0deb3a00, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x0, 0x2, 0x2, 0x0),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
    /* ACT */
    [ACT_RTD_PWR_MODE] =
        {
            .in_reg_cfg     = IN_REG_CFG(0x1c, 0x3),
            .pmic_cfg       = PMIC_CFG(0x23, 0x900),
            .pad_cfg        = PAD_CFG(0x0, 0x0, 0x0),
            .mon_cfg        = MON_CFG(0x0deb3a00, 0x0, 0x0),
            .bias_cfg       = BIAS_CFG(0x0, 0x2, 0x2, 0x0),
            .pwrsys_lpm_cfg = PWRSYS_LPM_CFG(0),
        },
};

static ps_rtd_swt_cfgs_t rtd_swt_cfgs = {
    /* PD */
    [PD_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x0, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0x003fec00, 0xc00, 0x003ff3ff),
        },
    /* DPD */
    [DPD_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x0, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0xc00, 0xc00, 0x003fffff),
        },
    /* DSL */
    [DSL_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x00060003, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0x003fffff, 0x003fffff, 0x003ff3ff),
        },
    /* SLP */
    [SLP_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x00060003, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0x003fffff, 0x003fffff, 0x003ff3ff),
        },
    /* ACT */
    [ACT_RTD_PWR_MODE] =
        {
            .swt_board[0] = SWT_BOARD(0x00060003, 0x0007ff83),
            .swt_mem[0]   = SWT_MEM(0x0, 0x0, 0xfffe0000),
            .swt_mem[1]   = SWT_MEM(0x003fffff, 0x003fffff, 0x003ff3ff),
        },
};

static lpm_nvic_context_t s_nvicContext;

/*
   Make sure the size of context buffer is not less
   than the register data plan to preserve
*/
static uint32_t s_iomuxc0Context[0xD6];
static uint32_t s_iomuxc0ContextIndex = 0;

static uint32_t s_rgpiobContext[0x7];
static uint32_t s_rgpiobContextIndex = 0;

static uint32_t s_pcc0Context[0x41];
static uint32_t s_pcc0ContextIndex = 0;

static uint32_t s_pcc1Context[0x14];
static uint32_t s_pcc1ContextIndex = 0;

static uint32_t s_flexspi0Context[0x72];
static uint32_t s_flexspi0ContextIndex = 0;

static uint32_t s_simsecContext[0x9];
static uint32_t s_simsecContextIndex = 0;

uint32_t tmp_stack[0x100];

/* FreeRTOS implemented Systick handler. */
extern void xPortSysTickHandler(void);
extern void __vector_table(void);

bool LPM_Init(void)
{
#if configSUPPORT_STATIC_ALLOCATION
    s_mutex = xSemaphoreCreateMutexStatic(&s_staticMutex);
#else
    s_mutex = xSemaphoreCreateMutex();
#endif

    if (s_mutex == NULL)
    {
        return false;
    }

    s_listenerHead = s_listenerTail = NULL;
    s_curMode                       = LPM_PowerModeRun;

    return true;
}

void LPM_Deinit(void)
{
    if (s_mutex != NULL)
    {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
}

bool LPM_SetPowerMode(lpm_power_mode_t mode)
{
    lpm_power_mode_listener_t *l1, *l2;
    bool ret = true;

    if (mode == s_curMode)
    {
        return ret;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (l1 = s_listenerHead; l1 != NULL; l1 = l1->next)
    {
        if (l1->callback == NULL)
        {
            continue;
        }

        if (!l1->callback(s_curMode, mode, l1->data))
        {
            /* One stakeholder doesn't allow new mode */
            ret = false;
            break;
        }
    }

    if (ret)
    {
        s_curMode = mode;
    }
    else
    {
        /* roll back the state change of previous listeners */
        for (l2 = s_listenerHead; l2 != l1; l2 = l2->next)
        {
            if (l2->callback == NULL)
            {
                continue;
            }

            l2->callback(mode, s_curMode, l2->data);
        }
    }

    xSemaphoreGive(s_mutex);

    return ret;
}

lpm_power_mode_t LPM_GetPowerMode(void)
{
    return s_curMode;
}

bool LPM_SystemSleep(void)
{
    /* Initialize upower config data */
    struct ps_pwr_mode_cfg_t *pwr_sys_cfg = (struct ps_pwr_mode_cfg_t *)UPWR_DRAM_SHARED_BASE_ADDR;

    /* Copy upower config data to target memory */
    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[SLP_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[SLP_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[SLP_RTD_PWR_MODE], &rtd_swt_cfgs[SLP_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[ACT_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[ACT_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[ACT_RTD_PWR_MODE], &rtd_swt_cfgs[ACT_RTD_PWR_MODE], sizeof(swt_config_t));

    for (uint32_t rtd_mode = 0; rtd_mode < NUM_RTD_PWR_MODES; rtd_mode++)
    {
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_board =
            (struct upwr_switch_board_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_board);
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_mem =
            (struct upwr_mem_switches_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_mem);
    }

    RTDCMC_SetPowerModeProtection(CMC_RTD, CMC_RTD_PMPROT_AS_MASK);
    RTDCMC_EnterLowPowerMode(CMC_RTD, kRTDCMC_SleepMode);

    return true;
}

bool LPM_SystemDeepSleep(void)
{
    /* Initialize upower config data */
    struct ps_pwr_mode_cfg_t *pwr_sys_cfg = (struct ps_pwr_mode_cfg_t *)UPWR_DRAM_SHARED_BASE_ADDR;

    /* Copy upower config data to target memory */
    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[DSL_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[DSL_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[DSL_RTD_PWR_MODE], &rtd_swt_cfgs[DSL_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[ACT_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[ACT_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[ACT_RTD_PWR_MODE], &rtd_swt_cfgs[ACT_RTD_PWR_MODE], sizeof(swt_config_t));

    for (uint32_t rtd_mode = 0; rtd_mode < NUM_RTD_PWR_MODES; rtd_mode++)
    {
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_board =
            (struct upwr_switch_board_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_board);
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_mem =
            (struct upwr_mem_switches_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_mem);
    }

    RTDCMC_SetClockMode(CMC_RTD, kRTDCMC_GateNoneClock);
    RTDCMC_SetPowerModeProtection(CMC_RTD, CMC_RTD_PMPROT_ADS_MASK);
    RTDCMC_EnterLowPowerMode(CMC_RTD, kRTDCMC_DeepSleepMode);

    return true;
}

void LPM_NvicStateSave(void)
{
    uint32_t i;
    uint32_t irqRegs;
    uint32_t irqNum;

    irqRegs = (SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) + 1;
    irqNum  = irqRegs * 32;

    s_nvicContext.PriorityGroup = NVIC_GetPriorityGrouping();

    for (i = 0; i < irqRegs; i++)
    {
        s_nvicContext.ISER[i] = NVIC->ISER[i];
    }

    for (i = 0; i < irqNum; i++)
    {
        s_nvicContext.IPR[i] = NVIC->IPR[i];
    }

    s_nvicContext.SHPR[0]  = SCB->SHPR[0];  /* MemManage */
    s_nvicContext.SHPR[1]  = SCB->SHPR[1];  /* BusFault */
    s_nvicContext.SHPR[2]  = SCB->SHPR[2];  /* UsageFault */
    s_nvicContext.SHPR[7]  = SCB->SHPR[7];  /* SVCall */
    s_nvicContext.SHPR[8]  = SCB->SHPR[8];  /* DebugMonitor */
    s_nvicContext.SHPR[10] = SCB->SHPR[10]; /* PendSV */
    s_nvicContext.SHPR[11] = SCB->SHPR[11]; /* SysTick */

    s_nvicContext.ICSR  = SCB->ICSR;
    s_nvicContext.VTOR  = SCB->VTOR;
    s_nvicContext.AIRCR = SCB->AIRCR;
    s_nvicContext.SCR   = SCB->SCR;
    s_nvicContext.CCR   = SCB->CCR;
    s_nvicContext.SHCSR = SCB->SHCSR;
    s_nvicContext.MMFAR = SCB->MMFAR;
    s_nvicContext.BFAR  = SCB->BFAR;
    s_nvicContext.CPACR = SCB->CPACR;
    s_nvicContext.NSACR = SCB->NSACR;
}

void LPM_NvicStateRestore(void)
{
    uint32_t i;
    uint32_t irqRegs;
    uint32_t irqNum;

    irqRegs = (SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) + 1;
    irqNum  = irqRegs * 32;

    NVIC_SetPriorityGrouping(s_nvicContext.PriorityGroup);

    for (i = 0; i < irqRegs; i++)
    {
        NVIC->ISER[i] = s_nvicContext.ISER[i];
    }

    for (i = 0; i < irqNum; i++)
    {
        NVIC->IPR[i] = s_nvicContext.IPR[i];
    }

    SCB->SHPR[0]  = s_nvicContext.SHPR[0];  /* MemManage */
    SCB->SHPR[1]  = s_nvicContext.SHPR[1];  /* BusFault */
    SCB->SHPR[2]  = s_nvicContext.SHPR[2];  /* UsageFault */
    SCB->SHPR[7]  = s_nvicContext.SHPR[7];  /* SVCall */
    SCB->SHPR[8]  = s_nvicContext.SHPR[8];  /* DebugMonitor */
    SCB->SHPR[10] = s_nvicContext.SHPR[10]; /* PendSV */
    SCB->SHPR[11] = s_nvicContext.SHPR[11]; /* SysTick */

    SCB->ICSR  = s_nvicContext.ICSR;
    SCB->VTOR  = s_nvicContext.VTOR;
    SCB->AIRCR = s_nvicContext.AIRCR;
    SCB->SCR   = s_nvicContext.SCR;
    SCB->CCR   = s_nvicContext.CCR;
    SCB->SHCSR = s_nvicContext.SHCSR;
    SCB->MMFAR = s_nvicContext.MMFAR;
    SCB->BFAR  = s_nvicContext.BFAR;
    SCB->CPACR = s_nvicContext.CPACR;
    SCB->NSACR = s_nvicContext.NSACR;
}

void LPM_SaveRegister(uint32_t *buf, uint32_t *index, uint32_t base, uint32_t begin, uint32_t end, uint32_t bitmap)
{
    uint32_t offset = begin;

    while (offset <= end)
    {
        buf[*index] = (*((volatile uint32_t *)(base + offset))) & bitmap;
        (*index)++;
        offset += 4;
    }
}

AT_QUICKACCESS_SECTION_CODE(void LPM_RestoreRegister(
    uint32_t *buf, uint32_t *index, uint32_t base, uint32_t begin, uint32_t end, uint32_t bitmap))
{
    uint32_t offset = begin;
    uint32_t tmp;

    while (offset <= end)
    {
        tmp = *((volatile uint32_t *)(base + offset));
        tmp &= ~bitmap;
        tmp |= buf[*index];
        *((volatile uint32_t *)(base + offset)) = tmp;
        (*index)++;
        offset += 4;
    }
}

void LPM_ModuleStateSave(void)
{
    /* IOMUXC0 */
    s_iomuxc0ContextIndex = 0;
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x0, 0xBC, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x100, 0x15C, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x400, 0x404, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x800, 0x810, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x898, 0x89C, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x8A0, 0x8FC, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x928, 0x9F0, 0xFFFFFFFF);
    LPM_SaveRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0xA04, 0xAE8, 0xFFFFFFFF);

    /* RGPIO B */
    s_rgpiobContextIndex = 0;
    LPM_SaveRegister(s_rgpiobContext, &s_rgpiobContextIndex, GPIOB_BASE, 0x10, 0x1C, 0xFFFFFFFF);
    LPM_SaveRegister(s_rgpiobContext, &s_rgpiobContextIndex, GPIOB_BASE, 0x40, 0x40, 0xFFFFFFFF);
    LPM_SaveRegister(s_rgpiobContext, &s_rgpiobContextIndex, GPIOB_BASE, 0x54, 0x58, 0xFFFFFFFF);

    /* PCC0 */
    s_pcc0ContextIndex = 0;
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x4, 0x90, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x98, 0x98, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0xA0, 0xA4, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0xB0, 0xB4, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0xC4, 0x110, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x118, 0x118, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x130, 0x130, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x138, 0x13c, 0xFFFFFFFF);

    /* PCC1 */
    s_pcc1ContextIndex = 0;
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0xC, 0xC, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x18, 0x18, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x48, 0x58, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x60, 0x7C, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x88, 0x88, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0xA0, 0xA0, 0xFFFFFFFF);
    LPM_SaveRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0xB4, 0xBC, 0xFFFFFFFF);

    /* FlexSPI0 */
    if (lpm_flash_target)
    {
        s_flexspi0ContextIndex = 0;
        LPM_SaveRegister(s_flexspi0Context, &s_flexspi0ContextIndex, FLEXSPI0_BASE, 0x0, 0x0,
                         0xFFFFFFF0); /* Skip MDIS and SWRESET */
        LPM_SaveRegister(s_flexspi0Context, &s_flexspi0ContextIndex, FLEXSPI0_BASE, 0x4, 0xC4, 0xFFFFFFFF);
        LPM_SaveRegister(s_flexspi0Context, &s_flexspi0ContextIndex, FLEXSPI0_BASE, 0x200, 0x2FC,
                         0xFFFFFFFF); /* LUT table */
    }

    /* SIM_SEC */
    s_simsecContextIndex = 0;
    LPM_SaveRegister(s_simsecContext, &s_simsecContextIndex, SIM_SEC_BASE, 0x40, 0x60, 0xFFFFFFFF);
}

AT_QUICKACCESS_SECTION_CODE(void LPM_ModuleStateRestore(void))
{
    /* IOMUXC0 */
    s_iomuxc0ContextIndex = 0;
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x0, 0xBC, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x100, 0x15C, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x400, 0x404, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x800, 0x810, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x898, 0x89C, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x8A0, 0x8FC, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0x928, 0x9F0, 0xFFFFFFFF);
    LPM_RestoreRegister(s_iomuxc0Context, &s_iomuxc0ContextIndex, IOMUXC0_BASE, 0xA04, 0xAE8, 0xFFFFFFFF);

    /* RGPIO B */
    s_rgpiobContextIndex = 0;
    LPM_RestoreRegister(s_rgpiobContext, &s_rgpiobContextIndex, GPIOB_BASE, 0x10, 0x1C, 0xFFFFFFFF);
    LPM_RestoreRegister(s_rgpiobContext, &s_rgpiobContextIndex, GPIOB_BASE, 0x40, 0x40, 0xFFFFFFFF);
    LPM_RestoreRegister(s_rgpiobContext, &s_rgpiobContextIndex, GPIOB_BASE, 0x54, 0x58, 0xFFFFFFFF);

    /* PCC0 */
    s_pcc0ContextIndex = 0;
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x4, 0x90, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x98, 0x98, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0xA0, 0xA4, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0xB0, 0xB4, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0xC4, 0x110, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x118, 0x118, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x130, 0x130, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc0Context, &s_pcc0ContextIndex, PCC0_BASE, 0x138, 0x13c, 0xFFFFFFFF);

    /* PCC1 */
    s_pcc1ContextIndex = 0;
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0xC, 0xC, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x18, 0x18, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x48, 0x58, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x60, 0x7C, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0x88, 0x88, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0xA0, 0xA0, 0xFFFFFFFF);
    LPM_RestoreRegister(s_pcc1Context, &s_pcc1ContextIndex, PCC1_BASE, 0xB4, 0xBC, 0xFFFFFFFF);

    /* FlexSPI0 */
    if (lpm_flash_target)
    {
        s_flexspi0ContextIndex = 0;

        /* Restore generic register */
        LPM_RestoreRegister(s_flexspi0Context, &s_flexspi0ContextIndex, FLEXSPI0_BASE, 0x0, 0x0,
                            0xFFFFFFF0); /* Skip MDIS and SWRESET bits */
        LPM_RestoreRegister(s_flexspi0Context, &s_flexspi0ContextIndex, FLEXSPI0_BASE, 0x4, 0xc4, 0xFFFFFFFF);

        /* Enable FlexSPI module */
        FLEXSPI0->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

        /* Restore LUT */
        FLEXSPI0->LUTKEY = FLEXSPI_LUT_KEY_VAL;
        FLEXSPI0->LUTCR  = FLEXSPI_LUTCR_UNLOCK_MASK;
        for (uint32_t i = 0; i < CUSTOM_LUT_LENGTH; i++)
        {
            FLEXSPI0->LUT[i] = s_flexspi0Context[s_flexspi0ContextIndex + i];
        }
        FLEXSPI0->LUTKEY = FLEXSPI_LUT_KEY_VAL;
        FLEXSPI0->LUTCR  = FLEXSPI_LUTCR_LOCK_MASK;

        /* Software reset */
        FLEXSPI0->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
        while (FLEXSPI0->MCR0 & FLEXSPI_MCR0_SWRESET_MASK)
        {
        }
    }

    /* SIM_SEC */
    s_simsecContextIndex = 0;
    LPM_RestoreRegister(s_simsecContext, &s_simsecContextIndex, SIM_SEC_BASE, 0x40, 0x60, 0xFFFFFFFF);
}

bool LPM_Suspend()
{
    __asm volatile(
        "loop:\n"
        "PUSH    {R4-R11, LR}\n" /* #1 Save core registers to current stack */

        "MRS     R0, PSP\n" /* #2 Save PSP register to global variable s_psp */
        "LDR     R1, =s_psp\n"
        "STR     R0, [R1]\n"

        "MRS     R0, MSP\n" /* #3 Save MSP register to global variable s_msp */
        "LDR     R1, =s_msp\n"
        "STR     R0, [R1]\n"

        "MRS     R0, CONTROL\n" /* #4 Save CONTROL register to global variable s_control */
        "LDR     R1, =s_control\n"
        "STR     R0, [R1]\n");

    LPM_NvicStateSave();   /* #5 Save NVIC setting */
    LPM_ModuleStateSave(); /* #6 Save peripheral setting */

    /* Program CMC0 to notify system (upower) to enter Power Down */
    RTDCMC_SetClockMode(CMC_RTD, kRTDCMC_GateAllSystemClocks);
    RTDCMC_SetPowerModeProtection(CMC_RTD, CMC_RTD_PMPROT_APD_MASK);
    RTDCMC_EnableDebugOperation(CMC_RTD, false);
    RTDCMC_EnterLowPowerMode(CMC_RTD, kRTDCMC_PowerDownMode);
    /* Successful Power Down exit will never come here, but jump to LPM_Resume() */

    /* Power Down entering fail, retore core register berore return */
    __asm volatile("POP    {R4-R11, LR}\n");
    return false;
}

AT_QUICKACCESS_SECTION_CODE(bool LPM_Resume(void))
{
    /* Re-entry point of Power Down wake*/
    __asm volatile(
        "resume:\n"
        "CPSID   I\n"              /* Mask interrupts */
        "LDR     R2, =tmp_stack\n" /* Setup temp stack for following init */
        "ADD     R2, R2, #0x300\n"
        "MSR     MSP, R2\n");

    /* Clear Power Down Resume Entry. */
    SIM_SEC->DGO_GP0   = 0;
    SIM_SEC->DGO_CTRL0 = SIM_SEC_DGO_CTRL0_UPDATE_DGO_GP0_MASK;
    /* Wait DGO GP0 updated */
    while ((SIM_SEC->DGO_CTRL0 & SIM_SEC_DGO_CTRL0_WR_ACK_DGO_GP0_MASK) == 0)
    {
    }
    /* Clear DGO GP0 ACK and UPDATE bits */
    SIM_SEC->DGO_CTRL0 =
        (SIM_SEC->DGO_CTRL0 & ~(SIM_SEC_DGO_CTRL0_UPDATE_DGO_GP0_MASK)) | SIM_SEC_DGO_CTRL0_WR_ACK_DGO_GP0_MASK;

    LPM_ModuleStateRestore(); /* #6 Restore peripheral setting */

    LPM_NvicStateRestore(); /* #5 Restore NVIC setting */

    /* Enable Cache Controller */
    CACHE64_CTRL0->CCR |= CACHE64_CTRL_CCR_ENWRBUF_MASK | CACHE64_CTRL_CCR_ENCACHE_MASK;

    __asm volatile(
        "LDR     R1, =s_control\n" /* #4 Restore CONTROL register from global variable s_control */
        "LDR     R0, [R1]\n"
        "MSR     CONTROL, R0\n"

        "LDR     R1, =s_msp\n" /* #3 Restore MSP register from global variable s_msp */
        "LDR     R0, [R1]\n"
        "MSR     MSP, R0\n"

        "LDR     R1, =s_psp\n" /* #2 Restore PSP register from global variable s_psp */
        "LDR     R0, [R1]\n"
        "MSR     PSP, R0\n"

        "POP    {R4-R11, LR}\n" /* #1 Restore core registers from current stack */
    );

    return true;
}

bool LPM_SystemPowerDown(void)
{
    uint32_t sampleAddr;
    bool status;

    /* Setup Power Down Resume Entry. */
    SIM_SEC->DGO_GP0   = (uint32_t)LPM_Resume;
    SIM_SEC->DGO_CTRL0 = SIM_SEC_DGO_CTRL0_UPDATE_DGO_GP0_MASK;
    /* Wait DGO GP0 updated */
    while ((SIM_SEC->DGO_CTRL0 & SIM_SEC_DGO_CTRL0_WR_ACK_DGO_GP0_MASK) == 0)
    {
    }
    /* Clear DGO GP0 ACK and UPDATE bits */
    SIM_SEC->DGO_CTRL0 =
        (SIM_SEC->DGO_CTRL0 & ~(SIM_SEC_DGO_CTRL0_UPDATE_DGO_GP0_MASK)) | SIM_SEC_DGO_CTRL0_WR_ACK_DGO_GP0_MASK;

    /* Initialize upower config data */
    struct ps_pwr_mode_cfg_t *pwr_sys_cfg = (struct ps_pwr_mode_cfg_t *)UPWR_DRAM_SHARED_BASE_ADDR;

    /* Copy upower config data to target memory */
    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[PD_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[PD_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[PD_RTD_PWR_MODE], &rtd_swt_cfgs[PD_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[ACT_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[ACT_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[ACT_RTD_PWR_MODE], &rtd_swt_cfgs[ACT_RTD_PWR_MODE], sizeof(swt_config_t));

    for (uint32_t rtd_mode = 0; rtd_mode < NUM_RTD_PWR_MODES; rtd_mode++)
    {
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_board =
            (struct upwr_switch_board_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_board);
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_mem =
            (struct upwr_mem_switches_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_mem);
    }

    /* Since AD is in Power Down already, there is
       not possible to save rdc/trdc value for AD, so
       turn off context save feature */
    SENTINEL_SetPowerDown(false, 0, 0, 0, 0);

    sampleAddr = (uint32_t)SystemInit >> 24;
    if ((sampleAddr == 0x4) || (sampleAddr == 0x14))
    {
        lpm_flash_target = true;
    }
    else
    {
        lpm_flash_target = false;
    }

    status = LPM_Suspend();

    BOARD_ResumeClockInit();

    return status;
}

bool LPM_SystemDeepPowerDown(void)
{
    /* Initialize upower config data */
    struct ps_pwr_mode_cfg_t *pwr_sys_cfg = (struct ps_pwr_mode_cfg_t *)UPWR_DRAM_SHARED_BASE_ADDR;

    /* Copy upower config data to target memory */
    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[DPD_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[DPD_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[DPD_RTD_PWR_MODE], &rtd_swt_cfgs[DPD_RTD_PWR_MODE], sizeof(swt_config_t));

    memcpy(&pwr_sys_cfg->ps_rtd_pwr_mode_cfg[ACT_RTD_PWR_MODE], &rtd_pwr_mode_cfgs[ACT_RTD_PWR_MODE],
           sizeof(struct ps_rtd_pwr_mode_cfg_t));
    memcpy(&pwr_sys_cfg->ps_rtd_swt_cfg[ACT_RTD_PWR_MODE], &rtd_swt_cfgs[ACT_RTD_PWR_MODE], sizeof(swt_config_t));

    for (uint32_t rtd_mode = 0; rtd_mode < NUM_RTD_PWR_MODES; rtd_mode++)
    {
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_board =
            (struct upwr_switch_board_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_board);
        pwr_sys_cfg->ps_rtd_pwr_mode_cfg[rtd_mode].swt_mem =
            (struct upwr_mem_switches_t *)(pwr_sys_cfg->ps_rtd_swt_cfg[rtd_mode].swt_mem);
    }

    /* Program CMC0 to notify system (upower) to enter Deep Power Down */
    RTDCMC_SetClockMode(CMC_RTD, kRTDCMC_GateAllSystemClocks);
    RTDCMC_SetPowerModeProtection(CMC_RTD, CMC_RTD_PMPROT_ADPD_MASK);
    RTDCMC_EnableDebugOperation(CMC_RTD, false);
    RTDCMC_EnterLowPowerMode(CMC_RTD, kRTDCMC_DeepPowerDown);

    /* Successful Power Down exit will never come here, but do normal reset */
    return kStatus_Fail;
}

bool LPM_WaitForInterrupt(uint32_t timeoutMilliSec)
{
    uint32_t irqMask;
    status_t status = kStatus_Success;

    irqMask = DisableGlobalIRQ();

    DisableIRQ(SYSTICK_IRQn);
    LPTMR_StopTimer(SYSTICK_BASE);

    switch (s_curMode)
    {
        case LPM_PowerModeWait:
            /* Clear the SLEEPDEEP bit to disable deep sleep mode (wait mode) */
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            __DSB();
            __WFI();
            __ISB();
            status = kStatus_Success;
            break;
        case LPM_PowerModeStop:
            /* Set the SLEEPDEEP bit to enable deep sleep mode (stop mode) */
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            __DSB();
            __WFI();
            __ISB();
            status = kStatus_Success;
            break;
        case LPM_PowerModeSleep:
            if (!LPM_SystemSleep())
            {
                status = kStatus_Fail;
            }
            break;
        case LPM_PowerModeDeepSleep:
            if (!LPM_SystemDeepSleep())
            {
                status = kStatus_Fail;
            }
            break;
        case LPM_PowerModePowerDown:
            if (!LPM_SystemPowerDown())
            {
                status = kStatus_Fail;
            }
            break;
        case LPM_PowerModeDeepPowerDown:
            if (!LPM_SystemDeepPowerDown())
            {
                status = kStatus_Fail;
            }
            break;
        default:
            break;
    }

    LPTMR_StartTimer(SYSTICK_BASE);
    EnableGlobalIRQ(irqMask);

    return status == kStatus_Success;
}

void LPM_RegisterPowerListener(lpm_power_mode_callback_t callback, void *data)
{
    lpm_power_mode_listener_t *l = (lpm_power_mode_listener_t *)pvPortMalloc(sizeof(lpm_power_mode_listener_t));
    assert(l);

    l->callback = callback;
    l->data     = data;
    l->next     = NULL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_listenerHead)
    {
        s_listenerTail->next = l;
        s_listenerTail       = l;
    }
    else
    {
        s_listenerHead = s_listenerTail = l;
    }

    xSemaphoreGive(s_mutex);
}

void LPM_UnregisterPowerListener(lpm_power_mode_callback_t callback, void *data)
{
    lpm_power_mode_listener_t *l, *p = NULL;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (l = s_listenerHead; l != NULL; l = l->next)
    {
        if (l->callback == callback && l->data == data)
        {
            if (p)
            {
                p->next = l->next;
            }
            else
            {
                s_listenerHead = l->next;
            }

            if (l->next == NULL)
            {
                s_listenerTail = p;
            }

            vPortFree(l);
            break;
        }
        p = l;
    }

    xSemaphoreGive(s_mutex);
}

/************ Internal public API start **************/
/* The systick interrupt handler. */
void SYSTICK_HANDLER(void)
{
    /* Clear the interrupt. */
    LPTMR_ClearStatusFlags(SYSTICK_BASE, kLPTMR_TimerCompareFlag);

    if (SYSTICK_BASE->CSR & LPTMR_CSR_TFC_MASK)
    {
        /* Freerun timer means this is the first tick after tickless exit. */
        LPTMR_StopTimer(SYSTICK_BASE);
        LPTMR_SetTimerPeriod(LPTMR0, SYSTICK_SOURCE_CLOCK / configTICK_RATE_HZ);
        SYSTICK_BASE->CSR &= ~LPTMR_CSR_TFC_MASK;
        LPTMR_StartTimer(SYSTICK_BASE);
    }
    /* Call FreeRTOS tick handler. */
    vPortSysTickHandler();
}

/* Override the default definition of vPortSetupTimerInterrupt() that is weakly
 * defined in the FreeRTOS Cortex-M4F port layer with a version that configures LPTMR0
 * to generate the tick interrupt. */
void vPortSetupTimerInterrupt(void)
{
    lptmr_config_t lptmrConfig;

    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    /* Select SIRC as tick timer clock source */
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
    /* Initialize the LPTMR */
    LPTMR_Init(SYSTICK_BASE, &lptmrConfig);

    /* Set timer period */
    LPTMR_SetTimerPeriod(SYSTICK_BASE, SYSTICK_SOURCE_CLOCK / configTICK_RATE_HZ);

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(SYSTICK_BASE, kLPTMR_TimerInterruptEnable);
    NVIC_SetPriority(SYSTICK_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(SYSTICK_IRQn);

    /* Start counting */
    LPTMR_StartTimer(SYSTICK_BASE);
}
