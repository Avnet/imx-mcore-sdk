/*
 * Copyright 2020-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_upower.h"
#include "fsl_clock.h"

#ifdef FSL_RTOS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.upower"
#endif

#if (defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE & 0x2))
#define POWERSYS_MUA_RTD ((MU_Type *)(0x38029000u))
#else
#define POWERSYS_MUA_RTD ((MU_Type *)(0x28029000u))
#endif

#define UPOWER_MU ((struct MU_tag *)(POWERSYS_MUA_RTD))

/*******************************************************************************
 * Variables
 ******************************************************************************/
static upwr_isr_callb s_muTxRxHandler;
static upwr_isr_callb s_muNmiHandler;

static volatile bool callbackStatus = false;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void uPower_IRQHandler(void)
{
    if ((POWERSYS_MUA_RTD->CSSR0 & MU_CSSR0_NMIC_MASK) != 0U)
    {
        /* Clear NMI */
        POWERSYS_MUA_RTD->CSSR0 = MU_CSSR0_NMIC_MASK;
        if (s_muNmiHandler)
        {
            s_muNmiHandler();
        }
        else
        {
        }
    }
    else
    {
        assert(s_muTxRxHandler);
        s_muTxRxHandler();
    }
}

static void UPOWER_DummyInstallISR(upwr_isr_callb txrx, upwr_isr_callb excp)
{
    s_muTxRxHandler = txrx;
    s_muNmiHandler  = excp;
}

static void UPOWER_LockMuInt(int lock)
{
    if (lock)
    {
        NVIC_DisableIRQ(uPower_IRQn);
    }
    else
    {
        NVIC_EnableIRQ(uPower_IRQn);
    }
}

static void UPOWER_Ready(uint32_t soc, uint32_t vmajor, uint32_t vminor)
{
    callbackStatus = true;

    (void)soc;
    (void)vmajor;
    (void)vminor;
}

static void UPOWER_Callback(upwr_sg_t sg, uint32_t func, upwr_resp_t errCode, int ret)
{
    callbackStatus = true;
}

bool UPOWER_CheckReq(upwr_sg_t sg)
{
    upwr_req_status_t reqStatus;

    /* wait callback */
    while (!callbackStatus)
        ;

    callbackStatus = false;

    /* Get reply from upower */
    reqStatus = upwr_poll_req_status(sg, NULL, NULL, NULL, 0U);
    if (reqStatus != UPWR_REQ_OK)
    {
        assert(false);
    }
    return true;
}

/*!
 * @brief Initialize MU interface for uPower access.
 *
 * @param pVersion Pointer to the structure to save uPower ROM and RAM versions
 */
void UPOWER_Init(upower_version_t *pVersion)
{
    int status;
    uint32_t soc;
    uint32_t major, minor, fixes;

    CLOCK_EnableClock(kCLOCK_UpowerMuARtd);

    status = upwr_init(RTD_DOMAIN, UPOWER_MU, NULL, NULL, UPOWER_DummyInstallISR, UPOWER_LockMuInt);
    if (status != 0)
    {
        assert(false);
    }

    NVIC_EnableIRQ(uPower_IRQn);

    soc = upwr_rom_version(&major, &minor, &fixes);
    if (soc == 0U)
    {
        assert(false);
    }

    if (pVersion)
    {
        pVersion->romMajor = major;
        pVersion->romMinor = minor;
        pVersion->romFixes = fixes;
    }

    status = upwr_start(1U, UPOWER_Ready);
    if (status != 0)
    {
        assert(false);
    }

    UPOWER_CheckReq(UPWR_SG_EXCEPT);

    major = upwr_ram_version(&minor, &fixes);

    if (pVersion)
    {
        pVersion->ramMajor = major;
        pVersion->ramMinor = minor;
        pVersion->ramFixes = fixes;
    }
}

/*!
 * @brief Deinitialize MU interface for Sentinel access.
 */
void UPOWER_Deinit(void)
{
    NVIC_DisableIRQ(uPower_IRQn);
}

/*!
 * @brief Power on certain domain without reset.
 *
 * @param domain Target domain to power on.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_PowerOnDomain(soc_domain_t domain)
{
    int status;

    status = upwr_pwm_dom_power_on(domain, 0, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Reset and kick off certain domain.
 *
 * @param domain Target domain to boot.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_BootDomain(soc_domain_t domain)
{
    int status;

    status = upwr_pwm_boot_start(domain, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Power on power switches.
 *
 * @param mask Bits to define which switch should be turned on. The value should be ORed by @ref upower_ps_mask_t.
 *             Bit value 1 means the switch is to be powered on; Value 0 means the switch keeps unchanged.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_PowerOnSwitches(uint32_t mask)
{
    int status;
    uint32_t switches = mask;

    status = upwr_pwm_power_on(&switches, NULL, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Power off power switches.
 *
 * @param mask Bits to define which switch should be turned off. The value should be ORed by @ref upower_ps_mask_t.
 *             Bit value 1 means the switch is to be powered off; Value 0 means the switch keeps unchanged.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_PowerOffSwitches(uint32_t mask)
{
    int status;
    uint32_t switches = mask;

    status = upwr_pwm_power_off(&switches, NULL, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Power on memory partitions array/periphery logic. If a memory is requested to turn on, but the
 * power switch that feeds that memory is not, the power switch will be turned on automatically.
 *
 * The parameter mask bits define which memory partition should be turned on. The value should be ORed by
 * upower_mp0_mask_t or upower_mp1_mask_t. Mask bit value 1 means the switch is to be powered on; Value 0
 * means the switch keeps unchanged.
 *
 * @param mask0 memory partition group 0 mask, see @ref upower_mp0_mask_t.
 * @param mask1 memory partition group 1 mask, see @ref upower_mp1_mask_t.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_PowerOnMemPart(uint32_t mask0, uint32_t mask1)
{
    int status;
    uint32_t mem[2] = {mask0, mask1};

    status = upwr_pwm_power_on(NULL, mem, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Power off memory partitions array/periphery logic.
 *
 * The parameter mask bits define which memory partition should be turned off. The value should be ORed by
 * upower_mp0_mask_t and upower_mp1_mask_t. Mask bit value 1 means the switch is to be powered off; Value 0
 * means the switch keeps unchanged.
 *
 * @param mask0 memory partition group 0 mask, see @ref upower_mp0_mask_t.
 * @param mask1 memory partition group 1 mask, see @ref upower_mp1_mask_t.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_PowerOffMemPart(uint32_t mask0, uint32_t mask1)
{
    int status;
    uint32_t mem[2] = {mask0, mask1};

    status = upwr_pwm_power_off(NULL, mem, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Power on memory partitions array logic and power off its periphery logic. If a memory array is
 * requested to turn on, but the power switch that feeds that memory is not, the power switch will be turned
 * on automatically.
 *
 * The parameter mask bits define which memory partition should be turned on. The value should be ORed by
 * upower_mp0_mask_t or upower_mp1_mask_t. Mask bit value 1 means the switch is to be powered on; Value 0
 * means the switch keeps unchanged.
 *
 * @param mask0 memory partition group 0 mask, see @ref upower_mp0_mask_t.
 * @param mask1 memory partition group 1 mask, see @ref upower_mp1_mask_t.
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_RetainMemPart(uint32_t mask0, uint32_t mask1)
{
    int status;
    uint32_t mem[2] = {mask0, mask1};

    status = upwr_pwm_mem_retain(mem, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief M33 call this API to inform uPower, M33 is using ddr.
 *
 * @param use_ddr not 0, true, means that RTD is using ddr. 0, false, means that, RTD is not using ddr.
 * @return 0 if ok, failure otherwise(-1 if service group is busy, -3 if called in an invalid API state).
 */
int UPOWER_SetRtdUseDdr(bool use_ddr)
{
    int status;

    status = upwr_xcp_set_rtd_use_ddr(RTD_DOMAIN, use_ddr, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}
/*!
 * @brief M33 call this API to Power On Application Domain when Application Domain is in Power Down/Deep Power Down
 * mode. After upower get the msg, upower write a flag to registers according to msg. Then upower will get a WUU1
 * interrupt request and upower change Power Down mode/Deep Power mode to Active mode when Application Domain is in
 * Power Down/Deep Power Down mode.
 *
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_PowerOnADInPDMode(void)
{
    int status;

    status = upwr_xcp_set_rtd_apd_llwu(APD_DOMAIN, true, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}

/*!
 * @brief Set SG PowerManagement parameters of uPower.
 *
 * Please see upower_soc_defs.h file for struct upwr_pwm_param_t
 *
 * @param pcfg_in User's selection for PowerManagement parameter of uPower
 * @return 0 if ok, failure otherwise.
 */
int UPOWER_SetPwrMgmtParam(upwr_pwm_param_t *pcfg_in)
{
    int status;

    assert(pcfg_in);
    status = upwr_pwm_param(pcfg_in, UPOWER_Callback);
    if (status == 0)
    {
        UPOWER_CheckReq(UPWR_SG_PWRMGMT);
    }

    return status;
}
