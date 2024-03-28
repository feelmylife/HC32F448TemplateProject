/**
 *******************************************************************************
 * @file  protect.c
 * @brief This file contains motor run protect function.
 * @verbatim
       Change Logs:
       Date             Author          Notes
       2022-09-30       CDT             First version
 * @endverbatim
 *******************************************************************************
 * Copyright (C) 2022, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_source/motor_control.h"

/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize function of DC bus voltage protection
 **
 ** \param [in] stcCfg: configuration parameters
 ** \param [in] *pstc:: pointer to DC bus voltage protection
 **
 ** \retval     none
 ******************************************************************************/
void Prot_InitMonitorVbus(stc_prot_vbus_cfg_t stcCfg, stc_prot_vbus_t *pstc)
{
    float32_t f32Temp;
    pstc->i32Q12_AbnormalHighVbus = Q12(stcCfg.f32AbnormalHighVbus);
    pstc->i32Q12_AbnormalLowVbus = Q12(stcCfg.f32AbnormalLowVbus);

    f32Temp = stcCfg.f32Fs * stcCfg.f32AbnormalTimeUs / 1000000.0f;
    pstc->i32AbnormalTime = (int32_t)f32Temp;

    pstc->i32Q12_OvpThreshold = Q12(stcCfg.f32OverVoltageThold);

    f32Temp = stcCfg.f32Fs * stcCfg.f32OverVoltageTimeUs / 1000000.0f;
    pstc->i32OvpTime = (int32_t)f32Temp;

    pstc->i32Q12_UvpThreshold = Q12(stcCfg.f32UnderVoltageThold);

    f32Temp = stcCfg.f32Fs * stcCfg.f32UnderVoltageTimeUs / 1000000.0f;
    pstc->i32UvpTime = (int32_t)f32Temp;

    f32Temp = stcCfg.f32Fs * stcCfg.f32ErrClearTimeMs / 1000.0f;
    pstc->i32ErrClrTime = (int32_t)f32Temp;

    pstc->i32AbnormalCnt = 0;
    pstc->i32OvpCnt = 0;
    pstc->i32UvpCnt = 0;
}

/**
 ******************************************************************************
 ** \brief monitor DC bus voltage
 **             monitor over voltage, under voltage, and abnormal voltage,
 **             record protection reason and do protection process,
 **             clear fault code is system recovers to normal after certain time
 **
 ** \param [in] i32Q8_Vbus: DC bus voltage
 ** \param [in] *pstc:     : DC bus voltage protection structure pointer
 **
 ** \retval     none
 ******************************************************************************/
void Prot_MonitorVbus(int32_t i32Q12_Vbus, stc_prot_vbus_t *pstc)
{
    /** abnormal DC bus voltage protection ************************************/
    if ((g_stcMotorRunPara.u32FaultCode & ERR_VDC_ABNORM) == 0U)
    {
        if ((i32Q12_Vbus >= pstc->i32Q12_AbnormalHighVbus)
                || (i32Q12_Vbus <= pstc->i32Q12_AbnormalLowVbus))
        {
            pstc->i32AbnormalCnt++;
            if (pstc->i32AbnormalCnt >= pstc->i32AbnormalTime)
            {
                g_stcMotorRunPara.u32FaultCode |= ERR_VDC_ABNORM;
                pstc->i32AbnormalCnt = pstc->i32ErrClrTime;
            }
        }
        else
        {
            pstc->i32AbnormalCnt = 0;
        }
    }
    else
    {
        /** start counting for clearing error if no error exist ***************/
        if ((i32Q12_Vbus <= pstc->i32Q12_AbnormalHighVbus)
                && (i32Q12_Vbus >= pstc->i32Q12_AbnormalLowVbus))
        {
            pstc->i32AbnormalCnt--;
            if (pstc->i32AbnormalCnt <= 0)
            {
                g_stcMotorRunPara.u32FaultCode &= (~ERR_VDC_ABNORM);
            }
        }
        else
        {
            pstc->i32AbnormalCnt = pstc->i32ErrClrTime;
        }
    }

    /** over DC bus voltage protection ****************************************/
    if ((g_stcMotorRunPara.u32FaultCode & ERR_VDC_OV) == 0U)
    {
        if (i32Q12_Vbus >= pstc->i32Q12_OvpThreshold)
        {
            pstc->i32OvpCnt++;
            if (pstc->i32OvpCnt >= pstc->i32OvpTime)
            {
                g_stcMotorRunPara.u32FaultCode |= ERR_VDC_OV;
                pstc->i32OvpCnt = pstc->i32ErrClrTime;
            }
        }
        else
        {
            pstc->i32OvpCnt = 0;
        }
    }
    /** clear over voltage error if DC bus voltage less than OV limit *********/
    else
    {
        if (i32Q12_Vbus <= pstc->i32Q12_OvpThreshold)
        {
            pstc->i32OvpCnt--;
            if (pstc->i32OvpCnt <= 0)
            {
                g_stcMotorRunPara.u32FaultCode &= (~ERR_VDC_OV);
            }
        }
        else
        {
            pstc->i32OvpCnt = pstc->i32ErrClrTime;
        }
    }

    /** under DC bus voltage protection ***************************************/
    if ((g_stcMotorRunPara.u32FaultCode & ERR_VDC_UV) == 0U)
    {
        if (i32Q12_Vbus <= pstc->i32Q12_UvpThreshold)
        {
            pstc->i32UvpCnt++;
            if (pstc->i32UvpCnt >= pstc->i32UvpTime)
            {
                g_stcMotorRunPara.u32FaultCode |= ERR_VDC_UV;
                pstc->i32UvpCnt = pstc->i32ErrClrTime;
            }
        }
        else
        {
            pstc->i32UvpCnt = 0;
        }
    }
    /** clear under voltage error if DC bus voltage greater than UV limit *****/
    else
    {
        if (i32Q12_Vbus >= pstc->i32Q12_UvpThreshold)
        {
            pstc->i32UvpCnt--;
            if (pstc->i32UvpCnt <= 0)
            {
                g_stcMotorRunPara.u32FaultCode &= (~ERR_VDC_UV);
            }
        }
        else
        {
            pstc->i32UvpCnt = pstc->i32ErrClrTime;
        }
    }
}

/**
 ******************************************************************************
 ** \brief initialize function of over current protection
 **
 ** \param [in] stcCfg: configuration parameters
 ** \param [in] *pstc:: pointer to over current protection
 **
 ** \retval     none
 ******************************************************************************/
void Prot_InitOverCurrent(stc_prot_oc_cfg_t stcCfg, stc_prot_oc_t *pstc)
{
    float32_t f32Temp;
    pstc->i32Q12_PeakOcpThold = Q12(stcCfg.f32PeakOcpThold);

    f32Temp = stcCfg.f32Fs * stcCfg.f32PeakOcpTimeUs / 1000000.0f;
    pstc->i32PeakOcpTime = (int32_t)f32Temp;

    f32Temp = stcCfg.f32Fs * stcCfg.f32ErrClearTimeMs / 1000.0f;
    pstc->i32ErrClrTime = (int32_t)f32Temp;
    pstc->i32PeakOcpCnt = 0;
}

/**
 ******************************************************************************
 ** \brief motor over current protection
 **             record fault reason and do protection process,
 **             clear fault code is system recovers to normal after certain time
 **
 ** \param [in] stcIuvw: three-phase motor current
 ** \param [in] *pstc: : over current protection structure
 **
 ** \retval     none
 ******************************************************************************/
void Prot_MotorOverCurrent(stc_uvw_t stcIuvw, stc_prot_oc_t *pstc)
{
    int32_t i32DeltaU = abs(stcIuvw.i32Q12_U) - pstc->i32Q12_PeakOcpThold;
    int32_t i32DeltaV = abs(stcIuvw.i32Q12_V) - pstc->i32Q12_PeakOcpThold;
    int32_t i32DeltaW = abs(stcIuvw.i32Q12_W) - pstc->i32Q12_PeakOcpThold;

    /** over current protection of peak value *********************************/
    if ((g_stcMotorRunPara.u32FaultCode & ERR_OC_PEAK) == 0U)
    {

        if ((i32DeltaU >= 0) || (i32DeltaV >= 0) || (i32DeltaW >= 0))
        {
            pstc->i32PeakOcpCnt++;
            if (pstc->i32PeakOcpCnt >= pstc->i32PeakOcpTime)
            {
                g_stcMotorRunPara.u32FaultCode |= ERR_OC_PEAK;
                pstc->i32PeakOcpCnt = pstc->i32ErrClrTime;
            }
        }
        else
        {
            pstc->i32PeakOcpCnt = 0;
        }
    }
    /** clear over current error if DC bus voltage less than OV limit *********/
    else
    {
        if ((i32DeltaU <= 0) || (i32DeltaV <= 0) || (i32DeltaW <= 0))
        {
            pstc->i32PeakOcpCnt--;
            if (pstc->i32PeakOcpCnt <= 0)
            {
                g_stcMotorRunPara.u32FaultCode &= (~ERR_OC_PEAK);
            }
        }
        else
        {
            pstc->i32PeakOcpCnt = pstc->i32ErrClrTime;
        }
    }
}

