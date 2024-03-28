/**
 *******************************************************************************
 * @file  adc_sample.c
 * @brief This file contains mcu adc sample data conversion and use function.
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
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
uint16_t *const Adc_u16AdcResult = ((uint16_t *)MTR1_ADC_DR0_ADDR); // address of ADC_DR0
/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize ADC sample parameters
 **
 ** \param [in] stcCfg: configuration parameters
 ** \param [in] *pstc: pointer to sample structure
 **
 ** \retval     none
 ******************************************************************************/
void Adc_InitSample(stc_sample_cfg_t stcCfg, stc_adc_sample_t *pstc)
{
    float32_t f32Temp;
    pstc->i32Q24_IuvwK = (Q24(ADC_VOLT_REF * stcCfg.f32IuvwK) >> ADC_BIT_LENGTH);
    pstc->i32Q24_VbusK = (Q24(ADC_VOLT_REF * stcCfg.f32VbusK) >> ADC_BIT_LENGTH);
    pstc->i32RefOffset = stcCfg.i32RefIuvwOffset;
    pstc->i32MaxOffsetBias = stcCfg.i32MaxIuvwOffsetBias;
    f32Temp = (stcCfg.f32DetectDelayTimeMs * stcCfg.f32Fs / 1000.0f);
    pstc->i32DelayTime = (int32_t) f32Temp;
    f32Temp = (stcCfg.f32OffsetCheckTimeMs * stcCfg.f32Fs / 1000.0f);
    pstc->i32OffsetDetectTime = (int32_t)f32Temp;
    pstc->i32IuOffset = 0;
    pstc->i32IvOffset = 0;
    pstc->i32IwOffset = 0;
    pstc->i32OffsetDetectCnt = 0;
    pstc->bAdcRdy = FALSE;
}

/**
 ******************************************************************************
 ** \brief ADC sampling, sample motor current and DC bus voltage
 **
 ** \param [in] *pstc: pointer to sample structure
 **
 ** \retval     none
 ******************************************************************************/
void Adc_Sample(stc_adc_sample_t *pstc)
{
//    int32_t i32AdcValue;
//    uint32_t u32AdcValue;

    /***************************************************************************
     * wait till ADC sample is finished
     **************************************************************************/
    while (0U == MTR1_ADC_STATE)
    {
        if (MTR1_bTIM4x->CCSR_b.IRQZF == 1U)
        {
            g_stcMotorRunPara.u32FaultCode |= ERR_DMA_FAIL;  /** error code */
            return;
        }
    }

    MTR1_ADC_EOC_CLR;

#if(TRUE == EN_VSP_CMD)
    int32_t i32AdcSpdSet = (int32_t)Adc_u16AdcResult[ADC_CH_VSP];
    g_stcMotorAnaSpdSet.i32AdcValue = i32AdcSpdSet;
#endif

    g_stcMotorRunPara.i32Q12_Vbus = (((pstc->i32Q24_VbusK * (int32_t)Adc_u16AdcResult[ADC_CH_VDC]) >> 12));
    /***************************************************************************
     * change motor phase sequence according to rotating direction
     **************************************************************************/
#if(FALSE == SWAP_ROTOR_DIR)
    if (ROTOR_DIR_CW == g_stcMotorRunPara.i32TgtWrDir)
    {
        g_stcMotorIuvw.i32Q12_V = ((pstc->i32Q24_IuvwK * (pstc->i32IuOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IU])) >> 12);
        g_stcMotorIuvw.i32Q12_U = ((pstc->i32Q24_IuvwK * (pstc->i32IvOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IV])) >> 12);
    }
    else
    {
        g_stcMotorIuvw.i32Q12_U = ((pstc->i32Q24_IuvwK * (pstc->i32IuOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IU])) >> 12);
        g_stcMotorIuvw.i32Q12_V = ((pstc->i32Q24_IuvwK * (pstc->i32IvOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IV])) >> 12);
    }
#else
    if (ROTOR_DIR_CCW == g_stcMotorRunPara.i32TgtWrDir)
    {
        g_stcMotorIuvw.i32Q12_V = ((pstc->i32Q24_IuvwK * (pstc->i32IuOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IU])) >> 12);
        g_stcMotorIuvw.i32Q12_U = ((pstc->i32Q24_IuvwK * (pstc->i32IvOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IV])) >> 12);
    }
    else
    {
        g_stcMotorIuvw.i32Q12_U = ((pstc->i32Q24_IuvwK * (pstc->i32IuOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IU])) >> 12);
        g_stcMotorIuvw.i32Q12_V = ((pstc->i32Q24_IuvwK * (pstc->i32IvOffset - (int32_t)Adc_u16AdcResult[ADC_CH_IV])) >> 12);
    }
#endif
    g_stcMotorIuvw.i32Q12_W = -(g_stcMotorIuvw.i32Q12_U + g_stcMotorIuvw.i32Q12_V);
}

/**
 ******************************************************************************
 ** \brief check the offset of current sampling
 **
 ** \param [in] *pstc: pointer to sample structure
 **
 ** \retval     none
 ******************************************************************************/
void Adc_CheckOffset(stc_adc_sample_t *pstc)
{
    /***************************************************************************
     * wait till ADC sample is finished
     **************************************************************************/
    while (0U == MTR1_ADC_STATE)
    {
        if (MTR1_bTIM4x->CCSR_b.IRQZF == 1U)
        {
            g_stcMotorRunPara.u32FaultCode |= ERR_DMA_FAIL;  /** error code */
            return;
        }
    }

    MTR1_ADC_EOC_CLR;

#if(TRUE == EN_VSP_CMD)
    int32_t i32AdcSpdSet = ((int32_t)Adc_u16AdcResult[ADC_CH_VSP] & 0x0FFF);
//    g_stcMotorAnaSpdSet.i32AdcValue=900;
   g_stcMotorAnaSpdSet.i32AdcValue = i32AdcSpdSet;
#endif

    g_stcMotorRunPara.i32Q12_Vbus = ((pstc->i32Q24_VbusK * ((int32_t)Adc_u16AdcResult[ADC_CH_VDC])) >> 12);

    /***************************************************************************
     * apply time delay before calibrating ADC
     **************************************************************************/
    if (pstc->i32DelayTime > 0)
    {
        pstc->i32DelayTime--;
        return;
    }

    /***************************************************************************
     * calibrating ADC
     **************************************************************************/
    if (pstc->i32OffsetDetectCnt < pstc->i32OffsetDetectTime)
    {
        pstc->i32IuOffset += (int32_t)Adc_u16AdcResult[ADC_CH_IU];
        pstc->i32IvOffset += (int32_t)Adc_u16AdcResult[ADC_CH_IV];
        pstc->i32OffsetDetectCnt++;
    }
    else
    {
        pstc->i32IuOffset = (2 * pstc->i32IuOffset) / pstc->i32OffsetDetectCnt;  // Q1 format
        pstc->i32IvOffset = (2 * pstc->i32IvOffset) / pstc->i32OffsetDetectCnt;  // Q1 format
        pstc->i32IuOffset = ((pstc->i32IuOffset + 1) >> 1); // rounding offset
        pstc->i32IvOffset = ((pstc->i32IvOffset + 1) >> 1); // rounding offset

        int32_t i32_TempU = abs(pstc->i32IuOffset - pstc->i32RefOffset);
        int32_t i32_TempV = abs(pstc->i32IvOffset - pstc->i32RefOffset);

        if ((i32_TempU > pstc->i32MaxOffsetBias) || (i32_TempV > pstc->i32MaxOffsetBias))
        {
///////////////////////////////////////////////////////////////////////
              i32_TempU=pstc->i32MaxOffsetBias;
              i32_TempV=pstc->i32MaxOffsetBias;
             pstc->bAdcRdy = TRUE;
            g_stcMotorRunPara.u32FaultCode |= ERR_AD_OFFSET;
        } else {
            pstc->bAdcRdy = TRUE;
        }
    }
}

