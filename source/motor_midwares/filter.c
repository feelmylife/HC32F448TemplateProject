/**
 *******************************************************************************
 * @file  filter.c
 * @brief This file contains digital filtering function.
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
#include "motor_midwares/filter.h"

/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 *******************************************************************************
 ** \brief: initialize first order low pass filter
 **
 ** \param [in] stc_1stLpf_cfg_t stcCfg: configuration parameters
 ** \param [in] stc_1stLpf_t *pstc: pointer to low-pass filter
 **
 ** \retval     none
 ******************************************************************************/
void Filter_InitFirstLpf(stc_1stLpf_cfg_t stcCfg, stc_1stLpf_t *pstc)
{
    pstc->i32Q15_LpfK = Q15(CONST_F32_2PI * stcCfg.f32Fc / stcCfg.f32Fs);
    pstc->i32Q12_Input = 0;

    pstc->i32Q27_Output = Q27(stcCfg.f32Y0);
    pstc->i32Q12_Output = (pstc->i32Q27_Output >> 15);
}

/**
 *******************************************************************************
 ** \brief: first order low-pass filter:
 **             y(n) = y(n-1) + Wc * Ts * [(x(n) + x(n-1)) / 2 - y(n-1)]
 **         parameter limitations:
 **             1. |i32Q12_Input| < Q16_BASE
 **             2. |i32Q15_LpfK| < Q15(1)
 **
 ** \param [in] int32_t i32Q12_Input: input of filter
 ** \param [in] stc_1stLpf_t *pstc: pointer to low-pass filter
 **
 ** \retval     int32_t i32Q12_Output: output of filter
 ******************************************************************************/
int32_t Filter_FirstLpf(int32_t i32Q12_Input, stc_1stLpf_t *pstc)
{
    int32_t i32_temp;

    i32_temp = ((i32Q12_Input + pstc->i32Q12_Input) >> 1) - pstc->i32Q12_Output;
    pstc->i32Q27_Output += (pstc->i32Q15_LpfK * i32_temp);
    pstc->i32Q12_Output = ((pstc->i32Q27_Output + Q14_BASE) >> 15);
    pstc->i32Q12_Input = i32Q12_Input;

    return pstc->i32Q12_Output;
}

/**
 *******************************************************************************
 ** \brief: reset states of first order low-pass filter
 **
 ** \param [in] int32_t i32Q12_Output:  set filter ouput
 ** \param [in] stc_1stLpf_t *pstc:     pointer of first order low-pass filter
 **
 ** \retval     none
 ******************************************************************************/
void Filter_RstFirstLpf(int32_t i32Q12_Output, stc_1stLpf_t *pstc)
{
    pstc->i32Q12_Output = i32Q12_Output;
    pstc->i32Q12_Input = i32Q12_Output;
}

/**
 *******************************************************************************
 ** \brief: initialize periodic average filter
 **
 ** \param [in] stc_period_avg_cfg_t stcCfg: configuration parameters
 ** \param [in] stc_period_avg_t *pstc: pointer to periodic average filter
 **
 ** \retval     none
 ******************************************************************************/
void Filter_InitPeriodAvg(stc_period_avg_cfg_t stcCfg, stc_period_avg_t *pstc)
{
    pstc->i32Q15_InvTotalCnt = Q15(1000.0f / (stcCfg.f32PeriodMs * stcCfg.f32Fs));
    pstc->i32Q12_Avg = Q12(stcCfg.f32Y0);
    pstc->i32Q15_Cnt = 0;
    pstc->i32Q12_Sum = 0;
}

/**
 *******************************************************************************
 ** \brief: periodic average filter
 **
 ** \param [in] int32_t i32Q12_Input: input of filter
 ** \param [in] stc_period_avg_t *pstc: pointer to periodic average filter
 **
 ** \retval     int32_t i32Q12_Output: output of filter
 ******************************************************************************/
int32_t Filter_PeriodAvg(int32_t i32Q12_Input, stc_period_avg_t *pstc)
{
    pstc->i32Q12_Sum += i32Q12_Input;
    pstc->i32Q15_Cnt += (pstc->i32Q15_InvTotalCnt + 1);     /** add 1 to remove quantization error */

    if (pstc->i32Q15_Cnt >= Q15_BASE) {
        pstc->i32Q12_Avg = ((pstc->i32Q12_Sum * pstc->i32Q15_InvTotalCnt) >> 15);
        pstc->i32Q12_Sum = 0;
        pstc->i32Q15_Cnt = 0;
    }
    return pstc->i32Q12_Avg;
}

/**
 *******************************************************************************
 ** \brief: reset periodic average filter output
 **
 ** \param [in] int32_t i32Q12_Output:  set onput of filter
 ** \param [in] stc_period_avg_t *pstc: pointer to periodic average filter
 **
 ** \retval     none
 ******************************************************************************/
void Filter_RstPeriodAvg(int32_t i32Q12_Output, stc_period_avg_t *pstc)
{
    pstc->i32Q12_Avg = i32Q12_Output;
    pstc->i32Q12_Sum = i32Q12_Output;
    pstc->i32Q15_Cnt = 0;
}
