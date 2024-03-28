/**
 *******************************************************************************
 * @file  filter.h
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
#ifndef MOTOR_MIDWARES_FILTER_H_
#define MOTOR_MIDWARES_FILTER_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/base_types.h"

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/

/** structure for initializing first-order LPF ********************************/
typedef struct {
    float32_t f32Fc;            /** cut-off frequency of LPF (Hz) */
    float32_t f32Fs;            /** sample frequency of LPF (Hz) */
    float32_t f32Y0;            /** initial output of filter */
} stc_1stLpf_cfg_t;

/** first-order low pass filter************************************************/
typedef struct {
    int32_t i32Q15_LpfK;        /** = Wc * Ts */
    int32_t i32Q27_Output;      /** Q27 format output */
    int32_t i32Q12_Output;      /** Q12 format output */
    int32_t i32Q12_Input;       /** Q12 format input(previous) */
} stc_1stLpf_t;

/** structure for initializing periodic average filter ************************/
typedef struct {
    float32_t f32PeriodMs;      /** period of filter */
    float32_t f32Fs;            /** sample frequency of LPF (Hz) */
    float32_t f32Y0;            /** initial output of filter */
} stc_period_avg_cfg_t;

/** periodic average filter ***************************************************/
typedef struct {
    int32_t i32Q12_Avg;         /** average output value */
    int32_t i32Q12_Sum;         /** sample value sum */
    int32_t i32Q15_Cnt;         /** current conter */
    int32_t i32Q15_InvTotalCnt; /** = 1 / TotalCnt */
} stc_period_avg_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 *******************************************************************************
 ** \brief: initialize first order low pass filter
 **
 ** \param [in] stc_1stLpf_cfg_t stcCfg: configuration parameters
 ** \param [in] stc_1stLpf_t* pstc: pointer to low-pass filter
 **
 ** \retval     none
 ******************************************************************************/
extern void Filter_InitFirstLpf(stc_1stLpf_cfg_t stcCfg, stc_1stLpf_t *pstc);

/**
 *******************************************************************************
 ** \brief: first order low-pass filter, parameter limitations:
 **         1. |i32Q12_Input| < Q16_BASE
 **         2. |i32Q15_LpfK| < Q15(1)
 **
 ** \param [in] int32_t i32Q12_Input: input of filter
 ** \param [in] stc_1stLpf_t* pstc: pointer to low-pass filter
 **
 ** \retval     int32_t i32Q12_Output: output of filter
 ******************************************************************************/
extern int32_t Filter_FirstLpf(int32_t i32Q12_Input, stc_1stLpf_t *pstc);

/**
 *******************************************************************************
 ** \brief: reset states of first order low-pass filter
 **
 ** \param [in] int32_t i32Q12_Output:  set filter ouput
 ** \param [in] stc_1stLpf_t *pstc:     pointer of first order low-pass filter
 **
 ** \retval     none
 ******************************************************************************/
extern void Filter_RstFirstLpf(int32_t i32Q12_Output, stc_1stLpf_t *pstc);

/**
 *******************************************************************************
 ** \brief: initialize periodic average filter
 **
 ** \param [in] stc_period_avg_cfg_t stcCfg: configuration parameters
 ** \param [in] stc_period_avg_t *pstc: pointer to periodic average filter
 **
 ** \retval     none
 ******************************************************************************/
extern void Filter_InitPeriodAvg(stc_period_avg_cfg_t stcCfg, stc_period_avg_t *pstc);

/**
 *******************************************************************************
 ** \brief: periodic average filter
 **
 ** \param [in] int32_t i32Q12_Input: input of filter
 ** \param [in] stc_period_avg_t *pstc: pointer to periodic average filter
 **
 ** \retval     int32_t i32Q12_Output: output of filter
 ******************************************************************************/
extern int32_t Filter_PeriodAvg(int32_t i32Q12_Input, stc_period_avg_t *pstc);

/**
 *******************************************************************************
 ** \brief: reset periodic average filter output
 **
 ** \param [in] int32_t i32Q12_Output:  set onput of filter
 ** \param [in] stc_period_avg_t *pstc: pointer to periodic average filter
 **
 ** \retval     none
 ******************************************************************************/
extern void Filter_RstPeriodAvg(int32_t i32Q12_Output, stc_period_avg_t *pstc);

#endif /* MOTOR_MIDWARES_FILTER_H_ */
