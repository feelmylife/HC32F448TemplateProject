/**
 *******************************************************************************
 * @file  flux_observer.h
 * @brief This file contains flux observer function.
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

#ifndef MOTOR_MIDWARES_FLUX_OBSERVER_H_
#define MOTOR_MIDWARES_FLUX_OBSERVER_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/module_include.h"

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/
typedef struct {
    float f32GammaHalf;        /** Observer Gain */
    float f32L0;               /** Motor's L_0 */
    float f32L1;               /** Motor's L_1 */
    float f32Rs;               /** motor's R_s */
    float f32LambdaF;          /** motor's flux linkage */
    float f32Fs;               /** Smaple time */
    float f32FB;               /** Frequency base */
    float f32PLLKp;            /** PLL Kp */
    float f32PLLKi;            /** PLL Ki */
    int32_t i32WrLpfFc;         /** cut-off frequency of Wr LPF */
} stc_flux_obs_cfg_t;

typedef struct {
    int32_t i32Q15_x1;
    int32_t i32Q15_x1_dot;
    int32_t i32Q15_x2;
    int32_t i32Q15_x2_dot;
    int32_t i32Q15_GammaHalf;
    int32_t i32Q24_Theta;
    int32_t i32Q24_ThetaEst;
    int32_t i32Q12_RealTimeWr;
    int32_t i32Q24_WrMidVar;
    int32_t i32Q15_PLLKp;
    int32_t i32Q15_PLLKi;
    int32_t i32Q24_PLLIout;
    int32_t i32Q12_WrF;
    int32_t i32Q15_L0;
    int32_t i32Q15_L1;
    int32_t i32Q15_Rs;
    int32_t i32Q15_LammbdaF;
    int32_t i32Q20_Ts;
    stc_1stLpf_t stcWrLpf;
} stc_flux_observer_t;
/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
/**
 *******************************************************************************
 ** \brief initialized flux observer
 **
 ** \param [in] const stc_flux_obs_cfg_t stcCfg: config value of flux observer
 ** \param [in] stc_Flux_Observer_t* pstc: pointer of flux observer
 **
 ** \retval     none
 ******************************************************************************/
extern void Flux_Init(const stc_flux_obs_cfg_t stcCfg, stc_flux_observer_t *pstc);

/**
 *******************************************************************************
 ** \brief flux observer calculate
 **
 ** \param [in] const stc_ab_t stcVab: alpha beta axis voltage
 ** \param [in] const stc_ab_t stcIab: alpha beta axis current
 ** \param [in] stc_Flux_Observer_t *pstc: pointer of flux observer struct
 **
 ** \retval     none
 ******************************************************************************/
extern void Flux_Observer(const stc_ab_t stcVab, const stc_ab_t stcIab, stc_flux_observer_t *pstc);

#endif /* MOTOR_MIDRES_FLUX_OBSERVER_H_ */
