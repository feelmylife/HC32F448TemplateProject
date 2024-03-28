/**
 *******************************************************************************
 * @file  flux_observer.c
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

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_midwares/flux_observer.h"

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
 ** \brief initialized flux observer
 **
 ** \param [in] const stc_flux_obs_cfg_t stcCfg: config value of flux observer
 ** \param [in] stc_Flux_Observer_t* pstc: pointer of flux observer
 **
 ** \retval     none
 ******************************************************************************/
void Flux_Init(const stc_flux_obs_cfg_t stcCfg, stc_flux_observer_t *pstc)
{
    pstc->i32Q15_GammaHalf = Q15(stcCfg.f32GammaHalf);
    pstc->i32Q15_Rs = Q15(stcCfg.f32Rs);
    pstc->i32Q15_L0 = Q15(stcCfg.f32L0);
    pstc->i32Q15_L1 = Q15(stcCfg.f32L1);
    pstc->i32Q15_LammbdaF = Q15(stcCfg.f32LambdaF);
    pstc->i32Q20_Ts = Q20(CONST_F32_2PI * stcCfg.f32FB / stcCfg.f32Fs);
    pstc->i32Q15_PLLKp = Q12(stcCfg.f32PLLKp);
    pstc->i32Q15_PLLKi = Q15(stcCfg.f32PLLKi);

    stc_1stLpf_cfg_t stcLpfCfg;
    stcLpfCfg.f32Fs = stcCfg.f32Fs;
    stcLpfCfg.f32Fc = (float32_t)stcCfg.i32WrLpfFc;
    stcLpfCfg.f32Y0 = 0.0f;
    Filter_InitFirstLpf(stcLpfCfg, &(pstc->stcWrLpf));

    pstc->i32Q12_RealTimeWr = 0;
    pstc->i32Q24_WrMidVar = 0;
    pstc->i32Q24_PLLIout = 0;
    pstc->i32Q24_ThetaEst = 0;
    pstc->i32Q15_x1 = 0;
    pstc->i32Q15_x1_dot = 0;
    pstc->i32Q15_x2 = 0;
    pstc->i32Q15_x2_dot = 0;
    pstc->i32Q24_Theta = 0;
}

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


void Flux_Observer(const stc_ab_t stcVab, const stc_ab_t stcIab, stc_flux_observer_t *pstc)
{
    /** calculate cosine and sine value of last estimate angle  */
    int32_t COS_2_theta = Math_Cos((2 * pstc->i32Q24_ThetaEst) & 0xFFFFFF);
    int32_t SIN_2_theta = Math_Sin((2 * pstc->i32Q24_ThetaEst) & 0xFFFFFF);

    /** IPMSM inductor matrix is not same to SPMSM, this is for IPMSM */
   
    int32_t Q15_L_ia = (((pstc->i32Q15_L0 + ((pstc->i32Q15_L1 * COS_2_theta) >> 15)) * stcIab.i32Q12_Alpha + ((pstc->i32Q15_L1 * SIN_2_theta) >> 15) * stcIab.i32Q12_Beta) >> 12);
    int32_t Q15_L_ib = ((((pstc->i32Q15_L1 * SIN_2_theta) >> 15) * stcIab.i32Q12_Alpha + (pstc->i32Q15_L0 - ((pstc->i32Q15_L1 * COS_2_theta) >> 15)) * stcIab.i32Q12_Beta) >> 12);



    int32_t Q15_R_ia = ((pstc->i32Q15_Rs * stcIab.i32Q12_Alpha) >> 12);
    int32_t Q15_R_ib = ((pstc->i32Q15_Rs * stcIab.i32Q12_Beta) >> 12);

    int32_t Q30_LammbdaF2 = pstc->i32Q15_LammbdaF * pstc->i32Q15_LammbdaF;
    int32_t Q15_Eta_a = pstc->i32Q15_x1 - Q15_L_ia;
    int32_t Q15_Eta_b = pstc->i32Q15_x2 - Q15_L_ib;

    /** calculate error for estimate flux and modol reference flux */
    int32_t Q15_Err = ((Q30_LammbdaF2 - ((Q15_Eta_a) * (Q15_Eta_a) + (Q15_Eta_b) * (Q15_Eta_b))) >> 15);

    if(Q15_Err >= 0)
    {
        Q15_Err = 0;
    }

    /** acroding error, calculate estimate flux, GammaHalf is learn rate */
  
    int32_t Q15_Cmp_x1 = ((((pstc->i32Q15_GammaHalf * Q15_Eta_a) >> 15) * Q15_Err) >> 15);
    int32_t Q15_Cmp_x2 = ((((pstc->i32Q15_GammaHalf * Q15_Eta_b) >> 15) * Q15_Err) >> 15);
    


    /** calculate flux's differential */
    pstc->i32Q15_x1_dot = (stcVab.i32Q12_Alpha << 3) + Q15_Cmp_x1 - Q15_R_ia ;
    pstc->i32Q15_x2_dot = (stcVab.i32Q12_Beta << 3)  + Q15_Cmp_x2 - Q15_R_ib;

    /** integral, x is flux in alpha-beta coordinate  */
    pstc->i32Q15_x1 += (((pstc->i32Q15_x1_dot >> 3) * pstc->i32Q20_Ts) >> 17);
    pstc->i32Q15_x2 += (((pstc->i32Q15_x2_dot >> 3) * pstc->i32Q20_Ts) >> 17);

    /** theta is from arctan function */
    pstc->i32Q24_Theta = Math_Atan2(pstc->i32Q15_x1 - Q15_L_ia, pstc->i32Q15_x2 - Q15_L_ib);
    pstc->i32Q24_Theta &= 0xFFFFFF;

    /** PLL for stable estimate angle */
    int32_t i32Q15_DeltaTheta = Math_Sin((pstc->i32Q24_Theta - pstc->i32Q24_ThetaEst) & 0xFFFFFF);


    int32_t i32Q24_KpDelta = ((pstc->i32Q15_PLLKp * i32Q15_DeltaTheta) >> 3);
    int32_t i32Q24_KiDelta = ((pstc->i32Q15_PLLKi * i32Q15_DeltaTheta) >> 6);

    pstc->i32Q24_PLLIout += (((i32Q24_KiDelta >> 12) * pstc->i32Q20_Ts) >> 8);
    
    /** limit integral value */
    if(i32Q15_DeltaTheta * pstc->i32Q24_PLLIout < 0)
    {
        pstc->i32Q24_PLLIout = 0;
    }
    

    /** get Wr middle variable */
    pstc->i32Q24_WrMidVar = i32Q24_KpDelta + pstc->i32Q24_PLLIout;

    /** integral, get estimate theta */
    pstc->i32Q24_ThetaEst += (((pstc->i32Q24_WrMidVar >> 12) * pstc->i32Q20_Ts) >> 8);

    pstc->i32Q24_ThetaEst &= 0xFFFFFF;


    /** get real-time Wr */
    pstc->i32Q12_RealTimeWr = (((pstc->i32Q24_WrMidVar >> 12) * Q15(CONST_F32_2PI)) >> 15);

    /** low-pass filter to Wr */
    pstc->i32Q12_WrF = Filter_FirstLpf(pstc->i32Q12_RealTimeWr, &(pstc->stcWrLpf));
}





