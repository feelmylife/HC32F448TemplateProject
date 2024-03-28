/**
 *******************************************************************************
 * @file  pid.c
 * @brief This file contains pid control function.
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
#include "motor_midwares/pid.h"

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
 ******************************************************************************
 ** \brief initialize incremental PI regulator
 **
 ** \param [in] stc_pid_cfg_t stcCfg: configuration parameters
 **             stc_inc_pid_t* pstc: pointer to PID regulator
 **
 ** \retval     none
 ******************************************************************************/
void PID_InitIncPid(stc_pid_cfg_t stcCfg, stc_inc_pid_t *pstc)
{
    pstc->i32Q15_Kp = Q15(stcCfg.f32Kp);
    pstc->i32Q30_Ki = Q30(stcCfg.f32Ki / stcCfg.f32Fs);
    pstc->i32Q27_OutMin = Q27(stcCfg.f32OutMin);
    pstc->i32Q27_OutMax = Q27(stcCfg.f32OutMax);
    pstc->i32Q12_AbsErrMax = Q12(stcCfg.f32ErrMax);
    pstc->i32Q27_Output = Q27(stcCfg.f32Y0);
    pstc->i32Q12_Err1 = 0;
}

/**
 ******************************************************************************
 ** \brief incremental PI regulator
 **
 ** \param [in] int32_t i32Q12_Ref:  reference
 **             int32_t i32Q12_Fb:   feedback
 **             stc_inc_pid_t* pstc: pointer to PID regulator
 **
 ** \retval     int32_t i32Q12_Output
 ******************************************************************************/
int32_t PID_IncPid(int32_t i32Q12_Ref, int32_t i32Q12_Fb, stc_inc_pid_t *pstc)
{
    int32_t i32Q12_Err = i32Q12_Ref - i32Q12_Fb;
    int32_t i32Q42_temp;
    int32_t i32Q27_temp;

    /** limit maximum error ***************************************************/
    if (i32Q12_Err > pstc->i32Q12_AbsErrMax)
    {
        i32Q12_Err = pstc->i32Q12_AbsErrMax;
    }
    else if (i32Q12_Err < -pstc->i32Q12_AbsErrMax)
    {
        i32Q12_Err = -pstc->i32Q12_AbsErrMax;
    }
    else
    {
        i32Q12_Err = i32Q12_Ref - i32Q12_Fb;
    }
    /** PI regulation considering round-off error of I-regulator **************/
    pstc->i32Q27_Output += pstc->i32Q15_Kp * (i32Q12_Err - pstc->i32Q12_Err1);
                                                                      // Kp term
    pstc->i32Q27_Output += (pstc->i32Q30_Ki>>15) * i32Q12_Err;
                                                   // multiply higher bits of Ki
    i32Q42_temp = (pstc->i32Q30_Ki & 0x7FFF) * i32Q12_Err;
                                                 // multiply lower 15 bits of Ki
    i32Q42_temp += pstc->i32Q42_Tail;
                                                 // add previous round-off error
    i32Q27_temp = i32Q42_temp>>15;
    pstc->i32Q42_Tail = i32Q42_temp - (i32Q27_temp<<15);
                                                       // update round-off error
    pstc->i32Q27_Output += i32Q27_temp;
                                                          // update final output

    if (pstc->i32Q27_Output > pstc->i32Q27_OutMax)
    {
        pstc->i32Q27_Output = pstc->i32Q27_OutMax;
    }
    else if (pstc->i32Q27_Output < pstc->i32Q27_OutMin)
    {
        pstc->i32Q27_Output = pstc->i32Q27_OutMin;
    }
    else
    {
        pstc->i32Q27_Output = pstc->i32Q27_Output;
    }

    pstc->i32Q12_Err1 = i32Q12_Err;
    return (pstc->i32Q27_Output >> 15);
}

/**
 ******************************************************************************
 ** \brief initialize position PI regulator
 **
 ** \param [in] stc_pid_cfg_t stcCfg: configuration parameters
 **             stc_inc_pid_t* pstc: pointer to PID regulator
 **
 ** \retval     none
 ******************************************************************************/
void PID_InitPosPid(stc_pid_cfg_t stcCfg, stc_pos_pid_t *pstc)
{
    pstc->i32Q15_Kp = Q15(stcCfg.f32Kp);
    pstc->i32Q30_Ki = Q30(stcCfg.f32Ki / stcCfg.f32Fs);
    pstc->i32Q27_OutMin = Q27(stcCfg.f32OutMin);
    pstc->i32Q27_OutMax = Q27(stcCfg.f32OutMax);
    pstc->i32Q12_AbsErrMax = Q12(stcCfg.f32ErrMax);
    pstc->i32Q27_Output = Q27(stcCfg.f32Y0);
    pstc->i32Q27_Iout = pstc->i32Q27_Output;
    pstc->i32Q12_Err1 = 0;
}

/**
 ******************************************************************************
 ** \brief position PI regulator
 **
 ** \param [in] int32_t i32Q12_Ref:  reference
 **             int32_t i32Q12_Fb:   feedback
 **             stc_inc_pid_t* pstc: pointer to PID regulator
 **
 ** \retval     int32_t i32Q12_Output
 ******************************************************************************/
int32_t PID_PosPid(int32_t i32Q12_Ref, int32_t i32Q12_Fb, stc_pos_pid_t *pstc)
{
    int32_t i32Q12_Err = i32Q12_Ref - i32Q12_Fb;
    int32_t i32Q42_temp;
    int32_t i32Q27_temp;

    /** limit maximum error ***************************************************/
    if (i32Q12_Err > pstc->i32Q12_AbsErrMax)
    {
        i32Q12_Err = pstc->i32Q12_AbsErrMax;
    }
    else if (i32Q12_Err < -pstc->i32Q12_AbsErrMax)
    {
        i32Q12_Err = -pstc->i32Q12_AbsErrMax;
    }
    else
    {
        i32Q12_Err = i32Q12_Ref - i32Q12_Fb;
    }

    if (0 == pstc->i32Q30_Ki)
    {
        pstc->i32Q27_Iout = 0;
        pstc->i32Q42_Tail = 0;
    }
    else
    {
        /** I-term regulation considering round-off error of I-regulator **********/
        pstc->i32Q27_Iout += (pstc->i32Q30_Ki >> 15) * i32Q12_Err;  // multiply higher bits of Ki
        i32Q42_temp = (pstc->i32Q30_Ki & 0x7FFF) * i32Q12_Err;      // multiply lower 15 bits of Ki
        i32Q42_temp += pstc->i32Q42_Tail;                           // add previous round-off error
        i32Q27_temp = i32Q42_temp >> 15;
        pstc->i32Q42_Tail = i32Q42_temp - (i32Q27_temp << 15);
        pstc->i32Q27_Iout += i32Q27_temp;                           // update round-off error

        /** I-regulator output limitation *****************************************/
        if (pstc->i32Q27_Iout > pstc->i32Q27_OutMax) {
            pstc->i32Q27_Iout = pstc->i32Q27_OutMax;
            pstc->i32Q42_Tail = 0;
        }
        else if (pstc->i32Q27_Iout < pstc->i32Q27_OutMin)
        {
            pstc->i32Q27_Iout = pstc->i32Q27_OutMin;
            pstc->i32Q42_Tail = 0;
        }
        else
        {
            ;
        }
    }

    /** sum of P and I regulator **********************************************/
    pstc->i32Q27_Output = pstc->i32Q15_Kp * i32Q12_Err + pstc->i32Q27_Iout;

    /** output limitation *****************************************************/
    if (pstc->i32Q27_Output > pstc->i32Q27_OutMax)
    {
        pstc->i32Q27_Output = pstc->i32Q27_OutMax;
    }
    else if (pstc->i32Q27_Output < pstc->i32Q27_OutMin)
    {
        pstc->i32Q27_Output = pstc->i32Q27_OutMin;
    }
    else
    {
        pstc->i32Q27_Output = pstc->i32Q27_Output;
    }

    pstc->i32Q12_Err1 = i32Q12_Err;
    return (pstc->i32Q27_Output >> 15);
}
