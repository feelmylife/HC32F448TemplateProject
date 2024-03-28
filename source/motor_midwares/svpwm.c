/**
 *******************************************************************************
 * @file  svpwm.c
 * @brief This file contains svpwm calculate function.
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
#include "motor_midwares/svpwm.h"

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
 ** \brief initialize SVPWM calculation
 **
 ** \param [in] i32PeakCnt: peak value of FRT in up-down counting mode
 ** \param [in] i32Q15_MaxDuty: maximum duration ratio <= 0.98
 ** \param [in] *pstc: SVPWM calculation data structure
 **
 ** \retval     TRUE: successfully initialized
 **             FALSE: wrong peak count of up-down counter
 ******************************************************************************/
boolean_t Svpwm_InitCalc(int32_t i32PeakCnt, int32_t i32Q15_MaxDuty,
                         stc_svpwm_calc_t *pstc)
{
    if ((i32PeakCnt < 1) || (i32PeakCnt > 0x0ffff)) {
        return FALSE;
    } else {
        pstc->i32PeakCnt = i32PeakCnt;
        /** limit maximum duty: <80% is allowed *******************************/
        if (i32Q15_MaxDuty >= Q15(0.8)) {
            pstc->i32Q15_MaxDutyCnt = Q15(0.8) * i32PeakCnt;
        } else {
            pstc->i32Q15_MaxDutyCnt = i32Q15_MaxDuty * i32PeakCnt;
        }
    }

    pstc->i32Q12_VaReal = 0;
    pstc->i32Q12_VbReal = 0;
    pstc->u8Sector = 0;
    pstc->u8OvFlag = 0;
    pstc->u16T0 = 0xffff;
    pstc->u16T1 = 0;
    pstc->u16T2 = 0;
    return TRUE;
}

/**
 ******************************************************************************
 ** \brief calculate duration time of voltage vectors by SVPWM
 **
 ** \param [in] i32Q12_Va: voltage in alpha-axis,  -8.0 < Va < 8.0
 ** \param [in] i32Q12_Vb: voltage in beta-axis,   -8.0 < Vb < 8.0
 ** \param [in] i32Q12_Vbus: DC bus voltage,       -16.0 < Vbus < 16.0
 ** \param [in] *pstc: SVPWM calculation data structure
 **
 ** \retval     none
 ******************************************************************************/
void Svpwm_CalcDuration(int32_t  i32Q12_Va, int32_t i32Q12_Vb,
                        int32_t  i32Q12_Vbus, stc_svpwm_calc_t *pstc)
{
    int32_t i32Q12_Vab;
    int32_t i32Q12_Vbc;
    int32_t i32Q12_Vca;
    int32_t i32Q12_V1;
    int32_t i32Q12_V2;
    int32_t i32Q15_InvVbus;
    int32_t i32T1;
    int32_t i32T2;
    int32_t i32Temp;

    /***************************************************************************
    ** voltage transformation: balanced voltage which has: Vab + Vbc + Vca = 0
    **  Vbc = sqrt(3)*Vbeta
    **  Vab = 1.5*Valpha - 0.5*Vbc
    **  Vca = -Vab - Vbc
    ***************************************************************************/
    i32Q12_Vbc = ((i32Q12_Vb * Q15_SQRT3 + Q14_BASE) >> 15);
    i32Q12_Vab = ((i32Q12_Va * Q15_1P5 + Q14_BASE) >> 15) - ((i32Q12_Vbc + 1) >> 1);
    i32Q12_Vca = -(i32Q12_Vab + i32Q12_Vbc);

    /***************************************************************************
    ** determine voltage sector and base voltage vector
    ***************************************************************************/
    if (i32Q12_Vab >= 0) {
        if (i32Q12_Vbc >= 0) {
            pstc->u8Sector = 1;
            i32Q12_V1 = i32Q12_Vab;
            i32Q12_V2 = i32Q12_Vbc;
        } else {
            if (i32Q12_Vca >= 0) {
                pstc->u8Sector = 5;
                i32Q12_V1 = i32Q12_Vca;
                i32Q12_V2 = i32Q12_Vab;
            } else {
                pstc->u8Sector = 6;
                i32Q12_V1 = -i32Q12_Vca;
                i32Q12_V2 = -i32Q12_Vbc;
            }
        }
    } else {
        if (i32Q12_Vbc <= 0) {
            pstc->u8Sector = 4;
            i32Q12_V1 = -i32Q12_Vbc;
            i32Q12_V2 = -i32Q12_Vab;
        } else {
            if (i32Q12_Vca <= 0) {
                pstc->u8Sector = 2;
                i32Q12_V1 = -i32Q12_Vab;
                i32Q12_V2 = -i32Q12_Vca;
            } else {
                pstc->u8Sector = 3;
                i32Q12_V1 = i32Q12_Vbc;
                i32Q12_V2 = i32Q12_Vca;
            }
        }
    }

    /***************************************************************************
    ** calculate duration time of base voltage, and limit the maximum duty count
    ***************************************************************************/
    i32Q15_InvVbus = Q27_BASE / i32Q12_Vbus;
    i32T1 = ((((i32Q12_V1 * i32Q15_InvVbus + Q11_BASE) >> 12) * pstc->i32PeakCnt) >> 15);
    i32T2 = ((((i32Q12_V2 * i32Q15_InvVbus + Q11_BASE) >> 12) * pstc->i32PeakCnt) >> 15);

    if (Q15(i32T1 + i32T2) > pstc->i32Q15_MaxDutyCnt) {
        int32_t i32Q15_K = (pstc->i32Q15_MaxDutyCnt) / (i32T1 + i32T2);
        i32T1 = ((i32Q15_K * i32T1) >> 15);
        i32T2 = ((i32Q15_K * i32T2) >> 15);
        pstc->i32Q12_VaReal = ((i32Q15_K * i32Q12_Va) >> 15);
        pstc->i32Q12_VbReal = ((i32Q15_K * i32Q12_Vb) >> 15);
        pstc->u8OvFlag = 1;
    }

    else {
        pstc->i32Q12_VaReal = i32Q12_Va;
        pstc->i32Q12_VbReal = i32Q12_Vb;
        pstc->u8OvFlag = 0;
    }

    pstc->u16T1 = (uint16_t)(i32T1);
    pstc->u16T2 = (uint16_t)(i32T2);
    i32Temp = pstc->i32PeakCnt - (int32_t)pstc->u16T1 - (int32_t)pstc->u16T2;
    pstc->u16T0 = (uint16_t)i32Temp;
}

/**
 ******************************************************************************
 ** \brief align voltage pulse by symmetric 7-segment method
 **
 ** \param [in] u8Sector: sector of voltage vector
 ** \param [in] u16T0: duration of zero voltage vector
 ** \param [in] u16T1: duration of first base voltage vector
 ** \param [in] u16T2: duration of second base voltage vector
 ** \param [in] *pstc: switch time of each inverter leg
 **
 ** \retval     none
 ******************************************************************************/
void Svpwm_7SegSymmPwm(uint8_t u8Sector, uint16_t u16T0, uint16_t u16T1,
                       uint16_t u16T2, stc_pwm_gen_t *pstc)
{
    /** sector 1: [1, 0, 0] -> [1, 1, 0] **************************************/
    if (1U == u8Sector)
    {
        pstc->u16Uon = (u16T0 >> 1U);
        pstc->u16Von = pstc->u16Uon + u16T1;
        pstc->u16Won = pstc->u16Von + u16T2;
    }
    /** sector 2: [0, 1, 0] -> [1, 1, 0] **************************************/
    else if (2U == u8Sector)
    {
        pstc->u16Von = (u16T0 >> 1);
        pstc->u16Uon = pstc->u16Von + u16T1;
        pstc->u16Won = pstc->u16Uon + u16T2;
    }
    /** sector 3: [0, 1, 0] ->[0, 1, 1] ***************************************/
    else if (3U == u8Sector)
    {
        pstc->u16Von = (u16T0 >> 1);
        pstc->u16Won = pstc->u16Von + u16T1;
        pstc->u16Uon = pstc->u16Won + u16T2;
    }
    /** sector 4: [0, 0, 1] -> [0, 1, 1] **************************************/
    else if (4U == u8Sector)
    {
        pstc->u16Won = (u16T0 >> 1);
        pstc->u16Von = pstc->u16Won + u16T1;
        pstc->u16Uon = pstc->u16Von + u16T2;
    }
    /** sector 5: [0, 0, 1] -> [1, 0, 1]***************************************/
    else if (5U == u8Sector)
    {
        pstc->u16Won = (u16T0 >> 1);
        pstc->u16Uon = pstc->u16Won + u16T1;
        pstc->u16Von = pstc->u16Uon + u16T2;
    }
    /** sector 6: [1, 0, 0] -> [1, 0, 1] **************************************/
    else if (6U == u8Sector)
    {
        pstc->u16Uon = (u16T0 >> 1);
        pstc->u16Won = pstc->u16Uon + u16T1;
        pstc->u16Von = pstc->u16Won + u16T2;
    }
    else
    {
        pstc->u16Uon = 0xffff;
        pstc->u16Von = 0xffff;
        pstc->u16Won = 0xffff;
    }
    pstc->u16Uoff = pstc->u16Uon;
    pstc->u16Voff = pstc->u16Von;
    pstc->u16Woff = pstc->u16Won;
}

/**
 ******************************************************************************
 ** \brief align voltage pulse by symmetric 5-segment method
 **
 ** \param [in] u8Sector: sector of voltage vector
 ** \param [in] u16T0: duration of zero voltage vector
 ** \param [in] u16T1: duration of first base voltage vector
 ** \param [in] u16T2: duration of second base voltage vector
 ** \param [in] *pstc: switch time of each inverter leg
 **
 ** \retval     none
 ******************************************************************************/
void Svpwm_5SegSymmPwm(uint8_t u8Sector, uint16_t u16T0, uint16_t u16T1,
                       uint16_t u16T2, stc_pwm_gen_t *pstc)
{
}
