/**
 *******************************************************************************
 * @file  svpwm.h
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
#ifndef MOTOR_MIDWARES_SVPWM_H_
#define MOTOR_MIDWARES_SVPWM_H_

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

/** SVPWM *********************************************************************/
typedef struct {
    int32_t i32Q12_VaReal;
    int32_t i32Q12_VbReal;
    int32_t i32Q15_MaxDutyCnt;
    int32_t i32PeakCnt;
    uint8_t u8Sector;
    uint8_t u8OvFlag;
    uint16_t u16T0;
    uint16_t u16T1;
    uint16_t u16T2;
} stc_svpwm_calc_t;

/** pulse alignment of PWM ****************************************************/
typedef struct {
    uint16_t u16Uon;
    uint16_t u16Uoff;
    uint16_t u16Von;
    uint16_t u16Voff;
    uint16_t u16Won;
    uint16_t u16Woff;
} stc_pwm_gen_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
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
extern boolean_t Svpwm_InitCalc(int32_t i32PeakCnt, int32_t i32Q15_MaxDuty,
                                stc_svpwm_calc_t *pstc);

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
extern void Svpwm_CalcDuration(int32_t  i32Q12_Va, int32_t i32Q12_Vb,
                               int32_t  i32Q12_Vbus, stc_svpwm_calc_t *pstc);

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
extern void Svpwm_7SegSymmPwm(uint8_t u8Sector, uint16_t u16T0, uint16_t u16T1,
                              uint16_t u16T2, stc_pwm_gen_t *pstc);

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
extern void Svpwm_5SegSymmPwm(uint8_t u8Sector, uint16_t u16T0, uint16_t u16T1,
                              uint16_t u16T2, stc_pwm_gen_t *pstc);

#endif /* MTOR_MIDWARES_SVPWM_H_ */
