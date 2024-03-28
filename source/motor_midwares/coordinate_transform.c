/**
 *******************************************************************************
 * @file  coordinate_transform.c
 * @brief This file contains coordinate transform function.
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
#include "motor_midwares/coordinate_transform.h"

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
 ** \brief: CLARKE transform: u-v-w => alpha-beta
 **         three-phase stationary variables to two-phase stationary variables
 **
 ** \param [in] stc_uvw_t stcuvw: three-phase variables
 ** \param [in] stc_ab_t* pstcab: pointer to two-phase variables
 **
 ** \retval     none
 ******************************************************************************/
void Transf_Clarke(stc_uvw_t stcuvw, stc_ab_t *pstcab)
{
    pstcab->i32Q12_Alpha = stcuvw.i32Q12_U;
    pstcab->i32Q12_Beta = (Q15_INV_SQRT3 * stcuvw.i32Q12_U + Q15_2INV_SQRT3 * stcuvw.i32Q12_V) >> 15;
}

/**
 *******************************************************************************
 ** \brief: INVERSE CLARKE transform: alpha-beta => u-v-w
 **         two-phase stationary variables to three-phase stationary variables
 **
 ** \param [in] stc_ab_t stcab: two-phase variables
 ** \param [in] stc_uvw_t* pstcuvw: pointer to three-phase variables
 **
 ** \retval     none
 ******************************************************************************/
void Transf_InvClarke(stc_ab_t stcab, stc_uvw_t *pstcuvw)
{
    pstcuvw->i32Q12_U = stcab.i32Q12_Alpha;
    pstcuvw->i32Q12_V = (((Q15_SQRT3 * stcab.i32Q12_Beta) >> 15) - stcab.i32Q12_Alpha) >> 1;
    pstcuvw->i32Q12_W = - (pstcuvw->i32Q12_U + pstcuvw->i32Q12_V);
}

/**
 *******************************************************************************
 ** \brief: PARK transform: alpha-beta => d-q
 **         two-phase stationary variables to two-phase rotation variables
 **
 ** \param [in] stc_ab_t stcab: two-phase stationary variables
 ** \param [in] stc_dq_t* pstcdq: pointer to two-phase rotation variables
 **
 ** \retval     none
 ******************************************************************************/
void Transf_Park(stc_ab_t stcab, stc_dq_t *pstcdq)
{
    pstcdq->i32Q12_D = (stcab.i32Q12_Alpha * pstcdq->i32Q15_Cos + stcab.i32Q12_Beta * pstcdq->i32Q15_Sin) >> 15;
    pstcdq->i32Q12_Q = (-stcab.i32Q12_Alpha * pstcdq->i32Q15_Sin + stcab.i32Q12_Beta * pstcdq->i32Q15_Cos) >> 15;
}

/**
 *******************************************************************************
 ** \brief: INVERSE PARK transform: d-q => alpha-beta
 **         two-phase rotation variables to two-phase stationary variables
 **
 ** \param [in] stc_dq_t stcdq: two-phase rotation variables
 ** \param [in] stc_ab_t* pstcab: pointer to two-phase stationary variables
 **
 ** \retval     none
 ******************************************************************************/
void Transf_InvPark(stc_dq_t stcdq, stc_ab_t *pstcab)
{
    pstcab->i32Q12_Alpha = (stcdq.i32Q12_D * stcdq.i32Q15_Cos - stcdq.i32Q12_Q * stcdq.i32Q15_Sin) >> 15;
    pstcab->i32Q12_Beta = (stcdq.i32Q12_D * stcdq.i32Q15_Sin + stcdq.i32Q12_Q * stcdq.i32Q15_Cos) >> 15;
}
