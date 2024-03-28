/**
 *******************************************************************************
 * @file  coordinate_transform.h
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
#ifndef MOTOR_MIDWARES_COORDINATE_TRANSFORM_H_
#define MOTOR_MIDWARES_COORDINATE_TRANSFORM_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/base_types.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/

/******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/

/** three-phase variables in stationary coordinate ****************************/
typedef struct {
    int32_t i32Q12_U;
    int32_t i32Q12_V;
    int32_t i32Q12_W;
} stc_uvw_t;

/** two-phase variables in stationary coordinate ******************************/
typedef struct {
    int32_t i32Q12_Alpha;
    int32_t i32Q12_Beta;
} stc_ab_t;

/** two phase variables in rotation coordinate ********************************/
typedef struct {
    int32_t i32Q12_D;
    int32_t i32Q12_Q;
    int32_t i32Q15_Sin;
    int32_t i32Q15_Cos;
} stc_dq_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
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
extern void Transf_Clarke(stc_uvw_t stcuvw, stc_ab_t *pstcab);

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
extern void Transf_InvClarke(stc_ab_t stcab, stc_uvw_t *pstcuvw);

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
extern void Transf_Park(stc_ab_t stcab, stc_dq_t *pstcdq);

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
extern void Transf_InvPark(stc_dq_t stcdq, stc_ab_t *pstcab);

#endif /* MOTOR_MIDWARES_COORDINATE_TRANSFORM_H_ */
