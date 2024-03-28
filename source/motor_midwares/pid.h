/**
 *******************************************************************************
 * @file  pid.h
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
#ifndef MOTOR_MIDWARES_PID_H_
#define MOTOR_MIDWARES_PID_H_

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

/** structure for initializing incremental PID ********************************/
typedef struct {
    float32_t f32Kp;        /** proportion gain */
    float32_t f32Ki;        /** integration gain */
    float32_t f32Y0;        /** initial output */
    float32_t f32OutMin;    /** maximum output */
    float32_t f32OutMax;    /** minimum output */
    float32_t f32ErrMax;    /** maximum error */
    float32_t f32Fs;        /** sample frequency of PID */
} stc_pid_cfg_t;

/** incremental PID ***********************************************************/
typedef struct {
    int32_t i32Q15_Kp;          /** Q-format proprotion gain */
    int32_t i32Q30_Ki;          /** Q-format integration gain */
    int32_t i32Q42_Tail;        /** Q-format tail */
    int32_t i32Q27_Output;      /** Q-format output */
    int32_t i32Q27_OutMin;      /** Q-format minimum output */
    int32_t i32Q27_OutMax;      /** Q-format maximum output */
    int32_t i32Q12_Err1;        /** Q-format last error */
    int32_t i32Q12_AbsErrMax;   /** Q-format maximum absolute error */
} stc_inc_pid_t;

/** incremental PID ***********************************************************/
typedef struct {
    int32_t i32Q15_Kp;          /** Q-format proportion gain */
    int32_t i32Q30_Ki;          /** Q-format integration gain */
    int32_t i32Q42_Tail;        /** Q-format tail */
    int32_t i32Q27_Output;      /** Q-format output */
    int32_t i32Q27_OutMin;      /** Q-format minumum output */
    int32_t i32Q27_OutMax;      /** Q-format maximum output */
    int32_t i32Q27_Iout;        /** Q-format store integration value */
    int32_t i32Q12_Err1;        /** Q-format last error */
    int32_t i32Q12_AbsErrMax;   /** Q-format maximum absolute error */
} stc_pos_pid_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize incremental PI regulator
 **
 ** \param [in] stc_pid_cfg_t stcCfg: configuration parameters
 **             stc_inc_pid_t* pstc: pointer to PID regulator
 **
 ** \retval     none
 ******************************************************************************/
extern void PID_InitIncPid(stc_pid_cfg_t stcCfg, stc_inc_pid_t *pstc);

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
extern int32_t PID_IncPid(int32_t i32Q12_Ref, int32_t i32Q12_Fb, stc_inc_pid_t *pstc);

/**
 ******************************************************************************
 ** \brief initialize position PI regulator
 **
 ** \param [in] stc_pid_cfg_t stcCfg: configuration parameters
 **             stc_inc_pid_t* pstc: pointer to PID regulator
 **
 ** \retval     none
 ******************************************************************************/
extern void PID_InitPosPid(stc_pid_cfg_t stcCfg, stc_pos_pid_t *pstc);

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
extern int32_t PID_PosPid(int32_t i32Q12_Ref, int32_t i32Q12_Fb, stc_pos_pid_t *pstc);

#endif /* MOTOR_MIDWARES_PID_H_ */
