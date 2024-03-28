/**
 *******************************************************************************
 * @file  stop_control.c
 * @brief This file contains motor stop control function.
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

#ifndef MOTOR_MIDWARES_STOP_CONTROL_H_
#define MOTOR_MIDWARES_STOP_CONTROL_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/module_include.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/
typedef enum {
    Stop_Initialize = 1,
    Stop_DecCurrent = 2,    // apply slop function to decrease current to zero
    Stop_ElecBrake  = 3,    // apply electric brake to stop motor
    Stop_ShortBrake = 4,
    Stop_Finish     = 5,
} en_stop_status_t;

typedef struct {
    float32_t f32Fs;                // sample frequency of this function
    boolean_t bEnDecCurrentStop;    // enable decrease current to zero, then stop motor
    boolean_t bEnBrakeStop;         // enable electric brake function to stop motor
    float32_t f32DeltaIdPerSec;     // (p.u./second) current decrease slop
    float32_t f32DeltaIqPerSec;     // (p.u./second) current decrease slop
    float32_t f32MaxCurrentDecayTimeMs;
    float32_t f32MaxBrakeTimeMs;
} stc_stop_ctrl_cfg_t;

typedef struct {
    en_stop_status_t enStatus;
    boolean_t bEnDecCurrentStop;
    boolean_t bEnBrakeStop;
    uint8_t u8ShortBrakeStatus;
    int32_t i32TimerCnt;
    int32_t i32MaxCurrentDecTime;
    int32_t i32MaxBrakeTime;
    int32_t i32Q24_IdRef;
    int32_t i32Q24_IqRef;
    int32_t i32Q24_DeltaId;
    int32_t i32Q24_DeltaIq;
} stc_stop_ctrl_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize stop control function
 **
 ** \param [in] stc_stop_ctrl_cfg_t stcCfg: configuration parameters
 ** \param [in] stc_stop_ctrl_t* pstc: pointer to stop-control structure
 **
 ** \retval     none
 ******************************************************************************/
extern void Stop_Init(stc_stop_ctrl_cfg_t stcCfg, stc_stop_ctrl_t *pstc);

/**
 ******************************************************************************
 ** \brief stop motor control process
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Stop_Control(stc_stop_ctrl_t *pstc);

#endif /* MOTOR_MIDWARES_STOP_CONTROL_H_ */
