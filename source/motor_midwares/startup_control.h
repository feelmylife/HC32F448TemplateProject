/**
 *******************************************************************************
 * @file  startup_control.h
 * @brief This file contains motor start up control function.
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
#ifndef MOTOR_MIDWARES_STARTUP_CONTROL_H_
#define MOTOR_MIDWARES_STARTUP_CONTROL_H_

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
typedef enum {
    Startup_Initialize          = 0,
    Startup_BscCharge           = 1,
    Startup_IPD                 = 2,
    Startup_Align               = 3,
    Startup_StateDetect         = 4,
    Startup_HeadWindFocBrake    = 5,
    Startup_HeadWindShortBrake  = 6,
    Startup_HeadWindForceDrive  = 7,
    Startup_CLStart             = 10,
    Startup_ForceDrive          = 11,
    Startup_Finish              = 13,
} en_startup_status_t;



typedef struct {
    en_startup_status_t enStatus;
    uint8_t  u8Status_StateDetect;
    uint8_t  u8Status_Align;
    uint8_t  u8Status_HeadWindShortBrake;
    int32_t i32TimerCnt;
    int32_t i32Q20_Ts;
    int32_t i32Q24_MaxStartIq;
    int32_t i32Q27_MaxNormaIq;
    int32_t i32Q12_LowSpdUpper;
    int32_t i32Q12_LowSpdLower;
    int32_t i32ObsStableTime;
    int32_t i32Q12_ObserableWr;
    int32_t i32Q24_ForceIq;
    int32_t i32Q24_ForceWr;
    /** align parameters ******************************************************/
    int32_t i32Q24_AlignIq;
    int32_t i32Q24_AlignDIq;
    int32_t i32Q24_AlignDTheta;
    int32_t i32Align1stTime;
    int32_t i32Align2ndTime;
    int32_t i32Align3rdTime;
    /** force-drive parameters ************************************************/
    int32_t i32Q24_ForceMaxIq;
    int32_t i32Q24_ForceDIq;
    int32_t i32Q24_ForceMaxWr;
    int32_t i32Q24_ForceAccRate;

    /** short-brake parameters ************************************************/
    int32_t i32MinShortBrakeTime;
    int32_t i32MaxShortBrakeTime;
    int32_t i32Q24_ShortBrakeEndSqrIs;
    /** head-wind force-drive parameters **************************************/
    int32_t i32Q24_HeadWindAccRate;
    int32_t i32Q24_HeadWindForceWr;
    int32_t i32Q12_HeadWindIs;


} stc_startup_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize start-up control function
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Startup_Init(stc_startup_t *pstc);

/**
 ******************************************************************************
 ** \brief start-up control process
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Startup_Control(stc_startup_t *pstc);

#endif /* MOTOR_MIDWARES_STARTUP_CONTROL_H_ */
