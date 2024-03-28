/**
 *******************************************************************************
 * @file  speed_set.h
 * @brief This file contains speed cruve set function.
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
#ifndef MOTOR_MIDWARES_CTRL_MODE_SPEED_H_
#define MOTOR_MIDWARES_CTRL_MODE_SPEED_H_

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

typedef struct {
    int32_t i32CmdRpm;                  /** (RPM) command speed */
    int32_t i32RealRpm;                 /** (RPM) real speed */
    int32_t i32PreCmdRpm;               /** (RPM) previous command speed */
    /***************************************************************************
     *  legal speed region:
     *    (-i32MaxRpm)<-->(-i32MinRpm)  &  (i32MinRpm)<-->(i32MaxRpm)
     **************************************************************************/
    int32_t i32MinRpm;                  /** (RPM) minimum speed > 0 */
    int32_t i32MaxRpm;                  /** (RPM) maximum speed > 0 */
    int32_t i32Q24_Rpm2Wr;              /** convert RPM to electric p.u. */
    int32_t i32Wr2Rpm;                  /** convert electric p.u. to  RPM */
} stc_cmd_speed_set_t;

typedef struct {
    int32_t     i32Q24_TgtWr;
    int32_t     i32Q24_MinWr;
    int32_t     i32Q24_MaxWr;
    int32_t     i32Q24_WrAccRate;
    int32_t     i32Q24_WrDecRate;
} stc_speed_traj_t;


/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize command speed set functions
 **
 ** \param [in] stc_cmd_speed_set_t* pstc: command speed set structure pointer
 **
 ** \retval     none
 ******************************************************************************/
extern void Speed_InitCmdSpdSet(stc_cmd_speed_set_t *pstc);

/**
 ******************************************************************************
 ** \brief initialize target speed trajectory setting function
 **
 ** \param [in] stc_speed_traj_t* pstc: pointer to speed trajectory control
 **
 ** \retval     none
 ******************************************************************************/
extern void Speed_InitSpdTraj(stc_speed_traj_t *pstc);

/**
 ******************************************************************************
 ** \brief command speed set function:
 **        according to user given speed (i32CmdRpm), rounding command speed in
 **        legal speed region, and set motor operation status.
 **
 ** \param [in] stc_cmd_speed_set_t* pstc: command speed set structure pointer
 **
 ** \retval     none
 ******************************************************************************/
extern void Speed_CmdSpdSet(stc_cmd_speed_set_t *pstc);

/**
 ******************************************************************************
 ** \brief: set target speed trajectory according to command speed, present
 **         speed, motor status, and acceleration/deceleration rate.
 ** \param [in] int32_t i32Q12_CmdWr: command speed(positive only) in p.u.
 ** \param [in] stc_speed_traj_t* pstc: pointer to speed trajectory control
 **
 ** \retval     int32_t i32Q12_TgtWr: target speed
 ******************************************************************************/
extern int32_t Speed_TgtSpdTraj(int32_t i32Q12_CmdWr, stc_speed_traj_t *pstc);

#endif /* MOTOR_MIDWARES_SPEED_SET_H_ */
