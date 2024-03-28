/**
 *******************************************************************************
 * @file  cmd_set_analog_io.h
 * @brief This file contains use analog IO to set command speed function.
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
#ifndef MOTOR_SOURCE_CMD_SET_ANALOG_IO_H_
#define MOTOR_SOURCE_CMD_SET_ANALOG_IO_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/module_include.h"
#include "motor_source/user_interface.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/
typedef struct {
    boolean_t bRdy;
    int32_t i32CmdRpm;
    int32_t i32MinCmdRpm;
    int32_t i32MaxCmdRpm;
    int32_t i32AdcValue;
    int32_t i32AvgAdcValue;
    int32_t i32StartAdcValue;
    int32_t i32StopAdcValue;
    int32_t i32MaxAdcValue;
    int32_t i32Cnt;
} stc_analog_cmd_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
extern stc_analog_cmd_t        g_stcMotorAnaSpdSet;

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief initialize command speed set by analog IO
 **
 ** \param [in] *pstc: pointer to analog command speed set structure
 **
 ** \retval     none
 ******************************************************************************/
extern void AnaSpd_Init(stc_analog_cmd_t *pstc);

/**
 ******************************************************************************
 ** \brief set command speed according to analog IO input
 **
 ** \param [in] *pstc: pointer to analog command speed set structure
 **
 ** \retval     none
 ******************************************************************************/
extern void AnaSpd_SetCmdSpd(stc_analog_cmd_t *pstc);

#endif /* MOTOR_SOURCE_CMD_SET_ANALOG_IO_H_ */
