/**
 *******************************************************************************
 * @file  speed_set.c
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
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_source/motor_control.h"

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
 ** \brief initialize command speed set functions
 **
 ** \param [in] stc_cmd_speed_set_t* pstc: command speed set structure pointer
 **
 ** \retval     none
 ******************************************************************************/
void Speed_InitCmdSpdSet(stc_cmd_speed_set_t *pstc)
{
    float32_t f32Temp;
    f32Temp = (60.0f * ui_f32BaseFrequency) / (float32_t)ui_i32PolePairs;
    pstc->i32Wr2Rpm = (int32_t)f32Temp;

    f32Temp = (float32_t)Q24(ui_i32PolePairs) / (60.0f * ui_f32BaseFrequency);
    pstc->i32Q24_Rpm2Wr = (int32_t)f32Temp;
    pstc->i32MinRpm = ui_i32MinSpeedRpm;
    pstc->i32MaxRpm = ui_i32MaxSpeedRpm;

    pstc->i32CmdRpm = 0;
    pstc->i32RealRpm = 0;
    pstc->i32PreCmdRpm = 0;
}

/**
 ******************************************************************************
 ** \brief initialize target speed trajectory setting function
 **
 ** \param [in] stc_speed_traj_t* pstc: pointer to speed trajectory control
 **
 ** \retval     none
 ******************************************************************************/
void Speed_InitSpdTraj(stc_speed_traj_t *pstc)
{
    pstc->i32Q24_TgtWr = 0;
    pstc->i32Q24_MaxWr = Q24((1.0f * (float32_t)ui_i32PolePairs * (float32_t)ui_i32MaxSpeedRpm) / (60.0f * ui_f32BaseFrequency));
    pstc->i32Q24_MinWr = Q24((1.0f * (float32_t)ui_i32PolePairs * (float32_t)ui_i32MinSpeedRpm) / (60.0f * ui_f32BaseFrequency));
    pstc->i32Q24_WrAccRate = Q24((ui_f32AccHzPerSec * ui_f32SpdRegPeriodMs) / (1000.0f * ui_f32BaseFrequency));
    pstc->i32Q24_WrDecRate = Q24((ui_f32DecHzPerSec * ui_f32SpdRegPeriodMs) / (1000.0f * ui_f32BaseFrequency));
}

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
void Speed_CmdSpdSet(stc_cmd_speed_set_t *pstc)
{
    int32_t i32Rpm;
    int32_t i32AbsRpm;

    /***************************************************************************
    * update rotor speed(Q12(p.u.) and RPM)
    ***************************************************************************/
    i32Rpm = ((g_stcMotorRunPara.i32Q12_RealTimeWr * pstc->i32Wr2Rpm) >> 12);
    if (ROTOR_DIR_CCW == g_stcMotorRunPara.i32TgtWrDir)
    {
        pstc->i32RealRpm = i32Rpm;
    }
    else
    {
        pstc->i32RealRpm = -i32Rpm;
    }

    /***************************************************************************
    * FAULT: clear parameters, and set motor in IDLE status
    ***************************************************************************/
    if ((ERR_NONE != g_stcMotorRunPara.u32FaultCode) || (FALSE == g_stcAdcSample.bAdcRdy))
    {
//        pstc->i32CmdRpm = 0;
//        pstc->i32PreCmdRpm = 0;
//        g_stcMotorRunPara.i32Q12_CmdWr = 0;
//        g_stcMotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
//        return;
    }

    /***************************************************************************
    * NO fault: with new command, check speed validity, reshape command speed:
    *       CmdRpm = {[-i32MaxRpm, -i32MinRpm], [0], [i32MinRpm, i32MaxRpm]}
    ***************************************************************************/
    if (pstc->i32CmdRpm != pstc->i32PreCmdRpm)
    {
        i32Rpm = pstc->i32CmdRpm;
        i32AbsRpm = abs(i32Rpm);

        /** round command speed in legal region *******************************/
        if (i32AbsRpm < pstc->i32MinRpm)
        {
            i32AbsRpm = pstc->i32MinRpm;
        }
        else if (i32AbsRpm > pstc->i32MaxRpm)
        {
            i32AbsRpm = pstc->i32MaxRpm;
        }
        else
        {

        }

        /** set command speed after rounding **********************************/
        if (i32Rpm > 0)
        {
            i32Rpm = i32AbsRpm;
            g_stcMotorRunPara.i32CmdWrDir = ROTOR_DIR_CCW;
        }
        else if (i32Rpm < 0)
        {
            i32Rpm = -i32AbsRpm;
            g_stcMotorRunPara.i32CmdWrDir = ROTOR_DIR_CW;
        }
        else
        {
            i32Rpm = 0;
            i32AbsRpm = 0;
        }
        /** restore processed parameters **************************************/
        pstc->i32CmdRpm = i32Rpm;
        pstc->i32PreCmdRpm = i32Rpm;
        g_stcMotorRunPara.i32Q12_CmdWr = ((i32AbsRpm * pstc->i32Q24_Rpm2Wr) >> 12);
    }

    /***************************************************************************
    * NO fault: scan command speed setting and set motor status
    * 0: ZERO speed command
    *       MOTOR_STUS_DRV_IDLE-->MOTOR_STUS_DRV_IDLE
    *       MOTOR_STUS_DRV_RUN -->MOTOR_STUS_DRV_STOP--[stop control]-->MOTOR_STUS_DRV_IDLE
    *       MOTOR_STUS_DRV_STOP--[stop control]-->MOTOR_STUS_DRV_IDLE
    * 1: NONE_ZERO speed command
    *       MOTOR_STUS_DRV_IDLE-->MOTOR_STUS_DRV_RUN
    *       MOTOR_STUS_DRV_RUN -->MOTOR_STUS_DRV_RUN (same direction)
    *                          --[stop control]-->MOTOR_STUS_DRV_IDLE
    *       MOTOR_STUS_DRV_STOP--[stop control]-->MOTOR_STUS_DRV_IDLE
    ***************************************************************************/
    if (0 == pstc->i32PreCmdRpm)
    {
        if (MOTOR_STUS_DRV_RUN == g_stcMotorRunPara.u32MotorStatus)
        {
            g_stcMotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_STOP;
        }
    }
    else
    {
        if (MOTOR_STUS_IDLE == g_stcMotorRunPara.u32MotorStatus)
        {
            Motor_SwInit();
            g_stcMotorRunPara.i32TgtWrDir = g_stcMotorRunPara.i32CmdWrDir;
            g_stcMotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_RUN;
        }
        else if (MOTOR_STUS_DRV_RUN == g_stcMotorRunPara.u32MotorStatus)
        {
            if (g_stcMotorRunPara.i32CmdWrDir != g_stcMotorRunPara.i32TgtWrDir)
            {
                g_stcMotorRunPara.u32MotorStatus = MOTOR_STUS_DRV_STOP;
            }
        }
        else
        {

        }
    }
}

/**
 ******************************************************************************
 ** \brief: set target speed trajectory according to command speed, present
 **         speed, motor status, and acceleration/deceleration rate.
 ** \param [in] int32_t i32Q12_CmdWr: command speed(positive only) in p.u.
 ** \param [in] stc_speed_traj_t* pstc: pointer to speed trajectory control
 **
 ** \retval     int32_t i32Q12_TgtWr: target speed
 ******************************************************************************/
int32_t Speed_TgtSpdTraj(int32_t i32Q12_CmdWr, stc_speed_traj_t *pstc)
{
    int32_t i32Q24_DeltaWr;
    int32_t i32Q24_CmdWr = (i32Q12_CmdWr << 12);

    i32Q24_DeltaWr = i32Q24_CmdWr - pstc->i32Q24_TgtWr;

    /** case acceleration *****************************************************/
    if (i32Q24_DeltaWr > pstc->i32Q24_WrAccRate)
    {
        pstc->i32Q24_TgtWr += pstc->i32Q24_WrAccRate;
    }

    /** case deceleration *****************************************************/
    else if (i32Q24_DeltaWr < (-pstc->i32Q24_WrDecRate))
    {
        pstc->i32Q24_TgtWr -= pstc->i32Q24_WrDecRate;
    }

    else
    {
        pstc->i32Q24_TgtWr = i32Q24_CmdWr;
    }

    /** target speed range check **********************************************/
    if (pstc->i32Q24_TgtWr < pstc->i32Q24_MinWr)
    {
        pstc->i32Q24_TgtWr = pstc->i32Q24_MinWr;
    }
    else if (pstc->i32Q24_TgtWr > pstc->i32Q24_MaxWr)
    {
        pstc->i32Q24_TgtWr = pstc->i32Q24_MaxWr;
    }
    else
    {

    }

    return (pstc->i32Q24_TgtWr >> 12);
}
