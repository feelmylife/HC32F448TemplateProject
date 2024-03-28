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
/* Local type definitions ('typedef')                                         */
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
 ** \brief initialize stop control function
 **
 ** \param [in] stc_stop_ctrl_cfg_t stcCfg: configuration parameters
 ** \param [in] stc_stop_ctrl_t* pstc: pointer to stop-control structure
 **
 ** \retval     none
 ******************************************************************************/
void Stop_Init(stc_stop_ctrl_cfg_t stcCfg, stc_stop_ctrl_t *pstc)
{
    pstc->bEnBrakeStop = stcCfg.bEnBrakeStop;
    pstc->enStatus = Stop_Initialize;

    pstc->i32TimerCnt = 0;

}

/**
 ******************************************************************************
 ** \brief stop motor control process
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Stop_Control(stc_stop_ctrl_t *pstc)
{
    int32_t i32Q12_Temp;
    g_stcMotorRunPara.i32Q24_DrvAngle = g_stcMotorFluxObs.i32Q24_ThetaEst;

    if (Startup_Finish != g_stcMotorStartup.enStatus)
    {
        g_stcMotorRunPara.bEnSpdPid = FALSE;
        g_stcMotorRunPara.bEnSpdTraj = FALSE;
        g_stcMotorRunPara.bEnBrake = FALSE;
        g_stcMotorRunPara.bEnCurrentPid = FALSE;

        Motor_FastStop();
        pstc->enStatus = Stop_Finish;
        return;
    }

    /***************************************************************************
     ** normal stop routines
     **************************************************************************/
    if (Stop_Initialize == pstc->enStatus)
    {
        pstc->i32TimerCnt = 0;
        /***********************************************************************
         ** if use braking function to stop motor is required
         **********************************************************************/
        if (TRUE == pstc->bEnBrakeStop)
        {
            if (g_stcMotorFluxObs.i32Q12_WrF < g_stcMotorStartup.i32Q12_ObserableWr)
            {
                g_stcMotorRunPara.bEnSpdPid = FALSE;
                g_stcMotorRunPara.bEnSpdTraj = FALSE;
                g_stcMotorRunPara.bEnBrake = FALSE;
                g_stcMotorRunPara.bEnCurrentPid = FALSE;
                pstc->enStatus = Stop_Finish;
                Motor_FastStop();
            }
            else
            {

                pstc->enStatus = Stop_ShortBrake;
                pstc->u8ShortBrakeStatus = 0;
            }

        }
        else
        {
            pstc->enStatus = Stop_Finish;
            Motor_FastStop();

        }
    }
    /***************************************************************************
     ** regulate current to zero then turn-off all IGBTs
     **     1. decrease IqRef to zero by slop
     **     2. if IdRef != 0, set lower speed to exit field weakening control
     **************************************************************************/
    else if (Stop_DecCurrent == pstc->enStatus)
    {

    }
    else if (Stop_ElecBrake == pstc->enStatus)
    {

    }
    else if (Stop_ShortBrake == pstc->enStatus)
    {
        g_stcMotorRunPara.i32Q24_DrvAngle = g_stcMotorFluxObs.i32Q24_ThetaEst;
        if (0U == pstc->u8ShortBrakeStatus)
        {

            g_stcMotorRunPara.bEnSpdPid = FALSE;
            g_stcMotorRunPara.bEnSpdTraj = FALSE;
            g_stcMotorRunPara.bEnBrake = FALSE;
            g_stcMotorRunPara.bEnCurrentPid = FALSE;

            g_stcMotorVdqRef.i32Q12_D = 0;
            g_stcMotorVdqRef.i32Q12_Q = 0;
            g_stcMotorVabRefZ.i32Q12_Alpha = 0;
            g_stcMotorVabRefZ.i32Q12_Beta = 0;
            g_stcMotorVabRef.i32Q12_Alpha = 0;
            g_stcMotorVabRef.i32Q12_Beta = 0;
            g_stcMotorPwmGen.u16Uon  = 0xFFFF;
            g_stcMotorPwmGen.u16Uoff = 0xFFFF;
            g_stcMotorPwmGen.u16Von  = 0xFFFF;
            g_stcMotorPwmGen.u16Voff = 0xFFFF;
            g_stcMotorPwmGen.u16Won  = 0xFFFF;
            g_stcMotorPwmGen.u16Woff = 0xFFFF;
            pstc->i32TimerCnt = 0;
            pstc->u8ShortBrakeStatus = 1;
        }
        else if (1U == pstc->u8ShortBrakeStatus)
        {
            if (++pstc->i32TimerCnt > g_stcMotorStartup.i32MinShortBrakeTime)
            {
                pstc->i32TimerCnt = 0;
                pstc->u8ShortBrakeStatus = 2;
            }
        }
        else
        {
            pstc->i32TimerCnt++;
            i32Q12_Temp = g_stcMotorIab.i32Q12_Alpha * g_stcMotorIab.i32Q12_Alpha
                          + g_stcMotorIab.i32Q12_Beta * g_stcMotorIab.i32Q12_Beta; // Q24

            if ((i32Q12_Temp <= g_stcMotorStartup.i32Q24_ShortBrakeEndSqrIs)
                    || (pstc->i32TimerCnt >= g_stcMotorStartup.i32MaxShortBrakeTime))
            {
                g_stcMotorRunPara.bEnCurrentPid = TRUE;
                g_stcMotorIdPid.i32Q27_Output = 0;
                g_stcMotorIqPid.i32Q27_Output = 0;
                g_stcMotorIdPid.i32Q12_Err1 = 0;
                g_stcMotorIqPid.i32Q12_Err1 = 0;
                g_stcMotorIdqRef.i32Q12_Q = 0;
                g_stcMotorIdqRef.i32Q12_D = 0;
                pstc->enStatus = Stop_Finish;
            }
        }
    }
    else if (Stop_Finish == pstc->enStatus)
    {
        g_stcMotorRunPara.bEnSpdPid = FALSE;
        g_stcMotorRunPara.bEnSpdTraj = FALSE;
        g_stcMotorRunPara.bEnBrake = FALSE;
        g_stcMotorRunPara.bEnCurrentPid = FALSE;

        Motor_FastStop();
    }
    else
    {
        g_stcMotorRunPara.bEnSpdPid = FALSE;
        g_stcMotorRunPara.bEnSpdTraj = FALSE;
        g_stcMotorRunPara.bEnBrake = FALSE;
        g_stcMotorRunPara.bEnCurrentPid = FALSE;

        Motor_FastStop();
    }
}
