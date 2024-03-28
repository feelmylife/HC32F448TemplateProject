/**
 *******************************************************************************
 * @file  startup_control.c
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
 ** \brief initialize start-up control function
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Startup_Init(stc_startup_t *pstc)
{
    float32_t f32Temp;
    float32_t f32InvIB = 1.0f / ui_f32BaseCurrent;
    float32_t f32InvFB = 1.0f / ui_f32BaseFrequency;

    pstc->enStatus = Startup_Initialize;
    pstc->u8Status_StateDetect = 0;
    pstc->u8Status_Align = 0;
    pstc->i32TimerCnt = 0;


    pstc->i32Q20_Ts = Q20(ui_f32BaseFrequency / (float32_t)g_stcMotorRunPara.i32Fs);    // only for angle integration

    pstc->i32Q24_MaxStartIq = Q24(0.89f * ui_f32MaxStartIs * f32InvIB);
    pstc->i32Q27_MaxNormaIq = Q27(ui_f32MaxNormaIs * f32InvIB);
    pstc->i32Q12_LowSpdLower = Q12(0.60f * (float32_t)ui_i32PolePairs * (float32_t)ui_i32MinSpeedRpm / (60.0f * ui_f32BaseFrequency));
    pstc->i32Q12_LowSpdUpper = Q12(0.90f * (float32_t)ui_i32PolePairs * (float32_t)ui_i32MinSpeedRpm / (60.0f * ui_f32BaseFrequency));

    /** initialize initial-state-detection function ***************************/
    f32Temp = ui_f32ObsStableTimeMs * (float32_t)g_stcMotorRunPara.i32Fs / 1000.0f;
    pstc->i32ObsStableTime = (int32_t)f32Temp;
    pstc->i32Q12_ObserableWr = Q12(ui_f32MinObserableWr * f32InvFB);

    if (pstc->i32Q12_ObserableWr >= pstc->i32Q12_LowSpdLower)
    {
        pstc->i32Q12_ObserableWr = pstc->i32Q12_LowSpdLower;
    }

    pstc->i32Q24_ForceIq = 0;
    pstc->i32Q24_ForceWr = 0;
    /***************************************************************************
     ** initialize align parameters
     **************************************************************************/
    f32Temp = ui_f32_1stAlignTimeMs * (float32_t)g_stcMotorRunPara.i32Fs / 1000.0f;
    pstc->i32Align1stTime = (int32_t)f32Temp;

    f32Temp = ui_f32_2ndAlignTimeMs * (float32_t)g_stcMotorRunPara.i32Fs / 1000.0f;
    pstc->i32Align2ndTime = (int32_t)f32Temp;

    f32Temp = ui_f32_3rdAlignTimeMs * (float32_t)g_stcMotorRunPara.i32Fs / 1000.0f;
    pstc->i32Align3rdTime = (int32_t)f32Temp;

    pstc->i32Q24_AlignIq = Q24(0.89f * ui_f32AlignCurrent * f32InvIB);
    if (pstc->i32Align1stTime <= 0)
    {
        pstc->i32Align1stTime = 0;
        pstc->i32Q24_AlignDIq = pstc->i32Q24_AlignIq;
    }
    else
    {
        pstc->i32Q24_AlignDIq = pstc->i32Q24_AlignIq / pstc->i32Align1stTime;
    }

    if (pstc->i32Align2ndTime <= 0)
    {
        pstc->i32Align2ndTime = 0;
        pstc->i32Q24_AlignDTheta = 0;
    }
    else
    {
        pstc->i32Q24_AlignDTheta = DEG_TO_Q24(ui_f32_2ndAlignFwdTheta) / pstc->i32Align2ndTime;
    }

    /***************************************************************************
     ** initialize force-drive parameters
     **************************************************************************/
    pstc->i32Q24_ForceMaxIq = pstc->i32Q24_MaxStartIq;

    f32Temp = 0.89f * ui_f32ForceCurrentSlop * f32InvIB / (float32_t)g_stcMotorRunPara.i32Fs;
    pstc->i32Q24_ForceDIq = Q24(f32Temp);
    pstc->i32Q24_ForceAccRate = Q24(ui_f32ForceAccRate * f32InvFB) / g_stcMotorRunPara.i32Fs;
    pstc->i32Q24_ForceMaxWr =  Q24(ui_f32MaxForceSpd * f32InvFB);

    /***************************************************************************
     ** initialize head-wind startup parameters
     **************************************************************************/
    f32Temp = ui_f32MinShortBrakeTimeMs * (float32_t)g_stcMotorRunPara.i32Fs / 1000.0f;
    pstc->i32MinShortBrakeTime = (int32_t)f32Temp;

    f32Temp = ui_f32MaxShortBrakeTimeMs * (float32_t)g_stcMotorRunPara.i32Fs / 1000.0f;
    pstc->i32MaxShortBrakeTime = (int32_t)f32Temp;

    pstc->i32Q24_ShortBrakeEndSqrIs = Q12(ui_f32ShortBrakeEndIs * f32InvIB) * Q12(ui_f32ShortBrakeEndIs * f32InvIB);

    pstc->i32Q12_HeadWindIs = Q12(ui_f32HeadWindStartCurrent * f32InvIB);
    pstc->i32Q24_HeadWindAccRate = Q24(ui_f32HeadWindForceAccRate * f32InvFB) / g_stcMotorRunPara.i32Fs;
    pstc->i32Q24_HeadWindForceWr =  Q24(ui_f32HeadWindMaxForceSpd * f32InvFB);


}

/**
 ******************************************************************************
 ** \brief start-up control process
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Startup_Control(stc_startup_t *pstc)
{
    int32_t i32Q12_Temp;
    int32_t i32Q24_dTheta;

    switch (pstc->enStatus)
    {
#define STARTUP_FINISH
        case Startup_Finish:
        {
            g_stcMotorRunPara.i32Q24_DrvAngle = g_stcMotorFluxObs.i32Q24_ThetaEst;
            break;
        }

#define STARTUP_CLOSE_LOOP_START
        case Startup_CLStart:
        {
            g_stcMotorTimer.i32SpdRegCnt = 0;     // reset speed regulation counter
            g_stcMotorRunPara.bEnSpdPid = TRUE;
            g_stcMotorRunPara.bEnSpdTraj = TRUE;

            g_stcMotorRunPara.i32Q24_DrvAngle = g_stcMotorFluxObs.i32Q24_ThetaEst;

            g_stcMotorWrPid.i32Q27_Output = Q15(g_stcMotorIdqRef.i32Q12_Q) >> 1;
            g_stcMotorWrPid.i32Q27_Iout = g_stcMotorWrPid.i32Q27_Output;
            g_stcMotorSpdTraj.i32Q24_TgtWr = ((g_stcMotorSpdTraj.i32Q24_MinWr) >> 1);

            pstc->enStatus = Startup_Finish;

            break;
        }

#define STARTUP_INITIALIZE
        case Startup_Initialize:
        {
            /** disable all control target ************************************/
            g_stcMotorRunPara.bEnSpdPid      = FALSE;
            g_stcMotorRunPara.bEnSpdTraj     = FALSE;
            g_stcMotorRunPara.bEnBrake       = FALSE;
            g_stcMotorRunPara.bEnCurrentPid  = FALSE;
            pstc->i32TimerCnt = 0;
            pstc->enStatus = Startup_BscCharge;
            break;
        }

#define STARTUP_BSC_CHARGE
        case Startup_BscCharge:
        {
            InitMcu_EnMotorPwm(TRUE);
            pstc->enStatus = Startup_StateDetect;
            break;
        }

#define STARTUP_STATE_DETECT
        /***********************************************************************
         ** detecting motor state: stand-still, tail-wind, or head-wind
         ** previous state: Startup_BscCharge
         **********************************************************************/
        case Startup_StateDetect:
        {
            g_stcMotorRunPara.i32Q24_DrvAngle = g_stcMotorFluxObs.i32Q24_ThetaEst;

            if (0U == pstc->u8Status_StateDetect)
            {
                g_stcMotorIdqRef.i32Q12_D = 0;
                g_stcMotorIdqRef.i32Q12_Q = 0;
                g_stcMotorRunPara.bEnCurrentPid = TRUE;
                pstc->i32TimerCnt = 0;
                pstc->u8Status_StateDetect = 1;
            }
            else if (1U == pstc->u8Status_StateDetect)
            {
                pstc->i32TimerCnt++;

                if (pstc->i32TimerCnt > pstc->i32ObsStableTime)
                {
                    pstc->i32TimerCnt = 0;
                    pstc->u8Status_StateDetect = 2;
                }
            }
            else if (2U == pstc->u8Status_StateDetect)
            {
                pstc->u8Status_StateDetect = 3;
            }

            else if (3U == pstc->u8Status_StateDetect)
            {
                /***************************************************************
                 ** 1. tail-wind, direct apply close-loop start-up
                 **************************************************************/
                if (g_stcMotorFluxObs.i32Q12_WrF >= pstc->i32Q12_ObserableWr)
                {
                    g_stcMotorTimer.i32SpdRegCnt = 0;     // reset speed regulation counter
                    g_stcMotorRunPara.bEnSpdPid = TRUE;
                    g_stcMotorRunPara.bEnSpdTraj = TRUE;

                    if (g_stcMotorFluxObs.i32Q12_WrF < pstc->i32Q12_LowSpdUpper)
                    {
                        g_stcMotorWrPid.i32Q27_Output = pstc->i32Q27_MaxNormaIq;
                        g_stcMotorWrPid.i32Q27_Iout = g_stcMotorWrPid.i32Q27_Output;
                        g_stcMotorSpdTraj.i32Q24_TgtWr = g_stcMotorSpdTraj.i32Q24_MinWr;
                    }
                    else
                    {
                        g_stcMotorWrPid.i32Q27_Output = 0;
                        g_stcMotorWrPid.i32Q27_Iout = 0;
                        g_stcMotorRunPara.i32Q12_TgtWr = g_stcMotorFluxObs.i32Q12_WrF;
                        g_stcMotorSpdTraj.i32Q24_TgtWr = Q12(g_stcMotorRunPara.i32Q12_TgtWr);
                    }

                    pstc->enStatus = Startup_Finish;

                }
                /***************************************************************
                 ** 2. head-wind, apply braking function
                 **************************************************************/
                else if (g_stcMotorFluxObs.i32Q12_WrF < (-pstc->i32Q12_ObserableWr))
                {
                    pstc->enStatus = Startup_HeadWindShortBrake;                /* short brake */

                }
                /***************************************************************
                 ** 3. stand-still, apply short-brake
                 **************************************************************/
                else
                {
                    pstc->enStatus = Startup_Align;
                    g_stcMotorRunPara.i32Q24_DrvAngle = 0;
                }
            }

            else
            {
                pstc->i32TimerCnt = 0;
            }

            break;
        }

#define STARTUP_IPD
        case Startup_IPD:
        {
            break;
        }

#define STARTUP_ALIGN_ROTOR
        case Startup_Align:
        {
            /*******************************************************************
             ** set align current by current slop
             ******************************************************************/
            if (pstc->i32Q24_ForceIq < pstc->i32Q24_AlignIq)
            {
                pstc->i32Q24_ForceIq += pstc->i32Q24_AlignDIq;
            }
            else
            {
                pstc->i32Q24_ForceIq = pstc->i32Q24_AlignIq;
            }

            /*******************************************************************
             ** first align, increasing current to maximum, none position change
             ******************************************************************/
            if (0U == pstc->u8Status_Align)
            {
                if ((--pstc->i32Align1stTime) <= 0)
                {
                    pstc->u8Status_Align = 1U;
                }
            }
            /*******************************************************************
             ** second align, maximum align current, rotates to forward angle
             ******************************************************************/
            else if (1U == pstc->u8Status_Align)
            {
                g_stcMotorRunPara.i32Q24_DrvAngle += pstc->i32Q24_AlignDTheta;
                g_stcMotorRunPara.i32Q24_DrvAngle &= 0xFFFFFF;
                if ((--pstc->i32Align2ndTime) <= 0)
                {
                    pstc->u8Status_Align = 2U;
                }
            }

            /*******************************************************************
             ** third align, maintain constant current and position
             ******************************************************************/
            else if (2U == pstc->u8Status_Align)
            {
                if ((--pstc->i32Align3rdTime) <= 0)
                {
                    pstc->enStatus = Startup_ForceDrive;
                }
            }
            else
            {

            }
            g_stcMotorIdqRef.i32Q12_Q = (pstc->i32Q24_ForceIq >> 12);
            g_stcMotorIdqRef.i32Q12_D = 0;
            break;
        }
#define STARTUP_FORCE_DRIVE
        case Startup_ForceDrive:
        {
            /*******************************************************************
             ** set force-current as maximum
             ******************************************************************/
            if (pstc->i32Q24_ForceIq < pstc->i32Q24_ForceMaxIq)
            {
                pstc->i32Q24_ForceIq += pstc->i32Q24_ForceDIq;
            }
            else
            {
                pstc->i32Q24_ForceIq = pstc->i32Q24_ForceMaxIq;
            }

            g_stcMotorIdqRef.i32Q12_Q = (pstc->i32Q24_ForceIq >> 12);
            g_stcMotorIdqRef.i32Q12_D = 0;

            /*******************************************************************
             ** set force speed according to acceleration rate and position error
             ******************************************************************/
            pstc->i32Q24_ForceWr += pstc->i32Q24_ForceAccRate;
            if (pstc->i32Q24_ForceWr > pstc->i32Q24_ForceMaxWr)
            {
                pstc->enStatus = Startup_CLStart;
            }

            i32Q24_dTheta = (((pstc->i32Q24_ForceWr >> 12) * pstc->i32Q20_Ts) >> 8);
            g_stcMotorRunPara.i32Q24_DrvAngle += i32Q24_dTheta;
            g_stcMotorRunPara.i32Q24_DrvAngle &= 0xFFFFFF;
            break;
        }

#define STARTUP_HEAD_WIND_FOC_BRAKE
        case Startup_HeadWindFocBrake:
        {
            break;
        }

#define STARTUP_HEAD_WIND_SHORT_BRAKE
        case Startup_HeadWindShortBrake:
        {
            g_stcMotorRunPara.i32Q24_DrvAngle = g_stcMotorFluxObs.i32Q24_ThetaEst;
            /*******************************************************************
             ** set PWM into short-brake mode
             ******************************************************************/
            if (0U == pstc->u8Status_HeadWindShortBrake)
            {
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
                g_stcMotorRunPara.bEnCurrentPid = FALSE;
                pstc->i32TimerCnt = 0;
                pstc->u8Status_HeadWindShortBrake = 1;
            }
            /*******************************************************************
             ** apply minimum brake time
             ******************************************************************/
            else if (1U == pstc->u8Status_HeadWindShortBrake)
            {
                if (++pstc->i32TimerCnt > pstc->i32MinShortBrakeTime)
                {
                    pstc->i32TimerCnt = 0;
                    pstc->u8Status_HeadWindShortBrake = 2;
                }
            }
            /*******************************************************************
             ** check short brake condition: current detection and time overflow
             ******************************************************************/
            else if (2U == pstc->u8Status_HeadWindShortBrake)
            {
                pstc->i32TimerCnt++;
                i32Q12_Temp = g_stcMotorIab.i32Q12_Alpha * g_stcMotorIab.i32Q12_Alpha
                              + g_stcMotorIab.i32Q12_Beta * g_stcMotorIab.i32Q12_Beta; // Q24

                if ((i32Q12_Temp <= pstc->i32Q24_ShortBrakeEndSqrIs)
                        || (pstc->i32TimerCnt >= pstc->i32MaxShortBrakeTime))
                {
                    g_stcMotorRunPara.bEnCurrentPid = TRUE;
                    g_stcMotorIdPid.i32Q27_Output = 0;
                    g_stcMotorIqPid.i32Q27_Output = 0;
                    g_stcMotorIdPid.i32Q12_Err1 = 0;
                    g_stcMotorIqPid.i32Q12_Err1 = 0;
                    g_stcMotorIdqRef.i32Q12_Q = pstc->i32Q12_HeadWindIs;
                    g_stcMotorIdqRef.i32Q12_D = 0;
                    pstc->i32Q24_ForceWr = Q12(g_stcMotorFluxObs.i32Q12_WrF);
                    pstc->enStatus = Startup_HeadWindForceDrive;
                }
            }
            else
            {
            }
            break;
        }

#define STARTUP_HEADWIND_START
        case Startup_HeadWindForceDrive:
        {
            pstc->i32Q24_ForceWr += pstc->i32Q24_HeadWindAccRate;
            i32Q24_dTheta = (((pstc->i32Q24_ForceWr >> 12) * pstc->i32Q20_Ts) >> 8);
            g_stcMotorRunPara.i32Q24_DrvAngle += i32Q24_dTheta;
            g_stcMotorRunPara.i32Q24_DrvAngle &= 0xFFFFFF;
            if (pstc->i32Q24_ForceWr > pstc->i32Q24_HeadWindForceWr)
            {
                pstc->i32Q24_ForceIq = Q12(pstc->i32Q12_HeadWindIs);
                pstc->enStatus = Startup_CLStart;
            }

            break;
        }

#define STARTUP_END
        default:
        {
            break;
        }
    }
}



