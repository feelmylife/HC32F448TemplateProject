/**
 *******************************************************************************
 * @file  cmd_set_analog_io.c
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


/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_source/cmd_set_analog_io.h"

/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/
stc_analog_cmd_t        g_stcMotorAnaSpdSet;

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
 ** \brief initialize command speed set by analog IO
 **
 ** \param [in] *pstc: pointer to analog command speed set structure
 **
 ** \retval     none
 ******************************************************************************/
void AnaSpd_Init(stc_analog_cmd_t *pstc)
{
    pstc->bRdy = FALSE;
    pstc->i32CmdRpm = 0;
    pstc->i32MinCmdRpm = ui_i32MinSpeedRpm;
    pstc->i32MaxCmdRpm = ui_i32MaxSpeedRpm;
    pstc->i32AdcValue = 0;
    pstc->i32AvgAdcValue = 0;
    pstc->i32StartAdcValue = 1000;
    pstc->i32StopAdcValue = 800;
    pstc->i32MaxAdcValue = 3500;
    pstc->i32Cnt = 0;
}

/**
 ******************************************************************************
 ** \brief set command speed according to analog IO input
 **
 ** \param [in] *pstc: pointer to analog command speed set structure
 **
 ** \retval     none
 ******************************************************************************/
void AnaSpd_SetCmdSpd(stc_analog_cmd_t *pstc)
{
    /***************************************************************************
     ** if no fault has been detected in potential meter
     **************************************************************************/
    if (TRUE == pstc->bRdy) 
    {
        /** filtering *********************************************************/
        pstc->i32Cnt++;
        if (pstc->i32Cnt < 16) 
        {
            pstc->i32AvgAdcValue += pstc->i32AdcValue;
            return;
        }

        /** get average ADC value, and set command speed **********************/
        pstc->i32AvgAdcValue = (pstc->i32AvgAdcValue >> 4);
//		/*/////*******/////*/	
// 				pstc->i32AvgAdcValue=1500;
        if (pstc->i32AvgAdcValue <= pstc->i32StopAdcValue) 
        {
            pstc->i32CmdRpm = 0;
        }

        else 
        {
            // round ADC value to maximum
            if (pstc->i32AvgAdcValue >= pstc->i32MaxAdcValue) 
            {
                pstc->i32AvgAdcValue = pstc->i32MaxAdcValue;
            }

            if (pstc->i32AvgAdcValue < pstc->i32StartAdcValue) 
            {
                if (0 == pstc->i32CmdRpm) 
                {
                    return;
                } 
                else 
                {
                    pstc->i32AvgAdcValue = pstc->i32StartAdcValue;
                }
            }

            pstc->i32CmdRpm = pstc->i32MinCmdRpm +
                              (pstc->i32MaxCmdRpm - pstc->i32MinCmdRpm)
                              * (pstc->i32AvgAdcValue - pstc->i32StartAdcValue)
                              / (pstc->i32MaxAdcValue - pstc->i32StartAdcValue);
        }
        pstc->i32AvgAdcValue = 0;       // clear average
        pstc->i32Cnt = 0;               // clear counter
    } 
    else 
    {
        if (pstc->i32AdcValue <= pstc->i32StopAdcValue) 
        {
            if (++pstc->i32Cnt > 20) {
                pstc->i32Cnt = 0;
                pstc->bRdy = TRUE;
            }
        }
       else
       {
        pstc->bRdy = TRUE;
       }
    }
}
