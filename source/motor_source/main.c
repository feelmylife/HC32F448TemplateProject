/**
 *******************************************************************************
 * @file  main.c
 * @brief
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

int main(void)
{
  uint32_t u32ClockErr;

  InitMcu_Nvic();
  u32ClockErr = InitMcu_Clock();
  InitMcu_Gpio();
  Motor_HwInit();
  Motor_SwInit();

#if(TRUE == USE_WATCHDOG)
  InitMcu_Wdg();
#endif

  InitMcu_StartSystemTicker(TRUE);
  InitMcu_StartMotorTimer(TRUE);               // enable PWM output
  
#if(TRUE == USE_WATCHDOG)
  /* fed watdog ,start watdog */
  InitMcu_FeedWdg();
#endif
  if(0U != u32ClockErr)
  {
    g_stcMotorRunPara.u32FaultCode |= ERR_SYS_CLK;
  }

  while (TRUE) 
  {

#if(TRUE == USE_WATCHDOG)
    InitMcu_FeedWdg();
#endif
    Speed_CmdSpdSet(&g_stcMotorCmdSpdSet);

  }
   
}



