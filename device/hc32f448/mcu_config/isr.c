/**
 *******************************************************************************
 * @file  isr.c
 * @brief This file contains mcu interrupt service function.
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
#include "mcu_common/mcu_include.h"

/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/


__root void WDT_IRQHandler(void)
{
    if (0x01U == bCM_WDT->SR_b.UDF)
    {
        MODIFY_REG32(CM_WDT->SR, WDT_SR_UDF, 0x0U << WDT_SR_UDF_POS);
    }

    if (0x01U == bCM_WDT->SR_b.REF)
    {
        MODIFY_REG32(CM_WDT->SR, WDT_SR_REF, 0x0U << WDT_SR_REF_POS);
    }

    InitMcu_FeedWdg();
    Motor_FastStop();
    g_stcMotorRunPara.u32FaultCode |= ERR_HWWD_INT;

}


__root void SysTick_Handler(void)
{


    if (TRUE == g_stcAdcSample.bAdcRdy)
    {
        Motor_SysTickerIsrHandler();
    }



}

__root void Motor_PWM_IRQHandler(void)
{

    
#if(TRUE == EN_TEST_IO)
    SET_TEST_GPIO_OUT(1);
#endif
    
    // zero interrupt triggered by timer-4
    if (MTR1_bTIM4x->CCSR_b.IRQZF == 1U)
    {
        MTR1_bTIM4x->CCSR_b.IRQZF = 0U;
        if (FALSE == g_stcAdcSample.bAdcRdy)
        {
            Adc_CheckOffset(&g_stcAdcSample);
        }
        else
        {
            Motor_MainIsrHandler();
        }
    }

    else
    {
        Motor_FastStop();
        g_stcMotorRunPara.u32FaultCode |= ERR_UNDEF_INT;
    }

#if(TRUE == EN_TEST_IO)
    SET_TEST_GPIO_OUT(0);
#endif
}

/**
 ******************************************************************************
 ** \brief EMI interrupt for hardware over current protection
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
__root void Motor_HW_OC_IRQHandler(void)
{
    /** EMB event *************************************************************/
    if (0U != MTR1_EMBx->STAT)
    {
        g_stcMotorRunPara.u32FaultCode |= ERR_OC_HW;

        Motor_FastStop();
        MTR1_EMBx->STATCLR = 0x0FU;           // clear interrupt
    }
}
