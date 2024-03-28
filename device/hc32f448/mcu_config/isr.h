/**
 *******************************************************************************
 * @file  isr.h
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

#ifndef MCU_CONFIG_ISR_H_
#define MCU_CONFIG_ISR_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_common/mcu_include.h"
#include "motor_source/motor_control.h"


/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/
#define WDT_IRQHandler(void)                IRQ000_Handler(void)    ///< software watch dog

#define Motor_HW_OC_IRQHandler(void)        IRQ001_Handler(void)    ///< EMI group0 for Timer6
#define Motor_PWM_IRQHandler(void)          IRQ002_Handler(void)    ///< timer41 CNT interrupt


extern void SysTick_Handler(void);

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
extern uint16_t *const Adc_u16AdcResult;
/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

#endif /* MCU_CONFIG_ISR_H_ */
