/**
 *******************************************************************************
 * @file  hardware_definitions.h
 * @brief define hardware macros for target MCU.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-09-30       CDT             First version
 @endverbatim
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

#ifndef MCU_CONFIG_HARDWARE_DEFINITIONS_H_
#define MCU_CONFIG_HARDWARE_DEFINITIONS_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_common/mcu_include.h"
/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/

//--CAUTION: DO NOT MODIFY START-----------------------------------------------
// define PWM active level for motor control

#define WATCHDOG_REST               (0x00)
#define WATCHDOG_IRQ                (0x01)

/*******************************************************************************
** define PWM active level for motor control
*******************************************************************************/
//--CAUTION: DO NOT MODIFY START-----------------------------------------------
//                                              |   UP arm  |   LOW arm |     |
#define PWM_ACTIVE_LEVEL_HH     0x0         //  |    HIGH   |    HIGH   |     |
#define PWM_ACTIVE_LEVEL_LL     0x1         //  |    LOW    |    LOW    |     |
#define PWM_ACTIVE_LEVEL_LH     0x2         //  |    LOW    |    HIGH   |     |
#define PWM_ACTIVE_LEVEL_HL     0x3         //  |    HIGH   |    LOW    |     |
//--CAUTION: DO NOT MODIFY END-------------------------------------------------

/*******************************************************************************
** define hardware over-current source
*******************************************************************************/
#define HW_OCP_SRC_VC    1
#define HW_OCP_SRC_IO    2


/*******************************************************************************
** define PWM IO operation macros, DO NOT MODIFY
*******************************************************************************/
/** TIM4-1 ********************************************************************/
#define TIM4_1_UH_PA08  0x10
#define TIM4_1_UH_PE09  0x11

#define TIM4_1_UL_PA07  0x12
#define TIM4_1_UL_PB13  0x13
#define TIM4_1_UL_PE08  0x14

#define TIM4_1_VH_PA09  0x15
#define TIM4_1_VH_PE11  0x16

#define TIM4_1_VL_PB00  0x17
#define TIM4_1_VL_PB14  0x18
#define TIM4_1_VL_PE10  0x19

#define TIM4_1_WH_PA10  0x1A
#define TIM4_1_WH_PE13  0x1B

#define TIM4_1_WL_PB01  0x1C
#define TIM4_1_WL_PB15  0x1D
#define TIM4_1_WL_PE12  0x1E

/** TIM4-2 ********************************************************************/
#define TIM4_2_UH_PA00  0x20
#define TIM4_2_UH_PC04  0x21

#define TIM4_2_UL_PA01  0x22
#define TIM4_2_UL_PC05  0x23

#define TIM4_2_VH_PA02  0x24
#define TIM4_2_VH_PB10  0x25

#define TIM4_2_VL_PA03  0x26
#define TIM4_2_VL_PB12  0x27

#define TIM4_2_WH_PA04  0x28
#define TIM4_2_WH_PC08  0x29

#define TIM4_2_WL_PA05  0x2A
#define TIM4_2_WL_PC09  0x2B

/** TIM4-3 ********************************************************************/
#define TIM4_3_UH_PB09  0x40
#define TIM4_3_UH_PC10  0x41

#define TIM4_3_UL_PB08  0x42
#define TIM4_3_UL_PD08  0x43

#define TIM4_3_VH_PB07  0x44
#define TIM4_3_VH_PC11  0x45

#define TIM4_3_VL_PB06  0x46
#define TIM4_3_VL_PD09  0x47

#define TIM4_3_WH_PB05  0x48
#define TIM4_3_WH_PC12  0x49

#define TIM4_3_WL_PA12  0x4A
#define TIM4_3_WL_PD10  0x4B

#endif /* MCU_CONFIG_HARDWARE_DEFINITIONS_H_ */
