/**
 *******************************************************************************
 * @file  hardware_config.c
 * @brief This file provides peripheral function initialization.
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
/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_config/hardware_config.h"
/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/*******************************************************************************
 ** basic drive information
 ******************************************************************************/
#if(TRUE == USE_HSRC)
int32_t i32_CLK_FREQ =  CLK_HSRC_FREQ;              // frequency of HSRC (MHz)
#else
int32_t i32_CLK_FREQ = CLK_XTAL_FREQ;               // frequency of XTAL (MHz)
#endif


uint8_t u8_ADC_CH_VDC              = ADC_CH_VDC;    /** VDC sample channel              */
uint8_t u8_ADC_CH_IU               = ADC_CH_IU;     /** IU  sample channel              */
uint8_t u8_ADC_CH_IV               = ADC_CH_IV;     /** IV  sample channel, case single-shunt, ignore */
uint8_t u8_ADC_CH_VSP              = ADC_CH_VSP;    /** VSP sample channel              */


