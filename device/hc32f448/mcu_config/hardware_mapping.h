/**
 *******************************************************************************
 * @file  hardware_mapping.h
 * @brief mapping ADC, TIM4 according to "hardware_config.h".
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
#ifndef MCU_CONFIG_HARDWARE_MAPPING_H_
#define MCU_CONFIG_HARDWARE_MAPPING_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_common/mcu_include.h"

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/

/*******************************************************************************
** MOTOR-1 PWM generation
*******************************************************************************/

/*******************************************************************************
** WARNING: DO NOT MODIFY
*******************************************************************************/
/*******************************************************************************
** IO macros for MOTOR-1
*******************************************************************************/
#if(0x10 == (0xF0&MTR1_PWM_UH&MTR1_PWM_UL&MTR1_PWM_VH&MTR1_PWM_VL&MTR1_PWM_WH&MTR1_PWM_WL))
    #define MTR1_TIM4_UNIT  (1)
    #define MTR1_TIM4x      CM_TMR4_1
    #define MTR1_bTIM4x     bCM_TMR4_1
    #define MTR1_EMBx       CM_EMB1
    #define MTR1_bEMBx      bCM_EMB1

    #define TIM41_PWM_UH    MTR1_PWM_UH
    #define TIM41_PWM_UL    MTR1_PWM_UL
    #define TIM41_PWM_VH    MTR1_PWM_VH
    #define TIM41_PWM_VL    MTR1_PWM_VL
    #define TIM41_PWM_WH    MTR1_PWM_WH
    #define TIM41_PWM_WL    MTR1_PWM_WL
#elif(0x20 == (0xF0&MTR1_PWM_UH&MTR1_PWM_UL&MTR1_PWM_VH&MTR1_PWM_VL&MTR1_PWM_WH&MTR1_PWM_WL))
    #define MTR1_TIM4_UNIT  (2)
    #define MTR1_TIM4x      CM_TMR4_2
    #define MTR1_bTIM4x     bCM_TMR4_2
    #define MTR1_EMBx       CM_EMB2
    #define MTR1_bEMBx      bCM_EMB2

    #define TIM42_PWM_UH    MTR1_PWM_UH
    #define TIM42_PWM_UL    MTR1_PWM_UL
    #define TIM42_PWM_VH    MTR1_PWM_VH
    #define TIM42_PWM_VL    MTR1_PWM_VL
    #define TIM42_PWM_WH    MTR1_PWM_WH
    #define TIM42_PWM_WL    MTR1_PWM_WL
#elif(0x40 == (0xF0&MTR1_PWM_UH&MTR1_PWM_UL&MTR1_PWM_VH&MTR1_PWM_VL&MTR1_PWM_WH&MTR1_PWM_WL))
    #define MTR1_TIM4_UNIT  (3)
    #define MTR1_TIM4x      CM_TMR4_3
    #define MTR1_bTIM4x     bCM_TMR4_3
    #define MTR1_EMBx       CM_EMB3
    #define MTR1_bEMBx      bCM_EMB3


    #define TIM43_PWM_UH    MTR1_PWM_UH
    #define TIM43_PWM_UL    MTR1_PWM_UL
    #define TIM43_PWM_VH    MTR1_PWM_VH
    #define TIM43_PWM_VL    MTR1_PWM_VL
    #define TIM43_PWM_WH    MTR1_PWM_WH
    #define TIM43_PWM_WL    MTR1_PWM_WL
#endif

    #define MTR1_ADCx           CM_ADC1
    #define MTR1_ADC_STATE      (MTR1_ADCx->ISR & 0x01)
    #define MTR1_ADC_EOC_CLR    do{MTR1_ADCx->ISCLRR &= ~ADC_ISCLRR_CLREOCAF;}while(0);
    #define MTR1_ADC_DR0_ADDR   (0x40040050L)

/*******************************************************************************
** IO macros for MOTOR-2
*******************************************************************************/
#if(0x10 == (0xF0&MTR2_PWM_UH&MTR2_PWM_UL&MTR2_PWM_VH&MTR2_PWM_VL&MTR2_PWM_WH&MTR2_PWM_WL))
    #define MTR2_TIM4_UNIT  (1)
    #define MTR2_TIM4x      CM_TMR4_1
    #define MTR2_bTIM4x     bCM_TMR4_1
    #define MTR2_EMBx       CM_EMB1
    #define MTR2_bEMBx      bCM_EMB1

    #define TIM41_PWM_UH    MTR2_PWM_UH
    #define TIM41_PWM_UL    MTR2_PWM_UL
    #define TIM41_PWM_VH    MTR2_PWM_VH
    #define TIM41_PWM_VL    MTR2_PWM_VL
    #define TIM41_PWM_WH    MTR2_PWM_WH
    #define TIM41_PWM_WL    MTR2_PWM_WL
#elif(0x20 == (0xF0&MTR2_PWM_UH&MTR2_PWM_UL&MTR2_PWM_VH&MTR2_PWM_VL&MTR2_PWM_WH&MTR2_PWM_WL))
    #define MTR2_TIM4_UNIT  (2)
    #define MTR2_TIM4x      CM_TMR4_2
    #define MTR2_bTIM4x     bCM_TMR4_2
    #define MTR2_EMBx       CM_EMB2
    #define MTR2_bEMBx      bCM_EMB2

    #define TIM42_PWM_UH    MTR2_PWM_UH
    #define TIM42_PWM_UL    MTR2_PWM_UL
    #define TIM42_PWM_VH    MTR2_PWM_VH
    #define TIM42_PWM_VL    MTR2_PWM_VL
    #define TIM42_PWM_WH    MTR2_PWM_WH
    #define TIM42_PWM_WL    MTR2_PWM_WL
#elif(0x40 == (0xF0&MTR2_PWM_UH&MTR2_PWM_UL&MTR2_PWM_VH&MTR2_PWM_VL&MTR2_PWM_WH&MTR2_PWM_WL))
    #define MTR2_TIM4_UNIT  (3)
    #define MTR2_TIM4x      CM_TMR4_3
    #define MTR2_bTIM4x     bCM_TMR4_3
    #define MTR2_EMBx       CM_EMB3
    #define MTR2_EMBx       bCM_EMB3

    #define TIM43_PWM_UH    MTR2_PWM_UH
    #define TIM43_PWM_UL    MTR2_PWM_UL
    #define TIM43_PWM_VH    MTR2_PWM_VH
    #define TIM43_PWM_VL    MTR2_PWM_VL
    #define TIM43_PWM_WH    MTR2_PWM_WH
    #define TIM43_PWM_WL    MTR2_PWM_WL
#endif

/*******************************************************************************
** macros to initialize IO port output as TIMER-4, unlock M4_PORT->PWPR before using
*******************************************************************************/
/*******************************************************************************
** TIM4-1-U
*******************************************************************************/
#if(TIM4_1_UH_PA08 == TIM41_PWM_UH)
    #define INIT_TIM41_UH(X)   do{CM_GPIO->PFSRA8 = 0;CM_GPIO->PCRA8 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_UH_PWM   do{CM_GPIO->PFSRA8 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_UH_GPO   do{CM_GPIO->PFSRA8 = 0;}while(0);    /** work as GPO */
#elif(TIM4_1_UH_PE09 == TIM41_PWM_UH)
    #define INIT_TIM41_UH(X)   do{CM_GPIO->PFSRE9 = 0;CM_GPIO->PCRE9 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_UH_PWM   do{CM_GPIO->PFSRE9 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_UH_GPO   do{CM_GPIO->PFSRE9 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_1_UL_PA07 == TIM41_PWM_UL)
    #define INIT_TIM41_UL(X)   do{CM_GPIO->PFSRA7 = 0;CM_GPIO->PCRA7 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_UL_PWM   do{CM_GPIO->PFSRA7 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_UL_GPO   do{CM_GPIO->PFSRA7 = 0;}while(0);    /** work as GPO */
#elif(TIM4_1_UL_PB13 == TIM41_PWM_UL)
    #define INIT_TIM41_UL(X)   do{CM_GPIO->PFSRB13 = 0;CM_GPIO->PCRB13 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_UL_PWM   do{CM_GPIO->PFSRB13 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_UL_GPO   do{CM_GPIO->PFSRB13 = 0;}while(0);    /** work as PWM */
#elif(TIM4_1_UL_PE08 == TIM41_PWM_UL)
    #define INIT_TIM41_UL(X)   do{CM_GPIO->PFSRE8 = 0;CM_GPIO->PCRE8 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_UL_PWM   do{CM_GPIO->PFSRE8 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_UL_GPO   do{CM_GPIO->PFSRE8 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-1-V
*******************************************************************************/
#if(TIM4_1_VH_PA09 == TIM41_PWM_VH)
    #define INIT_TIM41_VH(X)   do{CM_GPIO->PFSRA9 = 0;CM_GPIO->PCRA9 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_VH_PWM   do{CM_GPIO->PFSRA9 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_VH_GPO   do{CM_GPIO->PFSRA9 = 0;}while(0);    /** work as GPO */
#elif(TIM4_1_VH_PE11 == TIM41_PWM_VH)
    #define INIT_TIM41_VH(X)   do{CM_GPIO->PFSRE11 = 0;CM_GPIO->PCRE11 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_VH_PWM   do{CM_GPIO->PFSRE11 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_VH_GPO   do{CM_GPIO->PFSRE11 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_1_VL_PB00 == TIM41_PWM_VL)
    #define INIT_TIM41_VL(X)   do{CM_GPIO->PFSRB0 = 0;CM_GPIO->PCRB0 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_VL_PWM   do{CM_GPIO->PFSRB0 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_VL_GPO   do{CM_GPIO->PFSRB0 = 0;}while(0);    /** work as GPO */
#elif(TIM4_1_VL_PB14 == TIM41_PWM_VL)
    #define INIT_TIM41_VL(X)   do{CM_GPIO->PFSRB14 = 0;CM_GPIO->PCRB14 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_VL_PWM   do{CM_GPIO->PFSRB14 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_VL_GPO   do{CM_GPIO->PFSRB14 = 0;}while(0);    /** work as PWM */
#elif(TIM4_1_VL_PE10 == TIM41_PWM_VL)
    #define INIT_TIM41_VL(X)   do{CM_GPIO->PFSRE10 = 0;CM_GPIO->PCRE10 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_VL_PWM   do{CM_GPIO->PFSRE10 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_VL_GPO   do{CM_GPIO->PFSRE10 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-1-W
*******************************************************************************/
#if(TIM4_1_WH_PA10 == TIM41_PWM_WH)
    #define INIT_TIM41_WH(X)   do{CM_GPIO->PFSRA10 = 0;CM_GPIO->PCRA10 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_WH_PWM   do{CM_GPIO->PFSRA10 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_WH_GPO   do{CM_GPIO->PFSRA10 = 0;}while(0);    /** work as GPO */
#elif(TIM4_1_WH_PE13 == TIM41_PWM_WH)
    #define INIT_TIM41_WH(X)   do{CM_GPIO->PFSRE13 = 0;CM_GPIO->PCRE13 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_WH_PWM   do{CM_GPIO->PFSRE13 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_WH_GPO   do{CM_GPIO->PFSRE13 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_1_WL_PB01 == TIM41_PWM_WL)
    #define INIT_TIM41_WL(X)   do{CM_GPIO->PFSRB1 = 0;CM_GPIO->PCRB1 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM41_WL_PWM   do{CM_GPIO->PFSRB1 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_WL_GPO   do{CM_GPIO->PFSRB1 = 0;}while(0);    /** work as GPO */
#elif(TIM4_1_WL_PB15 == TIM41_PWM_WL)
    #define INIT_TIM41_WL(X)   do{CM_GPIO->PFSRB15 = 0;CM_GPIO->PCRB15 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_WL_PWM   do{CM_GPIO->PFSRB15 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_WL_GPO   do{CM_GPIO->PFSRB15 = 0;}while(0);    /** work as PWM */
#elif(TIM4_1_WL_PE12 == TIM41_PWM_WL)
    #define INIT_TIM41_WL(X)   do{CM_GPIO->PFSRE12 = 0;CM_GPIO->PCRE12 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM41_WL_PWM   do{CM_GPIO->PFSRE12 = 2;}while(0);    /** work as PWM */
    #define SET_TIM41_WL_GPO   do{CM_GPIO->PFSRE12 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-2-U
*******************************************************************************/
#if(TIM4_2_UH_PA00 == TIM42_PWM_UH)
    #define INIT_TIM42_UH(X)   do{CM_GPIO->PFSRA0 = 0;CM_GPIO->PCRA0 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_UH_PWM   do{CM_GPIO->PFSRA0 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_UH_GPO   do{CM_GPIO->PFSRA0 = 0;}while(0);    /** work as GPO */
#elif(TIM4_2_UH_PC04 == TIM42_PWM_UH)
    #define INIT_TIM42_UH(X)   do{CM_GPIO->PFSRC4 = 0;CM_GPIO->PCRC4 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_UH_PWM   do{CM_GPIO->PFSRC4 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_UH_GPO   do{CM_GPIO->PFSRC4 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_2_UL_PA01 == TIM42_PWM_UL)
    #define INIT_TIM42_UL(X)   do{CM_GPIO->PFSRA1 = 0;CM_GPIO->PCRA1 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_UL_PWM   do{CM_GPIO->PFSRA1 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_UL_GPO   do{CM_GPIO->PFSRA1 = 0;}while(0);    /** work as GPO */
#elif(TIM4_2_UL_PC05 == TIM42_PWM_UL)
    #define INIT_TIM42_UL(X)   do{CM_GPIO->PFSRC5 = 0;CM_GPIO->PCRC5 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_UL_PWM   do{CM_GPIO->PFSRC5 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_UL_GPO   do{CM_GPIO->PFSRC5 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-2-V
*******************************************************************************/
#if(TIM4_2_VH_PA02 == TIM42_PWM_VH)
    #define INIT_TIM42_VH(X)   do{CM_GPIO->PFSRA2 = 0;CM_GPIO->PCRA2 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_VH_PWM   do{CM_GPIO->PFSRA2 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_VH_GPO   do{CM_GPIO->PFSRA2 = 0;}while(0);    /** work as GPO */
#elif(TIM4_2_VH_PB10 == TIM42_PWM_VH)
    #define INIT_TIM42_VH(X)   do{CM_GPIO->PFSRB10 = 0;CM_GPIO->PCRB10 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM42_VH_PWM   do{CM_GPIO->PFSRB10 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_VH_GPO   do{CM_GPIO->PFSRB10 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_2_VL_PA03 == TIM42_PWM_VL)
    #define INIT_TIM42_VL(X)   do{CM_GPIO->PFSRA3 = 0;CM_GPIO->PCRA3 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_VL_PWM   do{CM_GPIO->PFSRA3 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_VL_GPO   do{CM_GPIO->PFSRA3 = 0;}while(0);    /** work as GPO */
#elif(TIM4_2_VL_PB12 == TIM42_PWM_VL)
    #define INIT_TIM42_VL(X)   do{CM_GPIO->PFSRB12 = 0;CM_GPIO->PCRB12 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM42_VL_PWM   do{CM_GPIO->PFSRB12 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_VL_GPO   do{CM_GPIO->PFSRB12 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-2-W
*******************************************************************************/
#if(TIM4_2_WH_PA04 == TIM42_PWM_WH)
    #define INIT_TIM42_WH(X)   do{CM_GPIO->PFSRA4 = 0;CM_GPIO->PCRA4 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_WH_PWM   do{CM_GPIO->PFSRA4 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_WH_GPO   do{CM_GPIO->PFSRA4 = 0;}while(0);    /** work as GPO */
#elif(TIM4_2_WH_PC08 == TIM42_PWM_WH)
    #define INIT_TIM42_WH(X)   do{CM_GPIO->PFSRC8 = 0;CM_GPIO->PCRC8 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_WH_PWM   do{CM_GPIO->PFSRC8 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_WH_GPO   do{CM_GPIO->PFSRC8 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_2_WL_PA05 == TIM42_PWM_WL)
    #define INIT_TIM42_WL(X)   do{CM_GPIO->PFSRA5 = 0;CM_GPIO->PCRA5 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_WL_PWM   do{CM_GPIO->PFSRA5 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_WL_GPO   do{CM_GPIO->PFSRA5 = 0;}while(0);    /** work as GPO */
#elif(TIM4_2_WL_PC09 == TIM42_PWM_WL)
    #define INIT_TIM42_WL(X)   do{CM_GPIO->PFSRC9 = 0;CM_GPIO->PCRC9 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM42_WL_PWM   do{CM_GPIO->PFSRC9 = 2;}while(0);    /** work as PWM */
    #define SET_TIM42_WL_GPO   do{CM_GPIO->PFSRC9 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-3-U
*******************************************************************************/
#if(TIM4_3_UH_PB09 == TIM43_PWM_UH)
    #define INIT_TIM43_UH(X)   do{CM_GPIO->PFSRB9 = 0;CM_GPIO->PCRB9 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_UH_PWM   do{CM_GPIO->PFSRB9 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_UH_GPO   do{CM_GPIO->PFSRB9 = 0;}while(0);    /** work as GPO */
#elif(TIM4_3_UH_PC10 == TIM43_PWM_UH)
    #define INIT_TIM43_UH(X)   do{CM_GPIO->PFSRC10 = 0;CM_GPIO->PCRC10 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM43_UH_PWM   do{CM_GPIO->PFSRC10 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_UH_GPO   do{CM_GPIO->PFSRC10 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_3_UL_PB08 == TIM43_PWM_UL)
    #define INIT_TIM43_UL(X)   do{CM_GPIO->PFSRB8 = 0;CM_GPIO->PCRB8 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_UL_PWM   do{CM_GPIO->PFSRB8 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_UL_GPO   do{CM_GPIO->PFSRB8 = 0;}while(0);    /** work as GPO */
#elif(TIM4_3_UL_PD08 == TIM43_PWM_UL)
    #define INIT_TIM43_UL(X)   do{CM_GPIO->PFSRD8 = 0;CM_GPIO->PCRD8 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_UL_PWM   do{CM_GPIO->PFSRD8 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_UL_GPO   do{CM_GPIO->PFSRD8 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-3-V
*******************************************************************************/
#if(TIM4_3_VH_PB07 == TIM43_PWM_VH)
    #define INIT_TIM43_VH(X)   do{CM_GPIO->PFSRB7 = 0;CM_GPIO->PCRB7 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_VH_PWM   do{CM_GPIO->PFSRB7 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_VH_GPO   do{CM_GPIO->PFSRB7 = 0;}while(0);    /** work as GPO */
#elif(TIM4_3_VH_PC11 == TIM43_PWM_VH)
    #define INIT_TIM43_VH(X)   do{CM_GPIO->PFSRC11 = 0;CM_GPIO->PCRC11 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM43_VH_PWM   do{CM_GPIO->PFSRC11 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_VH_GPO   do{CM_GPIO->PFSRC11 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_3_VL_PB06 == TIM43_PWM_VL)
    #define INIT_TIM43_VL(X)   do{CM_GPIO->PFSRB6 = 0;CM_GPIO->PCRB6 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_VL_PWM   do{CM_GPIO->PFSRB6 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_VL_GPO   do{CM_GPIO->PFSRB6 = 0;}while(0);    /** work as GPO */
#elif(TIM4_3_VL_PD09 == TIM43_PWM_VL)
    #define INIT_TIM43_VL(X)   do{CM_GPIO->PFSRD9 = 0;CM_GPIO->PCRD9 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_VL_PWM   do{CM_GPIO->PFSRD9 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_VL_GPO   do{CM_GPIO->PFSRD9 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** TIM4-3-W
*******************************************************************************/
#if(TIM4_3_WH_PB05 == TIM43_PWM_WH)
    #define INIT_TIM43_WH(X)   do{CM_GPIO->PFSRB5 = 0;CM_GPIO->PCRB5 = (0x32|(X));}while(0);   /** work as GPO, output specified level*/
    #define SET_TIM43_WH_PWM   do{CM_GPIO->PFSRB5 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_WH_GPO   do{CM_GPIO->PFSRB5 = 0;}while(0);    /** work as GPO */
#elif(TIM4_3_WH_PC12 == TIM43_PWM_WH)
    #define INIT_TIM43_WH(X)   do{CM_GPIO->PFSRC12 = 0;CM_GPIO->PCRC12 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM43_WH_PWM   do{CM_GPIO->PFSRC12 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_WH_GPO   do{CM_GPIO->PFSRC12 = 0;}while(0);    /** work as PWM */
#endif

#if(TIM4_3_WL_PA12 == TIM43_PWM_WL)
    #define INIT_TIM43_WL(X)   do{CM_GPIO->PFSRA12 = 0;CM_GPIO->PCRA12 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM43_WL_PWM   do{CM_GPIO->PFSRA12 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_WL_GPO   do{CM_GPIO->PFSRA12 = 0;}while(0);    /** work as GPO */
#elif(TIM4_3_WL_PD10 == TIM43_PWM_WL)
    #define INIT_TIM43_WL(X)   do{CM_GPIO->PFSRD10 = 0;CM_GPIO->PCRD10 = (0x32|(X));}while(0); /** work as GPO, output specified level*/
    #define SET_TIM43_WL_PWM   do{CM_GPIO->PFSRD10 = 2;}while(0);    /** work as PWM */
    #define SET_TIM43_WL_GPO   do{CM_GPIO->PFSRD10 = 0;}while(0);    /** work as PWM */
#endif

/*******************************************************************************
** IO macros for MOTOR-1
*******************************************************************************/
#if(0x10 == (0xF0&MTR1_PWM_UH&MTR1_PWM_UL&MTR1_PWM_VH&MTR1_PWM_VL&MTR1_PWM_WH&MTR1_PWM_WL))
    #define INIT_MTR1_UH(X)   INIT_TIM41_UH(X)
    #define INIT_MTR1_UL(X)   INIT_TIM41_UL(X)
    #define INIT_MTR1_VH(X)   INIT_TIM41_VH(X)
    #define INIT_MTR1_VL(X)   INIT_TIM41_VL(X)
    #define INIT_MTR1_WH(X)   INIT_TIM41_WH(X)
    #define INIT_MTR1_WL(X)   INIT_TIM41_WL(X)

    #define SET_MTR1_UH_GPO   SET_TIM41_UH_GPO
    #define SET_MTR1_UL_GPO   SET_TIM41_UL_GPO
    #define SET_MTR1_VH_GPO   SET_TIM41_VH_GPO
    #define SET_MTR1_VL_GPO   SET_TIM41_VL_GPO
    #define SET_MTR1_WH_GPO   SET_TIM41_WH_GPO
    #define SET_MTR1_WL_GPO   SET_TIM41_WL_GPO

    #define SET_MTR1_UH_PWM   SET_TIM41_UH_PWM
    #define SET_MTR1_UL_PWM   SET_TIM41_UL_PWM
    #define SET_MTR1_VH_PWM   SET_TIM41_VH_PWM
    #define SET_MTR1_VL_PWM   SET_TIM41_VL_PWM
    #define SET_MTR1_WH_PWM   SET_TIM41_WH_PWM
    #define SET_MTR1_WL_PWM   SET_TIM41_WL_PWM
#elif(0x20 == (0xF0&MTR1_PWM_UH&MTR1_PWM_UL&MTR1_PWM_VH&MTR1_PWM_VL&MTR1_PWM_WH&MTR1_PWM_WL))
    #define INIT_MTR1_UH(X)   INIT_TIM42_UH(X)
    #define INIT_MTR1_UL(X)   INIT_TIM42_UL(X)
    #define INIT_MTR1_VH(X)   INIT_TIM42_VH(X)
    #define INIT_MTR1_VL(X)   INIT_TIM42_VL(X)
    #define INIT_MTR1_WH(X)   INIT_TIM42_WH(X)
    #define INIT_MTR1_WL(X)   INIT_TIM42_WL(X)

    #define SET_MTR1_UH_GPO   SET_TIM42_UH_GPO
    #define SET_MTR1_UL_GPO   SET_TIM42_UL_GPO
    #define SET_MTR1_VH_GPO   SET_TIM42_VH_GPO
    #define SET_MTR1_VL_GPO   SET_TIM42_VL_GPO
    #define SET_MTR1_WH_GPO   SET_TIM42_WH_GPO
    #define SET_MTR1_WL_GPO   SET_TIM42_WL_GPO

    #define SET_MTR1_UH_PWM   SET_TIM42_UH_PWM
    #define SET_MTR1_UL_PWM   SET_TIM42_UL_PWM
    #define SET_MTR1_VH_PWM   SET_TIM42_VH_PWM
    #define SET_MTR1_VL_PWM   SET_TIM42_VL_PWM
    #define SET_MTR1_WH_PWM   SET_TIM42_WH_PWM
    #define SET_MTR1_WL_PWM   SET_TIM42_WL_PWM
#elif(0x40 == (0xF0&MTR1_PWM_UH&MTR1_PWM_UL&MTR1_PWM_VH&MTR1_PWM_VL&MTR1_PWM_WH&MTR1_PWM_WL))
    #define INIT_MTR1_UH(X)   INIT_TIM43_UH(X)
    #define INIT_MTR1_UL(X)   INIT_TIM43_UL(X)
    #define INIT_MTR1_VH(X)   INIT_TIM43_VH(X)
    #define INIT_MTR1_VL(X)   INIT_TIM43_VL(X)
    #define INIT_MTR1_WH(X)   INIT_TIM43_WH(X)
    #define INIT_MTR1_WL(X)   INIT_TIM43_WL(X)

    #define SET_MTR1_UH_GPO   SET_TIM43_UH_GPO
    #define SET_MTR1_UL_GPO   SET_TIM43_UL_GPO
    #define SET_MTR1_VH_GPO   SET_TIM43_VH_GPO
    #define SET_MTR1_VL_GPO   SET_TIM43_VL_GPO
    #define SET_MTR1_WH_GPO   SET_TIM43_WH_GPO
    #define SET_MTR1_WL_GPO   SET_TIM43_WL_GPO

    #define SET_MTR1_UH_PWM   SET_TIM43_UH_PWM
    #define SET_MTR1_UL_PWM   SET_TIM43_UL_PWM
    #define SET_MTR1_VH_PWM   SET_TIM43_VH_PWM
    #define SET_MTR1_VL_PWM   SET_TIM43_VL_PWM
    #define SET_MTR1_WH_PWM   SET_TIM43_WH_PWM
    #define SET_MTR1_WL_PWM   SET_TIM43_WL_PWM
#endif

/*******************************************************************************
** IO macros for MOTOR-2
*******************************************************************************/
#if(0x10 == (0xF0&MTR2_PWM_UH&MTR2_PWM_UL&MTR2_PWM_VH&MTR2_PWM_VL&MTR2_PWM_WH&MTR2_PWM_WL))
    #define INIT_MTR2_UH(X)   INIT_TIM41_UH(X)
    #define INIT_MTR2_UL(X)   INIT_TIM41_UL(X)
    #define INIT_MTR2_VH(X)   INIT_TIM41_VH(X)
    #define INIT_MTR2_VL(X)   INIT_TIM41_VL(X)
    #define INIT_MTR2_WH(X)   INIT_TIM41_WH(X)
    #define INIT_MTR2_WL(X)   INIT_TIM41_WL(X)

    #define SET_MTR2_UH_GPO   SET_TIM41_UH_GPO
    #define SET_MTR2_UL_GPO   SET_TIM41_UL_GPO
    #define SET_MTR2_VH_GPO   SET_TIM41_VH_GPO
    #define SET_MTR2_VL_GPO   SET_TIM41_VL_GPO
    #define SET_MTR2_WH_GPO   SET_TIM41_WH_GPO
    #define SET_MTR2_WL_GPO   SET_TIM41_WL_GPO

    #define SET_MTR2_UH_PWM   SET_TIM41_UH_PWM
    #define SET_MTR2_UL_PWM   SET_TIM41_UL_PWM
    #define SET_MTR2_VH_PWM   SET_TIM41_VH_PWM
    #define SET_MTR2_VL_PWM   SET_TIM41_VL_PWM
    #define SET_MTR2_WH_PWM   SET_TIM41_WH_PWM
    #define SET_MTR2_WL_PWM   SET_TIM41_WL_PWM
#elif(0x20 == (0xF0&MTR2_PWM_UH&MTR2_PWM_UL&MTR2_PWM_VH&MTR2_PWM_VL&MTR2_PWM_WH&MTR2_PWM_WL))
    #define INIT_MTR2_UH(X)   INIT_TIM42_UH(X)
    #define INIT_MTR2_UL(X)   INIT_TIM42_UL(X)
    #define INIT_MTR2_VH(X)   INIT_TIM42_VH(X)
    #define INIT_MTR2_VL(X)   INIT_TIM42_VL(X)
    #define INIT_MTR2_WH(X)   INIT_TIM42_WH(X)
    #define INIT_MTR2_WL(X)   INIT_TIM42_WL(X)

    #define SET_MTR2_UH_GPO   SET_TIM42_UH_GPO
    #define SET_MTR2_UL_GPO   SET_TIM42_UL_GPO
    #define SET_MTR2_VH_GPO   SET_TIM42_VH_GPO
    #define SET_MTR2_VL_GPO   SET_TIM42_VL_GPO
    #define SET_MTR2_WH_GPO   SET_TIM42_WH_GPO
    #define SET_MTR2_WL_GPO   SET_TIM42_WL_GPO

    #define SET_MTR2_UH_PWM   SET_TIM42_UH_PWM
    #define SET_MTR2_UL_PWM   SET_TIM42_UL_PWM
    #define SET_MTR2_VH_PWM   SET_TIM42_VH_PWM
    #define SET_MTR2_VL_PWM   SET_TIM42_VL_PWM
    #define SET_MTR2_WH_PWM   SET_TIM42_WH_PWM
    #define SET_MTR2_WL_PWM   SET_TIM42_WL_PWM
#elif(0x40 == (0xF0&MTR2_PWM_UH&MTR2_PWM_UL&MTR2_PWM_VH&MTR2_PWM_VL&MTR2_PWM_WH&MTR2_PWM_WL))
    #define INIT_MTR2_UH(X)   INIT_TIM43_UH(X)
    #define INIT_MTR2_UL(X)   INIT_TIM43_UL(X)
    #define INIT_MTR2_VH(X)   INIT_TIM43_VH(X)
    #define INIT_MTR2_VL(X)   INIT_TIM43_VL(X)
    #define INIT_MTR2_WH(X)   INIT_TIM43_WH(X)
    #define INIT_MTR2_WL(X)   INIT_TIM43_WL(X)

    #define SET_MTR2_UH_GPO   SET_TIM43_UH_GPO
    #define SET_MTR2_UL_GPO   SET_TIM43_UL_GPO
    #define SET_MTR2_VH_GPO   SET_TIM43_VH_GPO
    #define SET_MTR2_VL_GPO   SET_TIM43_VL_GPO
    #define SET_MTR2_WH_GPO   SET_TIM43_WH_GPO
    #define SET_MTR2_WL_GPO   SET_TIM43_WL_GPO

    #define SET_MTR2_UH_PWM   SET_TIM43_UH_PWM
    #define SET_MTR2_UL_PWM   SET_TIM43_UL_PWM
    #define SET_MTR2_VH_PWM   SET_TIM43_VH_PWM
    #define SET_MTR2_VL_PWM   SET_TIM43_VL_PWM
    #define SET_MTR2_WH_PWM   SET_TIM43_WH_PWM
    #define SET_MTR2_WL_PWM   SET_TIM43_WL_PWM
#endif

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

#endif
