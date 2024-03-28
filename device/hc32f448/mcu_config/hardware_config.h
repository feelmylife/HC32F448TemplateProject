/**
 *******************************************************************************
 * @file  hardware_config.h
 * @brief muc hardware config information  define.
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

#ifndef MCU_CONFIG_HARDWARE_CONFIG_H_
#define MCU_CONFIG_HARDWARE_CONFIG_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_config/hardware_definitions.h"
#include "motor_midwares/base_types.h"


/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
#define USE_HSRC                    TRUE            // use internal HSRC or external XTAL
#define USE_WATCHDOG                FALSE       /* enable or disable hardware watch dog */
#define WATCHDOG_MODE               WATCHDOG_REST   /* WATCHDOG_REST or WATCHDOG_IRQ */

#if(TRUE == USE_HSRC)
#define CLK_HSRC_FREQ               (16)              // frequency of HSRC (MHz)
#else
#define CLK_XTAL_FREQ               (8)               // frequency of XTAL (MHz)
#endif

#if(TRUE == USE_HSRC)
#define CLK_MPLL_N_MUL  (30L)   /** 20 <= N <= 480; 240 <= N*SRC_CLK <= 480 */
#define CLK_MPLL_P_DIV  (4L)    /** 2 <= P <= 16, SRC_CLK*N/P <= 200MHz, system clock */
#define CLK_MPLL_Q_DIV  (8L)    /** 2 <= Q <= 16, SRC_CLK*N/Q <= 60MHz, ADC clock */
#define CLK_MPLL_R_DIV  (8L)    /** 2 <= R <= 16 */
#else
#define CLK_MPLL_N_MUL  (30L)   /** 20 <= N <= 480; 240 <= N*SRC_CLK <= 480 */
#define CLK_MPLL_P_DIV  (2L)    /** 2 <= P <= 16, SRC_CLK*N/P <= 200MHz, system clock */
#define CLK_MPLL_Q_DIV  (4L)    /** 2 <= Q <= 16, SRC_CLK*N/Q <= 60MHz, ADC clock */
#define CLK_MPLL_R_DIV  (4L)    /** 2 <= R <= 16 */
#endif



/*******************************************************************************
** basic drive
*******************************************************************************/
#define SWAP_ROTOR_DIR      FALSE   /** TRUE: swap default direction, FALSE: default direction */
#define EN_DT_VOLT_RECOVER  FALSE    /** TRUE: apply dead-time voltage recovery, FALSE: none recovery */

/*******************************************************************************
** command source, default are all disabled
*******************************************************************************/
#define EN_VSP_CMD          TRUE   /** TRUE: enable potentiometer as command */



/*******************************************************************************
** MOTOR-1 PWM generation
*******************************************************************************/
/*******************************************************************************
** |     |  TIM4-1(EMB_IN2)  |  TIM4-2(EMB_IN3)  |  TIM4-3(EMB_IN4)  |
** |_____|___________________|___________________|___________________|
** | UH  | PA08, PE09        | PA00, PC04        | PB09, PC10        |
** | UL  | PA07, PB13, PE08  | PA01, PC05        | PB08, PD08        |
** |_____|___________________|___________________|___________________|
** | VH  | PA09, PE11        | PA02, PB10        | PB07, PC11        |
** | VL  | PB00, PB14, PE10  | PA03, PB12        | PB06, PD09        |
** |_____|___________________|___________________|___________________|
** | WH  | PA10, PE13        | PA04, PC08        | PB05, PC12        |
** | WL  | PB01, PB15, PE12  | PA05, PC09        | PA12, PD10        |
** |_____|___________________|___________________|___________________|
** | EMB | PA06, PB12, PE15  | PA07, PC02        | PB15, PH02        |
*******************************************************************************/
#define PWM_ACTIVE_LEVEL    PWM_ACTIVE_LEVEL_HH /** PWM active-level setting **/

#define MTR1_PWM_UH         TIM4_1_UH_PA08
#define MTR1_PWM_UL         TIM4_1_UL_PB13

#define MTR1_PWM_VH         TIM4_1_VH_PA09
#define MTR1_PWM_VL         TIM4_1_VL_PB14

#define MTR1_PWM_WH         TIM4_1_WH_PA10
#define MTR1_PWM_WL         TIM4_1_WL_PB15

/*******************************************************************************
** ADC sampling: default use ADC1 for sampling
*******************************************************************************/
#define ADC_VOLT_REF        (3.3f)      /** reference voltage of ADC        */
#define ADC_BIT_LENGTH      12          /** 12-bit ADC                      */
#define SHUNT_NUM           SHUNT_NUM_2 /** current sample shunt numbers    */

#define ADC_CH_VDC          3U           /** VDC sample channel              */
#define ADC_CH_IU           0U           /** IU  sample channel              */
#define ADC_CH_IV           1U           /** IV  sample channel, case single-shunt, ignore */
#define ADC_CH_VSP          5U           /** VSP sample channel              */


       
/*******************************************************************************
** hardware over-current protection
*******************************************************************************/
#define HW_OCP_SRC   HW_OCP_SRC_IO     /** hardware OCP source: HW_OCP_SRC_VC or HW_OCP_SRC_IO */
#if(HW_OCP_SRC_VC == HW_OCP_SRC)
/***************************************************************************
** hardware over current signal routing:
**    1. INM: reference voltage, from internal DA, Vref = AVCC * DA_DATA / 255
**    2. INP: over-current signal, from external PIN
** |      |  CMP1  |  CMP2  |  CMP3  |
** | INP1 |  PA00  |  PA04  |  PB00  |
** | INP2 |  PA01  |  PA05  |  PB01  |
** | INP3 |  PA02  |  PA06  |  PC00  |
** | INP4 |  PA03  |    X   |  PA04  |
***************************************************************************/
#define HW_OCP_CMP_UNIT         (3)   /** 1: CMP1, 2: CMP2, 3: CMP3    */
#define HW_OCP_CMP_INP_CH       (4)   /** INP channel, 1, 2, 3, 4      */
#define HW_OCP_CMP_DA_DATA      (14)   /** set DA output voltage, 0~255 */
#define INIT_HW_OCP_CMP_INP     do{bCM_GPIO->PCRA4_b.DDIS = 1;}while(0);
#elif(HW_OCP_SRC_IO == HW_OCP_SRC)
#define HW_OCP_EMB_LEVEL    (0)       /** EMB trigger level, 0: low, 1: high*/
#define INIT_HW_OCP_EMB_IO  do{CM_GPIO->PFSRB12 = 6;}while(0);
#endif

                                   
/*******************************************************************************
** GPIOs
*******************************************************************************/
#define EN_TEST_IO      FALSE
#if(TRUE == EN_TEST_IO)
#define INIT_TEST_GPIO_OUT(X) do{CM_GPIO->PWPR = 0xA501;\
                                     CM_GPIO->PCRB5 = (0x32|(X));\
                                     CM_GPIO->PWPR = 0x0000;}while(0);
#define SET_TEST_GPIO_OUT(X)  do{CM_GPIO->PWPR = 0xA501;\
                                     bCM_GPIO->PCRB5_b.POUT = (X);\
                                     CM_GPIO->PWPR = 0x0000;}while(0);

#endif

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
extern int32_t i32_CLK_FREQ;                   // frequency of HSRC or XTAL (MHz)

extern uint8_t u8_ADC_CH_VDC;           /** VDC sample channel              */
extern uint8_t u8_ADC_CH_IU;           /** IU  sample channel              */
extern uint8_t u8_ADC_CH_IV;           /** IV  sample channel, case single-shunt, ignore */
extern uint8_t u8_ADC_CH_VSP;           /** VSP sample channel              */


#endif /* MCU_CONFIG_HARDWARE_CONFIG_H_ */
