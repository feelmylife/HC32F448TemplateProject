/**
 *******************************************************************************
 * @file  init_mcu.c
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


/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu_common/mcu_include.h"
/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* ICG configuration, refer to manual to modify                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/
stc_SysClk_t   gstcSysClk;
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
#if(USE_REG_DRIVER_MODE == USE_DRIVER_MODE)

#endif
/**
 ******************************************************************************
 ** \brief initialize NVIC
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_Nvic(void)
{
  /***************************************************************************
   ** watchdog interrupt
   **************************************************************************/
  CM_INTC->INTSEL0 = (uint32_t)INT_SRC_WDT_REFUDF;     /* set interrupt event */
  NVIC_ClearPendingIRQ(INT000_IRQn);      /* clear pending IRQ */
  NVIC_SetPriority(INT000_IRQn, 0);       /* set IRQ priority */
  NVIC_EnableIRQ(INT000_IRQn);            /* enable IRQ */

#if (1 == MTR1_TIM4_UNIT)
  CM_INTC->INTSEL1 = (uint32_t)INT_SRC_EMB_GR1;        /** IRQ0 for hardware over current */
  CM_INTC->INTSEL2 = (uint32_t)INT_SRC_TMR4_1_UDF;     /** IRQ1 for zero-match of TMR4 */
#elif (2 == MTR1_TIM4_UNIT)
  CM_INTC->INTSEL1 = (uint32_t)INT_SRC_EMB_GR2;        /** IRQ0 for hardware over current */
  CM_INTC->INTSEL2 = (uint32_t)INT_SRC_TMR4_2_UDF;     /** IRQ1 for zero-match of TMR4 */
#elif (3 == MTR1_TIM4_UNIT)
  CM_INTC->INTSEL1 = (uint32_t)INT_SRC_EMB_GR3;        /** IRQ0 for hardware over current */
  CM_INTC->INTSEL2 = (uint32_t)INT_SRC_TMR4_3_UDF;     /** IRQ1 for zero-match of TMR4 */
#endif

  /***************************************************************************
   ** hardware over-current interrupt:
   ** timer6   -   EMB0    - INT_SRC_EMB_GR0
   ** timer41  -   EMB1    - INT_SRC_EMB_GR1
   ** timer42  -   EMB2    - INT_SRC_EMB_GR2
   ** timer43  -   EMB3    - INT_SRC_EMB_GR3
   **************************************************************************/
  NVIC_ClearPendingIRQ(INT001_IRQn);      /* clear pending IRQ */
  NVIC_SetPriority(INT001_IRQn, 1);       /* set IRQ priority */
  NVIC_EnableIRQ(INT001_IRQn);            /* enable IRQ */

  /***************************************************************************
   ** timer-42 zero-match interrupt
   **************************************************************************/
  NVIC_ClearPendingIRQ(INT002_IRQn);      /* clear pending IRQ */
  NVIC_SetPriority(INT002_IRQn, 2);       /* set IRQ priority */
  NVIC_EnableIRQ(INT002_IRQn);            /* enable IRQ */

}


#if(USE_REG_DRIVER_MODE == USE_DRIVER_MODE)
/**
 ******************************************************************************
 ** \brief initialize clocks of MCU, use PLL as main clock
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/

uint32_t InitMcu_Clock(void)
{
  int32_t i32SrcClockHz;
  int32_t i32PllClockHz;
  int32_t i32DelayCnt = 0;
  uint32_t u32Err = 0;
  uint16_t u16Mpll_N = CLK_MPLL_N_MUL;
  uint16_t u16Mpll_P = CLK_MPLL_P_DIV;
  uint16_t u16Mpll_Q = CLK_MPLL_Q_DIV;
  uint16_t u16Mpll_R = CLK_MPLL_R_DIV;
  uint8_t u8FlashWait;
  uint8_t u8Sram123Wait;
  uint8_t u8XtalDrv = 0;

  uint8_t u8GPIOWait = 0;

#if(TRUE == USE_HSRC)
  i32SrcClockHz = i32_CLK_FREQ * 1000 * 1000;
#else
  i32SrcClockHz = i32_CLK_FREQ * 1000 * 1000;

  if(i32_CLK_FREQ < 4)
  {
    u32Err = ERR_SYS_CLK;
  }
  else if(i32_CLK_FREQ < 8)
  {
    u8XtalDrv = 3U;      /** 0x3: 4~8MHz  */
  }
  else if(i32_CLK_FREQ < 16)
  {
    u8XtalDrv = 2U;      /** 0x2: 8~16MHz  */
  }
  else if(i32_CLK_FREQ < 20)
  {
    u8XtalDrv = 1U;      /** 0x1: 16~20MHz */
  }
  else if(i32_CLK_FREQ <= 24)
  {
    u8XtalDrv = 0U;      /** 0x0: 20~24MHz */
  }
  else
  {
    u32Err = ERR_SYS_CLK;
  }

#endif

  /***************************************************************************
   ** check validity of ADC clock
   **************************************************************************/
  i32PllClockHz = i32SrcClockHz * (int32_t)u16Mpll_N / (int32_t)u16Mpll_Q;
  if (i32PllClockHz > 60 * 1000 * 1000)
  {
    u32Err = ERR_SYS_CLK;
  }

  /***************************************************************************
   ** check validity of MPLL clock
   **************************************************************************/
  i32PllClockHz = i32SrcClockHz * (int32_t)u16Mpll_N;
  if ((i32PllClockHz < 240 * 1000 * 1000) || (i32PllClockHz > 480 * 1000 * 1000))
  {
    u32Err = ERR_SYS_CLK;
  }
  i32PllClockHz = i32PllClockHz / (int32_t)u16Mpll_P;
  if (i32PllClockHz > 200 * 1000 * 1000)
  {
    u32Err = ERR_SYS_CLK;
  }

  /***************************************************************************
   ** case parameter error, set HRC as source of PLL, and 120MHz system clock
   **************************************************************************/
  if (0U != u32Err) {

    u16Mpll_N = 30;
    u16Mpll_P = 4;
    u16Mpll_Q = 8;
    u16Mpll_R = 8;
    i32SrcClockHz = 16 * 1000 * 1000;
    i32PllClockHz = 120 * 1000 * 1000;
  }


  /***************************************************************************
   ** calculate flash wait cycle and SRAM wait cycle
   **************************************************************************/
  if (i32PllClockHz <= 33 * 1000 * 1000)
  {
    u8FlashWait = 0;
  }
  else if (i32PllClockHz <= 66 * 1000 * 1000)
  {
    u8FlashWait = 1;
  }
  else if (i32PllClockHz <= 99 * 1000 * 1000)
  {
    u8FlashWait = 2;
  }
  else if (i32PllClockHz <= 132 * 1000 * 1000)
  {
    u8FlashWait = 3;
  }
  else if (i32PllClockHz <= 168 * 1000 * 1000)
  {
    u8FlashWait = 4;
  }
  else
  {
    u8FlashWait = 5;
  }

  /***************************************************************************
   ** calculate SRAM wait-cycle
   **************************************************************************/
  if (i32PllClockHz <= 100 * 1000 * 1000)
  {
    u8Sram123Wait = 0;
  }
  else
  {
    u8Sram123Wait = 1;
  }

  if(i32PllClockHz <= 42 * 1000 * 1000)
  {
    u8GPIOWait = 0u;
  }
  else if((i32PllClockHz > 42 * 1000 * 1000) && (i32PllClockHz <= 84 * 1000 * 1000))
  {
    u8GPIOWait = 1u;
  }
  else if((i32PllClockHz > 84 * 1000 * 1000) && (i32PllClockHz <= 126 * 1000 * 1000))
  {
    u8GPIOWait = 2u;
  }
  else if((i32PllClockHz > 126 * 1000 * 1000) && (i32PllClockHz <= 200 * 1000 * 1000))
  {
    u8GPIOWait = 3u;
  }
  else
  {
      
  }


  /***************************************************************************
   ** set flash wait-cycle according to system clock
   **************************************************************************/
  CM_EFM->FAPRT = 0x0123U;
  CM_EFM->FAPRT = 0x3210U;

  /** insert 4 wait cycle for flash */
  MODIFY_REG32(CM_EFM->FRMC, EFM_FRMC_FLWT, ((uint32_t)u8FlashWait << EFM_FRMC_FLWT_POS));
  /** open the catch */
  MODIFY_REG32(CM_EFM->FRMC, EFM_FRMC_ICACHE | EFM_FRMC_DCACHE, (0x1u << EFM_FRMC_ICACHE_POS)|(0x1u << EFM_FRMC_DCACHE_POS));
  /** lock flash write protection */
  CM_EFM->FAPRT = 0x3210;
  CM_EFM->FAPRT = 0x3210;

  /***************************************************************************
   ** initialize SRAM wait cycle
   **************************************************************************/
  CM_SRAMC->WTPR = 0x77U;             /** unlock SRAM read/write protection */

  CM_SRAMC->WTCR = (((uint32_t)u8Sram123Wait << SRAMC_WTCR_SRAM0RWT_POS) | ((uint32_t)u8Sram123Wait << SRAMC_WTCR_SRAM0WWT_POS)
                  | (0U << SRAMC_WTCR_SRAMHRWT_POS) | (0U << SRAMC_WTCR_SRAMHWWT_POS)
                  | ((uint32_t)u8Sram123Wait << SRAMC_WTCR_SRAMBRWT_POS) | ((uint32_t)u8Sram123Wait << SRAMC_WTCR_SRAMBWWT_POS));

  CM_SRAMC->WTPR = 0x76U;             /** lock SRAM read/write protection */

  /***************************************************************************
   ** set MCU in high performance mode, stable-time is required(1ms)
   **************************************************************************/
  CM_PWC->FPRC   = 0xA502;
  CM_PWC->PWRC2  = 0xF0;
  CM_PWC->PWRC3  = 0x0F;
  CM_PWC->FPRC   = 0xA500;
  for (i32DelayCnt = 0; i32DelayCnt < 8000; i32DelayCnt++)   /** >1ms @8MHz */
  {
    __NOP();
  }

  /***************************************************************************
   ** since MPLL is going to be set, FCG should restore to default before
   ** setting bus clock divider. stable-time(30us) is required
   **************************************************************************/
  CM_PWC->FCG0PC = 0xA5A50001UL;   /** unlock FCG0 */
  CM_PWC->FCG0 = 0xFFFFFAEEUL;
  CM_PWC->FCG1 = 0xFFFFFFFFUL;
  CM_PWC->FCG2 = 0xFFFFFFFFUL;
  CM_PWC->FCG3 = 0xFFFFFFFFUL;
  for (i32DelayCnt = 0; i32DelayCnt < 300; i32DelayCnt++)   /** >30us @8MHz */
  {
    __NOP();
  }
  CM_PWC->FCG0PC = 0xA5A50000UL;    /** lock FCG0 */

  /***************************************************************************
   ** start clock configuration
   **************************************************************************/
  CM_PWC->FPRC = 0xA501;                   /** 0xA501: unlock CMU */
  /***************************************************************************
   ** set bus clock divider, stable-time is required (30us)
   ** 1. AHB   = MPLL_N = SRC_CLK*N/P
   ** 2. APB0  = AHB   (max.@200MHz)
   ** 3. APB1  = AHB/2 (max.@100MHz)
   ** 4. APB2  = APB4  (max.@60MHz)   = MPLL_Q = SRC_CLK*N/Q , for ADC
   ** 5. APB3  = AHB/4 (max.@50MHz)
   ** 6. EXCLK = AHB/2 (max.@100MHz)
   **************************************************************************/

  CM_CMU->SCFGR = 0;           /** 0x0: AHB = APB0 = system clock */

  CM_CMU->SCFGR |= (((uint32_t)1 << CMU_SCFGR_PCLK1S_POS) | ((uint32_t)2 << CMU_SCFGR_PCLK2S_POS)
                  | ((uint32_t)2 << CMU_SCFGR_PCLK3S_POS) | ((uint32_t)2 << CMU_SCFGR_PCLK4S_POS)
                  | ((uint32_t)1 << CMU_SCFGR_EXCKS_POS));

  for (i32DelayCnt = 0; i32DelayCnt < 300; i32DelayCnt++)         /** >30us @8MHz */
  {
    __NOP();
  }

  if ((0U != u32Err) || (TRUE == USE_HSRC))
  {
    MODIFY_REG8(CM_CMU->HRCCR, CMU_HRCCR_HRCSTP, 0x0U);  /** 0x0: start HRC */
    while (0U == READ_REG8_BIT(CM_CMU->OSCSTBSR, CMU_OSCSTBSR_HRCSTBF)) /** 0x0: start HRC oscillation */
    {
      ;
    }
    MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLSRC, (0x1U << CMU_PLLHCFGR_PLLSRC_POS));    /** 0x1: HRC as source of PLL */
  }
  else
  {
    CM_GPIO->PWPR = 0xA501;     /** unlock write protection of PCRxy  */
    bCM_GPIO->PCRH0_b.DDIS = 1; /** disable digital function of XTAL_IN */
    bCM_GPIO->PCRH1_b.DDIS = 1; /** disable digital function of XTAL_OUT */
    CM_GPIO->PWPR = 0xA500;     /** unlock write protection of PCRxy  */
    MODIFY_REG8(CM_CMU->XTALCFGR, CMU_XTALCFGR_XTALMS, (0x0u << CMU_XTALCFGR_XTALMS_POS));  /** 0x0: oscillator mode */
    MODIFY_REG8(CM_CMU->XTALCFGR, CMU_XTALCFGR_XTALDRV, u8XtalDrv << (CMU_XTALCFGR_XTALDRV_POS));   /** set XTAL drive capability */
    MODIFY_REG8(CM_CMU->XTALCR, CMU_XTALCR_XTALSTP, 0x0u);                  /** 0x0: start XTAL oscillation */

    while (0U == READ_REG8_BIT(CM_CMU->OSCSTBSR, CMU_OSCSTBSR_XTALSTBF)) /** wait till XTAL stable */
    {
      ;
    }
    MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLSRC, 0x0u << CMU_PLLHCFGR_PLLSRC_POS);  /** 0x0: XTAL as source of PLL */
  }

  for (i32DelayCnt = 0; i32DelayCnt < 8000; i32DelayCnt++)
  { /** >1ms @8MHz */
    __NOP();
  }

  MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLHP, (((uint32_t)u16Mpll_P - 1U) << CMU_PLLHCFGR_PLLHP_POS));
  MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLHQ, (((uint32_t)u16Mpll_Q - 1U) << CMU_PLLHCFGR_PLLHQ_POS));
  MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLHR, (((uint32_t)u16Mpll_R - 1U) << CMU_PLLHCFGR_PLLHR_POS));
  MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLHN, (((uint32_t)u16Mpll_N - 1U) << CMU_PLLHCFGR_PLLHN_POS));
  MODIFY_REG32(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLHM, (0 << CMU_PLLHCFGR_PLLHM));

  /***************************************************************************
   ** start MPLL and wait till stable
   **************************************************************************/
  MODIFY_REG8(CM_CMU->PLLHCR, CMU_PLLHCR_PLLHOFF, 0x0u);        /** start MPLL */

  while (0U == READ_REG8_BIT(CM_CMU->OSCSTBSR, CMU_OSCSTBSR_PLLHSTBF)) /** wait till PLL stable */
  {
    ;
  }

  MODIFY_REG8(CM_CMU->CKSWR, CMU_CKSWR_CKSW, 0x05U);;              /** set MPLL as clock source */
  CM_PWC->FPRC = 0xA500;                   /** 0xA500: lock CMU */
  CM_PWC->FPRC = 0xA502;                   /** 0xA502: unlock PERICKSEL */
  MODIFY_REG16(CM_CMU->PERICKSEL, CMU_PERICKSEL_PERICKSEL, 0x9u);   /** 0x9: MPLLQ as clock source of ADC */
  CM_PWC->FPRC = 0xA500;                   /** 0xA500: lock PERICKSEL */

  /***************************************************************************
   ** Set the number of wait cycles inserted when reading registers PIDRx, PCRxy
   **************************************************************************/
  CM_GPIO->PWPR = 0xA501;     /** unlock write protection of PCCR  */
  MODIFY_REG16(CM_GPIO->PCCR, GPIO_PCCR_RDWT, ((uint16_t)u8GPIOWait << GPIO_PCCR_RDWT_POS));
  CM_GPIO->PWPR = 0xA500;     /** unlock write protection of PCCR  */


  /***************************************************************************
   ** record configuration
   **************************************************************************/
  gstcSysClk.i32_AHBClk_Hz = i32SrcClockHz * (int32_t)u16Mpll_N / (int32_t)u16Mpll_P;
  gstcSysClk.i32_APB0Clk_Hz = gstcSysClk.i32_AHBClk_Hz;
  gstcSysClk.i32_APB1Clk_Hz = (gstcSysClk.i32_AHBClk_Hz >> 1);
  gstcSysClk.i32_APB3Clk_Hz = (gstcSysClk.i32_AHBClk_Hz >> 2);
  gstcSysClk.i32_ExClk_Hz = (gstcSysClk.i32_AHBClk_Hz >> 1);

  gstcSysClk.i32_APB2Clk_Hz = i32SrcClockHz * (int32_t)u16Mpll_N / (int32_t)u16Mpll_Q;
  gstcSysClk.i32_APB4Clk_Hz = gstcSysClk.i32_APB2Clk_Hz;

  return u32Err;

}
#elif(USE_DDL_DRIVER_MODE == USE_DRIVER_MODE)

uint32_t InitMcu_Clock(void)
{
  int32_t i32SrcClockHz;
  int32_t i32PllClockHz;
    
  uint32_t u32Err = 0;
  uint16_t u16Mpll_N = CLK_MPLL_N_MUL;
  uint16_t u16Mpll_P = CLK_MPLL_P_DIV;
  uint16_t u16Mpll_Q = CLK_MPLL_Q_DIV;
  uint16_t u16Mpll_R = CLK_MPLL_R_DIV;
  uint8_t u8FlashWait;
  uint8_t u8Sram123Wait;
    
  uint16_t u16GPIOWait = 0;
    
  uint16_t u16Temp;

  stc_clock_pll_init_t      stcMpllInit;
    
  uint8_t u8XtalDrv = 0;
  stc_clock_xtal_init_t     stcXtalInit;
  (void)CLK_XtalStructInit(&stcXtalInit);
    
  (void)CLK_PLLStructInit(&stcMpllInit);


#if(TRUE == USE_HSRC)
  i32SrcClockHz = i32_CLK_FREQ * 1000 * 1000;
#else
  i32SrcClockHz = i32_CLK_FREQ * 1000 * 1000;
    
  if(i32SrcClockHz < 4 * 1000 * 1000)
  {
    u32Err = ERR_SYS_CLK;
  }
  else if (i32SrcClockHz < 8 * 1000 * 1000)
  {
    u8XtalDrv = 3U;      /** 0x3: 4~8MHz  */
  }
  else if (i32SrcClockHz < 16 * 1000 * 1000)
  {
    u8XtalDrv = 2U;      /** 0x2: 8~16MHz  */
  }
  else if (i32SrcClockHz < 20 * 1000 * 1000)
  {
    u8XtalDrv = 1U;      /** 0x1: 16~20MHz */
  }
  else if (i32SrcClockHz <= 24 * 1000 * 1000)
  {
    u8XtalDrv = 0U;      /** 0x0: 20~24MHz */
  }
  else
  {
    u32Err = ERR_SYS_CLK;
  }
#endif

  /***************************************************************************
   ** check validity of ADC clock
   **************************************************************************/
  i32PllClockHz = i32SrcClockHz * (int32_t)u16Mpll_N / (int32_t)u16Mpll_Q;
  if (i32PllClockHz > 60 * 1000 * 1000)
  {
    u32Err = ERR_SYS_CLK;
  }
  else
  {
        
  }

  /***************************************************************************
   ** check validity of MPLL clock
   **************************************************************************/
  i32PllClockHz = i32SrcClockHz * (int32_t)u16Mpll_N;
  if ((i32PllClockHz < 240 * 1000 * 1000) || (i32PllClockHz > 480 * 1000 * 1000))
  {
    u32Err = ERR_SYS_CLK;
  }
  i32PllClockHz = i32PllClockHz / (int32_t)u16Mpll_P;
  if (i32PllClockHz > 200 * 1000 * 1000)
  {
    u32Err = ERR_SYS_CLK;
  }

  /***************************************************************************
   ** case parameter error, set HRC as source of PLL, and 120MHz system clock
   **************************************************************************/
  if (0U != u32Err)
  {
    u16Mpll_N = 30U;
    u16Mpll_P = 4U;
    u16Mpll_Q = 8U;
    u16Mpll_R = 8U;
    i32SrcClockHz = 16 * 1000 * 1000;
    i32PllClockHz = 120 * 1000 * 1000;
  }

  /***************************************************************************
   ** calculate flash wait cycle and SRAM wait cycle
   **************************************************************************/
  if (i32PllClockHz <= 33 * 1000 * 1000)
  {
    u8FlashWait = 0;
  }
  else if (i32PllClockHz <= 66 * 1000 * 1000)
  {
    u8FlashWait = 1;
  }
  else if (i32PllClockHz <= 99 * 1000 * 1000)
  {
    u8FlashWait = 2;
  }
  else if (i32PllClockHz <= 132 * 1000 * 1000)
  {
    u8FlashWait = 3;
  }
  else if (i32PllClockHz <= 168 * 1000 * 1000)
  {
    u8FlashWait = 4;
  }
  else
  {
    u8FlashWait = 5;
  }

  /***************************************************************************
   ** calculate SRAM wait-cycle
   **************************************************************************/
  if (i32PllClockHz <= 100 * 1000 * 1000)
  {
     u8Sram123Wait = 0;
  }
  else
  {
    u8Sram123Wait = 1;
  }

  if(i32PllClockHz <= 42 * 1000 * 1000)
  {
      u16GPIOWait = 0u;
  }
  else if((i32PllClockHz > 42 * 1000 * 1000) && (i32PllClockHz <= 84 * 1000 * 1000))
  {
    u16GPIOWait = 1u;
  }
  else if((i32PllClockHz > 84 * 1000 * 1000) && (i32PllClockHz <= 126 * 1000 * 1000))
  {
    u16GPIOWait = 2u;
  }
  else if((i32PllClockHz > 126 * 1000 * 1000) && (i32PllClockHz <= 200 * 1000 * 1000))
  {
    u16GPIOWait = 3u;
  }
  else
  {
        
  }

  /***************************************************************************
   ** set flash wait-cycle according to system clock
   **************************************************************************/
  LL_PERIPH_WE(LL_PERIPH_EFM);

  /** insert 4 wait cycle for flash */
  /* flash read wait cycle setting */
  (void)EFM_SetWaitCycle((uint32_t)u8FlashWait << EFM_FRMC_FLWT_POS);

  /** open the catch */
  EFM_DCacheCmd(ENABLE);
  EFM_ICacheCmd(ENABLE);

  /** lock flash write protection */
  LL_PERIPH_WP(LL_PERIPH_EFM);

  /***************************************************************************
   ** initialize SRAM wait cycle
   **************************************************************************/
  LL_PERIPH_WE(LL_PERIPH_SRAM);                       /** unlock SRAM read/write protection */

  /* sram init include read/write wait cycle setting */
  SRAM_SetWaitCycle((SRAM_SRAMH | SRAM_SRAM0), SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
  SRAM_SetWaitCycle(SRAM_SRAMB, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

  LL_PERIPH_WP(LL_PERIPH_SRAM);                       /** lock SRAM read/write protection */

  /***************************************************************************
   ** set MCU in high performance mode, stable-time is required(1ms)
   **************************************************************************/
  LL_PERIPH_WE(LL_PERIPH_PWC_CLK_RMU);
  (void)PWC_LowSpeedToHighSpeed();       ///< Switch driver ability to High Speed Run Mode
  LL_PERIPH_WP(LL_PERIPH_PWC_CLK_RMU);

  /***************************************************************************
   ** since MPLL is going to be set, FCG should restore to default before
   ** setting bus clock divider. stable-time(30us) is required
   **************************************************************************/
  LL_PERIPH_WE(LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);             /** unlock FCG0 / 0xA501: unlock CMU */
  /***************************************************************************
   ** start clock configuration
   **************************************************************************/
  /***************************************************************************
   ** set bus clock divider, stable-time is required (30us)
   ** 1. AHB   = MPLL_N = SRC_CLK*N/P
   ** 2. APB0  = AHB   (max.@200MHz)
   ** 3. APB1  = AHB/2 (max.@100MHz)
   ** 4. APB2  = APB4  (max.@60MHz)   = MPLL_Q = SRC_CLK*N/Q , for ADC
   ** 5. APB3  = AHB/4 (max.@50MHz)
   ** 6. EXCLK = AHB/2 (max.@100MHz)
   **************************************************************************/
  /* Set bus clk div. */
  CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                   CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV4));

  if ((0U != u32Err) || (TRUE == USE_HSRC))
  {
    (void)CLK_HrcCmd(ENABLE);
  }
  else
  {
    /* Config Xtal and enable Xtal */
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv = u8XtalDrv << (CMU_XTALCFGR_XTALDRV_POS);
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);
  }

  /* MPLL config (XTAL / pllmDiv * plln / PllpDiv = 200M). */
  stcMpllInit.PLLCFGR = 0UL;
  stcMpllInit.PLLCFGR_f.PLLM = 0U;
    
  u16Temp = u16Mpll_N - 1U;
  stcMpllInit.PLLCFGR_f.PLLN = (uint32_t)u16Temp;
    
  u16Temp = u16Mpll_P - 1U;
  stcMpllInit.PLLCFGR_f.PLLP = (uint32_t)u16Temp;
    
  u16Temp = u16Mpll_Q - 1U;
  stcMpllInit.PLLCFGR_f.PLLQ = (uint32_t)u16Temp;
    
  u16Temp = u16Mpll_R - 1U;
  stcMpllInit.PLLCFGR_f.PLLR = (uint32_t)u16Temp;
  stcMpllInit.u8PLLState = CLK_PLL_ON;
#if(TRUE == USE_HSRC)
  stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_HRC;
#else
  stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
#endif

  (void)CLK_PLLInit(&stcMpllInit);
  /* Wait MPLL ready. */
  while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL))
  {
     ;
  }



  /***************************************************************************
   ** start MPLL and wait till stable
   **************************************************************************/
  /* Switch system clock source to MPLL. */
  CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);

  LL_PERIPH_WP(LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU);

  LL_PERIPH_WE(LL_PERIPH_PWC_CLK_RMU);

  CLK_SetPeriClockSrc(CLK_PERIPHCLK_PLLQ);

  LL_PERIPH_WP(LL_PERIPH_PWC_CLK_RMU);

  LL_PERIPH_WE(LL_PERIPH_GPIO);
  GPIO_SetReadWaitCycle(u16GPIOWait << GPIO_PCCR_RDWT_POS);
  LL_PERIPH_WP(LL_PERIPH_GPIO);

  /***************************************************************************
   ** record configuration
   **************************************************************************/
  gstcSysClk.i32_AHBClk_Hz = i32SrcClockHz * (int32_t)u16Mpll_N / (int32_t)u16Mpll_P;
  gstcSysClk.i32_APB0Clk_Hz = gstcSysClk.i32_AHBClk_Hz;
  gstcSysClk.i32_APB1Clk_Hz = (gstcSysClk.i32_AHBClk_Hz >> 1);
  gstcSysClk.i32_APB3Clk_Hz = (gstcSysClk.i32_AHBClk_Hz >> 2);
  gstcSysClk.i32_ExClk_Hz = (gstcSysClk.i32_AHBClk_Hz >> 1);

  gstcSysClk.i32_APB2Clk_Hz = i32SrcClockHz * (int32_t)u16Mpll_N / (int32_t)u16Mpll_Q;
  gstcSysClk.i32_APB4Clk_Hz = gstcSysClk.i32_APB2Clk_Hz;
  return u32Err;

}



#endif


/**
 ******************************************************************************
 ** \brief initialize system timer for low priority motor control
 **
 ** \param [in] i32FreqHz: counting frequency of system ticker
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_SystemTicker(int32_t i32FreqHz)
{
    int32_t i32Temp;
    uint32_t u32ReloadValue;
    i32Temp = gstcSysClk.i32_AHBClk_Hz / i32FreqHz;
    u32ReloadValue = (uint32_t)i32Temp;
    SysTick->LOAD = u32ReloadValue - 1U; // set reload value
    SysTick->VAL = 0;                   // clear current counter value
    NVIC_SetPriority(SysTick_IRQn, 4);  // set priority
}



/**
 ******************************************************************************
 ** \brief start/stop system ticker
 **
 ** \param [in] bStart: TRUE-start, FALSE-stop
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_StartSystemTicker(boolean_t bStart)
{
    if (FALSE == bStart)
    {
        SysTick->CTRL = 0x4;    // disable counting, disable interrupt
    }
    else
    {
        SysTick->CTRL = 0x7;    // enable counting, enable interrupt, use processor clock
    }
}


/**
 ******************************************************************************
 ** \brief initialize software watch-dog timer. the Hardware watch-dog uses
 **        APB0 clock as clock source
 **
 ** \param [in] none
 **
 **
 ** \retval     none
 ******************************************************************************/
#if(USE_REG_DRIVER_MODE == USE_DRIVER_MODE)

void InitMcu_Wdg()
{
    /** initial SWDT */
    uint32_t u32WDTCR = 0x80010FF3u;

#if(WATCHDOG_REST == WATCHDOG_MODE)

    MODIFY_REG32(u32WDTCR, WDT_CR_ITS, 0x1u << WDT_CR_ITS_POS);
#elif(WATCHDOG_IRQ == WATCHDOG_MODE)
    MODIFY_REG32(u32WDTCR, WDT_CR_ITS, 0x0u << WDT_CR_ITS_POS);
#endif


    MODIFY_REG32(u32WDTCR, WDT_CR_WDPT, 0xFu << WDT_CR_WDPT_POS);

    /**  PCLK3/1024 */
    MODIFY_REG32(u32WDTCR, WDT_CR_CKS, 0xBu << WDT_CR_CKS_POS);

    /**  16384 cycle */
    MODIFY_REG32(u32WDTCR, WDT_CR_PERI, 0x2u << WDT_CR_PERI_POS);

    CM_WDT->CR = u32WDTCR;


    /** start SWDT */
    CM_WDT->RR = 0x0123;
    CM_WDT->RR = 0x3210;
}
#elif(USE_DDL_DRIVER_MODE == USE_DRIVER_MODE)
void InitMcu_Wdg()
{

    stc_wdt_init_t stcWdtInit;


#if(WATCHDOG_REST == WATCHDOG_MODE)
    stcWdtInit.u32ExceptionType = WDT_EXP_TYPE_RST;

#elif(WATCHDOG_IRQ == WATCHDOG_MODE)
    stcWdtInit.u32ExceptionType = WDT_EXP_TYPE_INT;
#endif

    stcWdtInit.u32CountPeriod   = WDT_CNT_PERIOD16384;
    stcWdtInit.u32ClockDiv      = WDT_CLK_DIV2048;
    stcWdtInit.u32RefreshRange  = WDT_RANGE_0TO100PCT;
    stcWdtInit.u32LPMCount      = WDT_LPM_CNT_CONTINUE;

    (void)WDT_Init(&stcWdtInit);

    /** start SWDT */
    WDT_FeedDog();
}
#endif

/**
 ******************************************************************************
 ** \brief feed hardware watch-dog
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_FeedWdg(void)
{
    CM_WDT->RR = 0x0123;
    CM_WDT->RR = 0x3210;
}


/**
 *******************************************************************************
 ** \brief initialized timer-4 for SVPWM generation
 **
 ** \param [in] u16_PeakCnt     peakcnt of timer4
 ** \param [in] u8_DeadTimeCnt     the time count of dead time
 ** \param [in] u8_ZeroIsrMaskCnt  mask interrupt count
 **
 ** \retval     none
 ******************************************************************************/
#if(USE_REG_DRIVER_MODE == USE_DRIVER_MODE)

void InitMcu_InitMotorTimer(uint16_t u16PeakCnt, uint16_t u16DeadTimeCnt, uint8_t u8ZeroIsrMaskCnt)
{
#if(1 == MTR1_TIM4_UNIT)
    MODIFY_REG32(CM_PWC->FCG2, PWC_FCG2_TMR4_1, 0x0U << PWC_FCG2_TMR4_1_POS);
#elif(2 == MTR1_TIM4_UNIT)
    MODIFY_REG32(CM_PWC->FCG2, PWC_FCG2_TMR4_2, 0x0U << PWC_FCG2_TMR4_2_POS);
#elif(3 == MTR1_TIM4_UNIT)
    MODIFY_REG32(CM_PWC->FCG2, PWC_FCG2_TMR4_3, 0x0U << PWC_FCG2_TMR4_3_POS);
#endif
    MODIFY_REG32(CM_PWC->FCG2, PWC_FCG2_EMB, 0x0U << PWC_FCG2_EMB_POS);

    /***************************************************************************
    * configure counter module
    ***************************************************************************/
    MTR1_bTIM4x->CCSR_b.STOP = 1;

    /** issue an initialization request to timer-4 ****************************/
    MTR1_bTIM4x->CCSR_b.CLEAR = 1;

    MTR1_bTIM4x->CCSR_b.IRQZEN = 1;      /** enable zero-match interrupt */
    MTR1_bTIM4x->CCSR_b.IRQZF = 0;       /** clear zero-match interrupt */

    MTR1_bTIM4x->CCSR_b.IRQPEN = 0;      /** disable peak interrupt */
    MTR1_bTIM4x->CCSR_b.IRQPF = 0;       /** clear peak interrupt */

    MTR1_bTIM4x->CCSR_b.BUFEN = 1;       /** enable CPSR buffer */
    MTR1_bTIM4x->CCSR_b.MODE = 1;        /** up-down counting mode */
    MTR1_TIM4x->CPSR = u16PeakCnt;        /** set peak value of counter */
    MTR1_TIM4x->CNTR = u16PeakCnt;

    /** set PCLK as counting clock, no division *******************************/
    MTR1_bTIM4x->CCSR_b.ECKEN = 0;
    MODIFY_REG16(MTR1_TIM4x->CCSR, TMR4_CCSR_CKDIV, 0x0U << TMR4_CCSR_CKDIV_POS);

    MODIFY_REG16(MTR1_TIM4x->CVPR, TMR4_CVPR_PIM, 0x0U << TMR4_CVPR_PIM_POS);   /** do NOT mask peak interrupt */
    MODIFY_REG16(MTR1_TIM4x->CVPR, TMR4_CVPR_ZIM, (uint16_t)u8ZeroIsrMaskCnt << TMR4_CVPR_ZIM_POS);   /** do NOT mask peak interrupt */

    /***************************************************************************
    * configure output compare module
    ***************************************************************************/
    /** disable compare operation for configuring output compare **************/
    MTR1_bTIM4x->OCSRU_b.OCEH = 0;
    MTR1_bTIM4x->OCSRU_b.OCEL = 0;
    MTR1_bTIM4x->OCSRV_b.OCEH = 0;
    MTR1_bTIM4x->OCSRV_b.OCEL = 0;
    MTR1_bTIM4x->OCSRW_b.OCEH = 0;
    MTR1_bTIM4x->OCSRW_b.OCEL = 0;

    /** set compare output level when disable comparison **********************/
    MTR1_bTIM4x->OCSRU_b.OCPH = 0;
    MTR1_bTIM4x->OCSRU_b.OCPL = 0;
    MTR1_bTIM4x->OCSRV_b.OCPH = 0;
    MTR1_bTIM4x->OCSRV_b.OCPL = 0;
    MTR1_bTIM4x->OCSRW_b.OCPH = 0;
    MTR1_bTIM4x->OCSRW_b.OCPL = 0;

    /** disable compare match interrupt ***************************************/
    MTR1_bTIM4x->OCSRU_b.OCIEH = 0;
    MTR1_bTIM4x->OCSRU_b.OCIEL = 0;
    MTR1_bTIM4x->OCSRV_b.OCIEH = 0;
    MTR1_bTIM4x->OCSRV_b.OCIEL = 0;
    MTR1_bTIM4x->OCSRW_b.OCIEH = 0;
    MTR1_bTIM4x->OCSRW_b.OCIEL = 0;

    /** clear compare match interrupt *****************************************/
    MTR1_bTIM4x->OCSRU_b.OCFH = 0;
    MTR1_bTIM4x->OCSRU_b.OCFL = 0;
    MTR1_bTIM4x->OCSRV_b.OCFH = 0;
    MTR1_bTIM4x->OCSRV_b.OCFL = 0;
    MTR1_bTIM4x->OCSRW_b.OCFH = 0;
    MTR1_bTIM4x->OCSRW_b.OCFL = 0;

    /** enable OCCR buffer function, and transfer at zero *********************/
    MODIFY_REG16(MTR1_TIM4x->OCERU, TMR4_OCER_CHBUFEN, 0x1U << TMR4_OCER_CHBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERU, TMR4_OCER_CLBUFEN, 0x1U << TMR4_OCER_CLBUFEN_POS);

    MODIFY_REG16(MTR1_TIM4x->OCERV, TMR4_OCER_CHBUFEN, 0x1U << TMR4_OCER_CHBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERV, TMR4_OCER_CLBUFEN, 0x1U << TMR4_OCER_CLBUFEN_POS);

    MODIFY_REG16(MTR1_TIM4x->OCERW, TMR4_OCER_CHBUFEN, 0x1U << TMR4_OCER_CHBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERW, TMR4_OCER_CLBUFEN, 0x1U << TMR4_OCER_CLBUFEN_POS);

    /** enable OCMR buffer function *******************************************/
    MODIFY_REG16(MTR1_TIM4x->OCERU, TMR4_OCER_MHBUFEN, 0x1U << TMR4_OCER_MHBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERU, TMR4_OCER_MLBUFEN, 0x1U << TMR4_OCER_MLBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERV, TMR4_OCER_MHBUFEN, 0x1U << TMR4_OCER_MHBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERV, TMR4_OCER_MLBUFEN, 0x1U << TMR4_OCER_MLBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERW, TMR4_OCER_MHBUFEN, 0x1U << TMR4_OCER_MHBUFEN_POS);
    MODIFY_REG16(MTR1_TIM4x->OCERW, TMR4_OCER_MLBUFEN, 0x1U << TMR4_OCER_MLBUFEN_POS);

    /** enable interrupt-mask linking *****************************************/
    MTR1_bTIM4x->OCERU_b.LMCH = 1;
    MTR1_bTIM4x->OCERU_b.LMCL = 1;
    MTR1_bTIM4x->OCERU_b.LMMH = 1;
    MTR1_bTIM4x->OCERU_b.LMML = 1;

    MTR1_bTIM4x->OCERV_b.LMCH = 1;
    MTR1_bTIM4x->OCERV_b.LMCL = 1;
    MTR1_bTIM4x->OCERV_b.LMMH = 1;
    MTR1_bTIM4x->OCERV_b.LMML = 1;

    MTR1_bTIM4x->OCERW_b.LMCH = 1;
    MTR1_bTIM4x->OCERW_b.LMCL = 1;
    MTR1_bTIM4x->OCERW_b.LMMH = 1;
    MTR1_bTIM4x->OCERW_b.LMML = 1;

    /** disable extended compare match ****************************************/
    MTR1_bTIM4x->OCERU_b.MCECH = 0;
    MTR1_bTIM4x->OCERU_b.MCECL = 0;

    MTR1_bTIM4x->OCERV_b.MCECH = 0;
    MTR1_bTIM4x->OCERV_b.MCECL = 0;

    MTR1_bTIM4x->OCERW_b.MCECH = 0;
    MTR1_bTIM4x->OCERW_b.MCECL = 0;

    /** set compare module operation conditions *******************************/
    MTR1_TIM4x->OCMRHUH = 0x0000;
    MTR1_TIM4x->OCMRLUL = 0x55A498A0;
    MTR1_TIM4x->OCMRHVH = 0x0000;
    MTR1_TIM4x->OCMRLVL = 0x55A498A0;
    MTR1_TIM4x->OCMRHWH = 0x0000;
    MTR1_TIM4x->OCMRLWL = 0x55A498A0;

    /** set initial compare value *********************************************/
    MTR1_TIM4x->OCCRUH = 0xFFFF;
    MTR1_TIM4x->OCCRUL = 0xFFFF;
    MTR1_TIM4x->OCCRVH = 0xFFFF;
    MTR1_TIM4x->OCCRVL = 0xFFFF;
    MTR1_TIM4x->OCCRWH = 0xFFFF;
    MTR1_TIM4x->OCCRWL = 0xFFFF;

    /** enable channel UL,VL,WL operation *************************************/
    MTR1_bTIM4x->OCSRU_b.OCEL = 1;
    MTR1_bTIM4x->OCSRV_b.OCEL = 1;
    MTR1_bTIM4x->OCSRW_b.OCEL = 1;

    /***************************************************************************
    * configure PWM generation module
    ***************************************************************************/
    // set as dead timer mode
    MODIFY_REG16(MTR1_TIM4x->POCRU, TMR4_POCR_PWMMD, 0x1U << TMR4_POCR_PWMMD_POS);
    MODIFY_REG16(MTR1_TIM4x->POCRV, TMR4_POCR_PWMMD, 0x1U << TMR4_POCR_PWMMD_POS);
    MODIFY_REG16(MTR1_TIM4x->POCRW, TMR4_POCR_PWMMD, 0x1U << TMR4_POCR_PWMMD_POS);


    /** set PCLK as dead-time counter clock ***********************************/
    MODIFY_REG16(MTR1_TIM4x->POCRU, TMR4_POCR_DIVCK, 0x0U << TMR4_POCR_DIVCK_POS);
    MODIFY_REG16(MTR1_TIM4x->POCRV, TMR4_POCR_DIVCK, 0x0U << TMR4_POCR_DIVCK_POS);
    MODIFY_REG16(MTR1_TIM4x->POCRW, TMR4_POCR_DIVCK, 0x0U << TMR4_POCR_DIVCK_POS);

    /***************************************************************************
     ** set active level of PWM
     **     00 - H_H, output PWM_0 and PWM_1 without changing level
     **     01 - L_L, output both PWM_0 and PWM_1 reversed
     **     10 - L_H, PWM_0 reversed, PWM_1 without changing level
     **     11 - H_L, PWM_0 without changing level, PWM_1 reversed
     **************************************************************************/
    MODIFY_REG16(MTR1_TIM4x->POCRU, TMR4_POCR_LVLS, ((uint8_t)(PWM_ACTIVE_LEVEL)) << TMR4_POCR_LVLS_POS);
    MODIFY_REG16(MTR1_TIM4x->POCRV, TMR4_POCR_LVLS, ((uint8_t)(PWM_ACTIVE_LEVEL)) << TMR4_POCR_LVLS_POS);
    MODIFY_REG16(MTR1_TIM4x->POCRW, TMR4_POCR_LVLS, ((uint8_t)(PWM_ACTIVE_LEVEL)) << TMR4_POCR_LVLS_POS);

    CM_GPIO->PWPR = 0xA501;     /** unlock write protection of PCRxy  */
    uint32_t *const u32ECER1 = ((uint32_t *)(0x40055408UL)); /** address of ECER1 */
   
#if(PWM_ACTIVE_LEVEL_HH == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(0);
    INIT_MTR1_UL(0);
    INIT_MTR1_VH(0);
    INIT_MTR1_VL(0);
    INIT_MTR1_WH(0);
    INIT_MTR1_WL(0);
    u32ECER1[MTR1_TIM4_UNIT - 1] = 0x02; /** 0x2: switch PWM to LOW when EMB event happens */
#elif(PWM_ACTIVE_LEVEL_LL == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(1);
    INIT_MTR1_UL(1);
    INIT_MTR1_VH(1);
    INIT_MTR1_VL(1);
    INIT_MTR1_WH(1);
    INIT_MTR1_WL(1);
    u32ECER1[MTR1_TIM4_UNIT - 1] = 0x3; /** 0x3: switch PWM to HIGH when EMB event happens */
#elif(PWM_ACTIVE_LEVEL_LH == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(1);
    INIT_MTR1_UL(0);
    INIT_MTR1_VH(1);
    INIT_MTR1_VL(0);
    INIT_MTR1_WH(1);
    INIT_MTR1_WL(0);
    u32ECER1[MTR1_TIM4_UNIT - 1] = 0x1; /** 0x1: switch PWM to HiZ when EMB event happens */
#elif(PWM_ACTIVE_LEVEL_HL == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(0);
    INIT_MTR1_UL(1);
    INIT_MTR1_VH(0);
    INIT_MTR1_VL(1);
    INIT_MTR1_WH(0);
    INIT_MTR1_WL(1);
    u32ECER1[MTR1_TIM4_UNIT - 1] = 0x1; /** 0x1: switch PWM to HiZ when EMB event happens */
#endif

    /** set dead time *********************************************************/
    MTR1_TIM4x->PDARU = u16DeadTimeCnt;
    MTR1_TIM4x->PDBRU = u16DeadTimeCnt;

    MTR1_TIM4x->PDARV = u16DeadTimeCnt;
    MTR1_TIM4x->PDBRV = u16DeadTimeCnt;

    MTR1_TIM4x->PDARW = u16DeadTimeCnt;
    MTR1_TIM4x->PDBRW = u16DeadTimeCnt;

    /** disable reload timer **************************************************/
    MTR1_bTIM4x->RCSR_b.RTEU = 0;       /** stop operation of reload timer */
    MTR1_bTIM4x->RCSR_b.RTEV = 0;
    MTR1_bTIM4x->RCSR_b.RTEW = 0;

    MTR1_bTIM4x->RCSR_b.RTIDU = 1;      /** disable sending interrupt flag to CPU */
    MTR1_bTIM4x->RCSR_b.RTIDV = 1;
    MTR1_bTIM4x->RCSR_b.RTIDW = 1;

    MTR1_bTIM4x->RCSR_b.RTICU = 1;      /** clear reload timer interrupt */
    MTR1_bTIM4x->RCSR_b.RTICV = 1;
    MTR1_bTIM4x->RCSR_b.RTICW = 1;

    /***************************************************************************
    * configure EMB
    *   1. effect level from VC to EMB is 1
    ***************************************************************************/
#if(HW_OCP_SRC_VC == HW_OCP_SRC)
    MODIFY_REG32(CM_PWC->FCG3, PWC_FCG3_CMP, 0x0U << PWC_FCG3_CMP_POS);         /** enable CMP clock                      */
#if(1 == HW_OCP_CMP_UNIT)
    bCM_CMPCR->DACR_b.DA1EN = 1;
    CM_CMPCR->DADR1 = (HW_OCP_CMP_DA_DATA & 0xFF);
#define HW_OCP_CMP  CM_CMP1
#define HW_OCP_bCMP  bCM_CMP1
#elif(2 == HW_OCP_CMP_UNIT)
    bCM_CMPCR->DACR_b.DA2EN = 1;
    CM_CMPCR->DADR2 = (HW_OCP_CMP_DA_DATA & 0xFF);
#define HW_OCP_CMP  CM_CMP2
#define HW_OCP_bCMP  bCM_CMP2
#elif(3 == HW_OCP_CMP_UNIT)
    bCM_CMPCR->DACR_b.DA1EN = 1;
    CM_CMPCR->DADR1 = (HW_OCP_CMP_DA_DATA & 0xFF);
#define HW_OCP_CMP  CM_CMP3
#define HW_OCP_bCMP  bCM_CMP3
#endif
    for (i32DelayCnt = 0; i32DelayCnt < 100; i32DelayCnt++)
    { /** >300ns @200MHz */
        
    }

    INIT_HW_OCP_CMP_INP;
    MODIFY_REG16(HW_OCP_CMP->VLTSEL, CMP_VLTSEL_CVSL, (1 << (HW_OCP_CMP_INP_CH - 1)) << CMP_VLTSEL_CVSL_POS);
    MODIFY_REG16(HW_OCP_CMP->VLTSEL, CMP_VLTSEL_RVSL, 0x4 << CMP_VLTSEL_RVSL_POS);      /** INM3 from DA */

    HW_OCP_bCMP->CTRL_b.INV = 0x0;       /** 0x0: output with no reversal */
    HW_OCP_bCMP->CTRL_b.OUTEN = 0x1;     /** 0x1: output enable */
    MODIFY_REG16(HW_OCP_CMP->CTRL, CMP_CTRL_FLTSL, 0x0 << CMP_CTRL_FLTSL_POS);  /** 0x0: no digital filtering */
    HW_OCP_bCMP->CTRL_b.IEN = 0;         /** 0x0: disable interrupt */
    HW_OCP_bCMP->CTRL_b.CMPON = 1;       /** 0x1: start CMP */
    for (i32DelayCnt = 0; i32DelayCnt < 100; i32DelayCnt++)
    { /** >300ns @200MHz */
        __NOP();
    }

    HW_OCP_bCMP->CTRL_b.CMPOE = 1;       /** 0x1: VCOUT enable */
    MTR1_EMBx->CTL = (1 << HW_OCP_CMP_UNIT);
    MTR1_bEMBx->INTEN_b.CMPINTEN = 1;  /** enable EMB interrupt from CMP */
#else
    INIT_HW_OCP_EMB_IO;
#if(0 == HW_OCP_EMB_LEVEL)
    MTR1_EMBx->CTL1 = ((0x0FUL << 22) + (0x0FUL << 16));
    MTR1_EMBx->CTL2 = ((1UL << 27) + (3UL << 25));
#else
    MTR1_EMBx->CTL1 = ((1UL << 30) + (3UL << 28) + (1UL));
    MTR1_EMBx->CTL2 = ((1UL << 30) + (3UL << 28) + (1UL));
#endif

    MTR1_bEMBx->INTEN_b.PORTININTEN1 = 1;   /** enable EMB interrupt from EMB_IN */
    MTR1_bEMBx->INTEN_b.PORTININTEN2 = 1;   /** enable EMB interrupt from EMB_IN */
    MTR1_bEMBx->INTEN_b.PORTININTEN3 = 1;   /** enable EMB interrupt from EMB_IN */
    MTR1_bEMBx->INTEN_b.PORTININTEN4 = 1;   /** enable EMB interrupt from EMB_IN */
#endif

    CM_GPIO->PWPR = 0x0000;        /** lock write protection of PCRxy  */
}
#elif(USE_DDL_DRIVER_MODE == USE_DRIVER_MODE)
void InitMcu_InitMotorTimer(uint16_t u16PeakCnt, uint16_t u16DeadTimeCnt, uint8_t u8ZeroIsrMaskCnt)
{
    stc_tmr4_init_t stcTmr4Init;
    stc_tmr4_oc_init_t stcTmr4OcInit;
    un_tmr4_oc_ocmrh_t unTmr4OcOcmrh;
    un_tmr4_oc_ocmrl_t unTmr4OcOcmrl;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    stc_emb_tmr4_init_t stcEmbInit;

#if(HW_OCP_SRC_VC == HW_OCP_SRC)
    stc_cmp_init_t stcCmpInit;
#endif


    int32_t i32DelayCnt;
#if(1 == MTR1_TIM4_UNIT)
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_1, ENABLE);
#elif(2 == MTR1_TIM4_UNIT)
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_2, ENABLE);
#elif(3 == MTR1_TIM4_UNIT)
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);
#endif
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_EMB, ENABLE);
    /***************************************************************************
    * configure counter module
    ***************************************************************************/



    /** set PCLK as counting clock, no division *******************************/
    stcTmr4Init.u16ClockDiv = TMR4_CLK_DIV1;
    stcTmr4Init.u16PeriodValue = u16PeakCnt;
    stcTmr4Init.u16ClockSrc = TMR4_CLK_SRC_INTERNCLK;
    stcTmr4Init.u16CountMode = TMR4_MD_TRIANGLE;
    (void)TMR4_Init(MTR1_TIM4x, &stcTmr4Init);


    /** issue an initialization request to timer-4 ****************************/
    TMR4_IntCmd(MTR1_TIM4x, TMR4_INT_CNT_VALLEY, ENABLE);      /** enable zero-match interrupt */
    TMR4_ClearStatus(MTR1_TIM4x, TMR4_FLAG_CNT_VALLEY);        /** clear zero-match interrupt */

    TMR4_IntCmd(MTR1_TIM4x, TMR4_INT_CNT_PEAK, DISABLE);       /** disable peak interrupt */
    TMR4_ClearStatus(MTR1_TIM4x, TMR4_FLAG_CNT_PEAK);          /** clear peak interrupt */

    TMR4_PeriodBufCmd(MTR1_TIM4x, ENABLE);                      /** enable CPSR buffer */
    TMR4_SetCountValue(MTR1_TIM4x, u16PeakCnt);                 /** set count initial value*/

    TMR4_SetCountIntMaskTime(MTR1_TIM4x, TMR4_INT_CNT_PEAK, 0); /** do NOT mask peak interrupt */
    TMR4_SetCountIntMaskTime(MTR1_TIM4x, TMR4_INT_CNT_VALLEY, u8ZeroIsrMaskCnt);    /** do  mask zero interrupt */

    /***************************************************************************
    * configure output compare module
    ***************************************************************************/
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;              // OCEx = 0/OCPx= 0/OCIEx = 0/OCFx = 0
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;     /** enable OCCR buffer function, and transfer at zero *********************/
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_VALLEY;      /** enable OCMR buffer function *******************************************/
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE | TMR4_OC_BUF_CMP_MD;   /** enable interrupt-mask linking : LMCx = 1 LMMx = 1******/
    stcTmr4OcInit.u16CompareValue = 0xFFFF;                             /** set initial compare value *********************************************/
    (void)TMR4_OC_Init(MTR1_TIM4x, TMR4_OC_CH_UH, &stcTmr4OcInit);
    (void)TMR4_OC_Init(MTR1_TIM4x, TMR4_OC_CH_UL, &stcTmr4OcInit);
    (void)TMR4_OC_Init(MTR1_TIM4x, TMR4_OC_CH_VH, &stcTmr4OcInit);
    (void)TMR4_OC_Init(MTR1_TIM4x, TMR4_OC_CH_VL, &stcTmr4OcInit);
    (void)TMR4_OC_Init(MTR1_TIM4x, TMR4_OC_CH_WH, &stcTmr4OcInit);
    (void)TMR4_OC_Init(MTR1_TIM4x, TMR4_OC_CH_WL, &stcTmr4OcInit);

    TMR4_OC_ExtendControlCmd(MTR1_TIM4x, TMR4_OC_CH_UH, DISABLE);       /** disable extended compare match :MCEC= 0**************************/
    TMR4_OC_ExtendControlCmd(MTR1_TIM4x, TMR4_OC_CH_UL, DISABLE);
    TMR4_OC_ExtendControlCmd(MTR1_TIM4x, TMR4_OC_CH_VH, DISABLE);
    TMR4_OC_ExtendControlCmd(MTR1_TIM4x, TMR4_OC_CH_VL, DISABLE);
    TMR4_OC_ExtendControlCmd(MTR1_TIM4x, TMR4_OC_CH_WH, DISABLE);
    TMR4_OC_ExtendControlCmd(MTR1_TIM4x, TMR4_OC_CH_WL, DISABLE);

    /** set compare module operation conditions *******************************/
    unTmr4OcOcmrh.OCMRx = 0x0000;
    TMR4_OC_SetHighChCompareMode(MTR1_TIM4x, TMR4_OC_CH_UH, unTmr4OcOcmrh);
    TMR4_OC_SetHighChCompareMode(MTR1_TIM4x, TMR4_OC_CH_VH, unTmr4OcOcmrh);
    TMR4_OC_SetHighChCompareMode(MTR1_TIM4x, TMR4_OC_CH_WH, unTmr4OcOcmrh);

    unTmr4OcOcmrl.OCMRx = 0x55A498A0;
    TMR4_OC_SetLowChCompareMode(MTR1_TIM4x, TMR4_OC_CH_UL, unTmr4OcOcmrl);
    TMR4_OC_SetLowChCompareMode(MTR1_TIM4x, TMR4_OC_CH_VL, unTmr4OcOcmrl);
    TMR4_OC_SetLowChCompareMode(MTR1_TIM4x, TMR4_OC_CH_WL, unTmr4OcOcmrl);

    /** enable channel UL,VL,WL operation *************************************/
    TMR4_OC_Cmd(MTR1_TIM4x, TMR4_OC_CH_UL, ENABLE);
    TMR4_OC_Cmd(MTR1_TIM4x, TMR4_OC_CH_VL, ENABLE);
    TMR4_OC_Cmd(MTR1_TIM4x, TMR4_OC_CH_WL, ENABLE);

    /***************************************************************************
    * configure PWM generation module
    ***************************************************************************/
    stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;                  // set as dead timer mode
    stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;                 /** set PCLK as dead-time counter clock ***********************************/
    /***************************************************************************
     ** set active level of PWM
     **     00 - H_H, output PWM_0 and PWM_1 without changing level
     **     01 - L_L, output both PWM_0 and PWM_1 reversed
     **     10 - L_H, PWM_0 reversed, PWM_1 without changing level
     **     11 - H_L, PWM_0 without changing level, PWM_1 reversed
     **************************************************************************/
    stcTmr4PwmInit.u16Polarity = ((uint16_t)(PWM_ACTIVE_LEVEL)) << TMR4_POCR_LVLS_POS;
    (void)TMR4_PWM_Init(MTR1_TIM4x, TMR4_PWM_CH_U, &stcTmr4PwmInit);
    (void)TMR4_PWM_Init(MTR1_TIM4x, TMR4_PWM_CH_V, &stcTmr4PwmInit);
    (void)TMR4_PWM_Init(MTR1_TIM4x, TMR4_PWM_CH_W, &stcTmr4PwmInit);

    LL_PERIPH_WE(LL_PERIPH_GPIO);   /** unlock write protection of PCRxy  */
#if(PWM_ACTIVE_LEVEL_HH == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(0);
    INIT_MTR1_UL(0);
    INIT_MTR1_VH(0);
    INIT_MTR1_VL(0);
    INIT_MTR1_WH(0);
    INIT_MTR1_WL(0);
    TMR4_PWM_SetAbnormalPinStatus(MTR1_TIM4x, TMR4_PWM_PIN_OUH, TMR4_PWM_ABNORMAL_PIN_LOW);     /** 0x2: switch PWM to LOW when EMB event happens */
#elif(PWM_ACTIVE_LEVEL_LL == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(1);
    INIT_MTR1_UL(1);
    INIT_MTR1_VH(1);
    INIT_MTR1_VL(1);
    INIT_MTR1_WH(1);
    INIT_MTR1_WL(1);
    TMR4_PWM_SetAbnormalPinStatus(MTR1_TIM4x, TMR4_PWM_PIN_OUH, TMR4_PWM_ABNORMAL_PIN_HIGH);    /** 0x3: switch PWM to HIGH when EMB event happens */
#elif(PWM_ACTIVE_LEVEL_LH == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(1);
    INIT_MTR1_UL(0);
    INIT_MTR1_VH(1);
    INIT_MTR1_VL(0);
    INIT_MTR1_WH(1);
    INIT_MTR1_WL(0);
    TMR4_PWM_SetAbnormalPinStatus(MTR1_TIM4x, TMR4_PWM_PIN_OUH, TMR4_PWM_ABNORMAL_PIN_HIZ);   /** 0x1: switch PWM to HiZ when EMB event happens */
#elif(PWM_ACTIVE_LEVEL_HL == PWM_ACTIVE_LEVEL)
    INIT_MTR1_UH(0);
    INIT_MTR1_UL(1);
    INIT_MTR1_VH(0);
    INIT_MTR1_VL(1);
    INIT_MTR1_WH(0);
    INIT_MTR1_WL(1);
    TMR4_PWM_SetAbnormalPinStatus(MTR1_TIM4x, TMR4_PWM_PIN_OUH, TMR4_PWM_ABNORMAL_PIN_HIZ);   /** 0x1: switch PWM to HiZ when EMB event happens */
#endif

    /** set dead time *********************************************************/
    TMR4_PWM_SetDeadTimeValue(MTR1_TIM4x, TMR4_PWM_CH_U, TMR4_PWM_PDAR_IDX, u16DeadTimeCnt);
    TMR4_PWM_SetDeadTimeValue(MTR1_TIM4x, TMR4_PWM_CH_U, TMR4_PWM_PDBR_IDX, u16DeadTimeCnt);

    TMR4_PWM_SetDeadTimeValue(MTR1_TIM4x, TMR4_PWM_CH_V, TMR4_PWM_PDAR_IDX, u16DeadTimeCnt);
    TMR4_PWM_SetDeadTimeValue(MTR1_TIM4x, TMR4_PWM_CH_V, TMR4_PWM_PDBR_IDX, u16DeadTimeCnt);

    TMR4_PWM_SetDeadTimeValue(MTR1_TIM4x, TMR4_PWM_CH_W, TMR4_PWM_PDAR_IDX, u16DeadTimeCnt);
    TMR4_PWM_SetDeadTimeValue(MTR1_TIM4x, TMR4_PWM_CH_W, TMR4_PWM_PDBR_IDX, u16DeadTimeCnt);

    /** disable reload timer **************************************************/
    /** stop operation of reload timer */
    TMR4_PWM_StopReloadTimer(MTR1_TIM4x, TMR4_PWM_CH_U);
    TMR4_PWM_StopReloadTimer(MTR1_TIM4x, TMR4_PWM_CH_V);
    TMR4_PWM_StopReloadTimer(MTR1_TIM4x, TMR4_PWM_CH_W);

    /** disable sending interrupt flag to CPU */
    TMR4_IntCmd(MTR1_TIM4x, TMR4_INT_RELOAD_TMR_U, DISABLE);
    TMR4_IntCmd(MTR1_TIM4x, TMR4_INT_RELOAD_TMR_V, DISABLE);
    TMR4_IntCmd(MTR1_TIM4x, TMR4_INT_RELOAD_TMR_W, DISABLE);

    /** clear reload timer interrupt */
    TMR4_ClearStatus(MTR1_TIM4x, TMR4_FLAG_RELOAD_TMR_U);
    TMR4_ClearStatus(MTR1_TIM4x, TMR4_FLAG_RELOAD_TMR_V);
    TMR4_ClearStatus(MTR1_TIM4x, TMR4_FLAG_RELOAD_TMR_W);

    /***************************************************************************
    * configure EMB
    *   1. effect level from VC to EMB is 1
    ***************************************************************************/
#if(HW_OCP_SRC_VC == HW_OCP_SRC)
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_CMP, ENABLE);                            /** enable CMP clock                      */
#if(1 == HW_OCP_CMP_UNIT)
    CMP_8BitDAC_WriteData(CMP_8BITDAC_CH1, HW_OCP_CMP_DA_DATA);
    CMP_8BitDAC_Cmd(CMP_8BITDAC_CH1, ENABLE);
#define HW_OCP_CMP  CM_CMP1
#define HW_OCP_bCMP  bCM_CMP1
#elif(2 == HW_OCP_CMP_UNIT)
    CMP_8BitDAC_WriteData(CMP_8BITDAC_CH2, HW_OCP_CMP_DA_DATA);
    CMP_8BitDAC_Cmd(CMP_8BITDAC_CH2, ENABLE);
#define HW_OCP_CMP  CM_CMP2
#define HW_OCP_bCMP  bCM_CMP2
#elif(3 == HW_OCP_CMP_UNIT)
    CMP_8BitDAC_WriteData(CMP_8BITDAC_CH1, HW_OCP_CMP_DA_DATA);
    CMP_8BitDAC_Cmd(CMP_8BITDAC_CH1, ENABLE);
#define HW_OCP_CMP  CM_CMP3
#define HW_OCP_bCMP  bCM_CMP3
#endif
    for (i32DelayCnt = 0; i32DelayCnt < 100; i32DelayCnt++)
    { /** >300ns @200MHz */
        __NOP();
    }

    INIT_HW_OCP_CMP_INP;

    stcCmpInit.u16PositiveInput = (1 << (HW_OCP_CMP_INP_CH - 1)) << CMP_VLTSEL_CVSL_POS;
    stcCmpInit.u16NegativeInput = 0x4 << CMP_VLTSEL_RVSL_POS;
    stcCmpInit.u16OutPolarity = CMP_OUT_INVT_OFF;
    stcCmpInit.u16OutDetectEdge = CMP_DETECT_EDGS_NONE;
    stcCmpInit.u16OutFilter = CMP_OUT_FILTER_NONE;
    (void)CMP_NormalModeInit(HW_OCP_CMP, &stcCmpInit);

    CMP_IntCmd(HW_OCP_CMP, DISABLE);            /** 0x0: disable interrupt */

    CMP_CompareOutCmd(HW_OCP_CMP, ENABLE);      /** 0x1: VCOUT enable */

    CMP_PinVcoutCmd(HW_OCP_CMP, ENABLE);        /** 0x1: output enable */

    (void)EMB_TMR4_StructInit(&stcEmbInit);

#if(1 == HW_OCP_CMP_UNIT)
    stcEmbInit.stcCmp.u32Cmp1State = EMB_CMP1_ENABLE; 
#elif(2 == HW_OCP_CMP_UNIT)
    stcEmbInit.stcCmp.u32Cmp2State = EMB_CMP2_ENABLE;
#elif(3 == HW_OCP_CMP_UNIT)
    stcEmbInit.stcCmp.u32Cmp3State = EMB_CMP3_ENABLE;
#endif
    (void)EMB_TMR4_Init(MTR1_EMBx, &stcEmbInit);

    EMB_IntCmd(MTR1_EMBx, EMB_INT_CMP, ENABLE);         /** enable EMB interrupt from CMP */

#else
    INIT_HW_OCP_EMB_IO;
    (void)EMB_TMR4_StructInit(&stcEmbInit);
    stcEmbInit.stcPort.stcPort1.u32PortState = EMB_PORT1_ENABLE;
    stcEmbInit.stcPort.stcPort1.u32PortFilterDiv = EMB_PORT1_FILTER_CLK_DIV128;
    stcEmbInit.stcPort.stcPort1.u32PortFilterState = EMB_PORT1_FILTER_ENABLE;
#if(0 == HW_OCP_EMB_LEVEL)
    stcEmbInit.stcPort.stcPort1.u32PortLevel = EMB_PORT1_DETECT_LVL_LOW;
#else
    stcEmbInit.stcPort.stcPort1.u32PortLevel = EMB_PORT1_DETECT_LVL_HIGH;

#endif
    (void)EMB_TMR4_Init(MTR1_EMBx, &stcEmbInit);
    /* EMB: enable interrupt */
    EMB_IntCmd(MTR1_EMBx, EMB_INT_PORT1, ENABLE);           /** enable EMB interrupt from EMB_IN */
#endif

    LL_PERIPH_WP(LL_PERIPH_GPIO);        /** lock write protection of PCRxy  */
}


#endif

/**
 ******************************************************************************
 ** \brief set GPIO work as input of ADC
 ** \param [in] uint8_t u8AniCH: ADC channel
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_SetGPO2ANI(uint8_t u8AniCh)
{
    CM_GPIO->PWPR = 0xA501;    /** unlock write protection of PCRxy        */
    switch(u8AniCh)
    {
        case 0U:  {bCM_GPIO->PCRA0_b.DDIS = 1; break;}    /** PA0  */
        case 1U:  {bCM_GPIO->PCRA1_b.DDIS = 1; break;}    /** PA1  */
        case 2U:  {bCM_GPIO->PCRA2_b.DDIS = 1; break;}    /** PA2  */
        case 3U:  {bCM_GPIO->PCRA3_b.DDIS = 1; break;}    /** PA3  */
        case 4U:  {bCM_GPIO->PCRA4_b.DDIS = 1; break;}    /** PA4  */
        case 5U:  {bCM_GPIO->PCRA5_b.DDIS = 1; break;}    /** PA5  */
        case 6U:  {bCM_GPIO->PCRA6_b.DDIS = 1; break;}    /** PA6  */
        case 7U:  {bCM_GPIO->PCRA7_b.DDIS = 1; break;}    /** PA7  */
        case 8U:  {bCM_GPIO->PCRB0_b.DDIS = 1; break;}    /** PB0  */
        case 9U:  {bCM_GPIO->PCRB1_b.DDIS = 1; break;}    /** PB1  */
        case 10U: {bCM_GPIO->PCRC0_b.DDIS = 1; break;}    /** PC0  */
        case 11U: {bCM_GPIO->PCRC1_b.DDIS = 1; break;}    /** PC1  */
        case 12U: {bCM_GPIO->PCRC2_b.DDIS = 1; break;}    /** PC2  */
        case 13U: {bCM_GPIO->PCRC3_b.DDIS = 1; break;}    /** PC3  */
        case 14U: {bCM_GPIO->PCRC4_b.DDIS = 1; break;}    /** PC4  */
        case 15U: {bCM_GPIO->PCRC5_b.DDIS = 1; break;}    /** PC5  */
        default: {break;}
    }
    CM_GPIO->PWPR = 0x0000;    /** lock write protection of PCRxy          */
}



/**
 ******************************************************************************
 ** \brief initialize ADC for signal sampling
 **     1. two-SHUNT
 **        a. sequence B for current sampling, by TRG0 from SCCRUH
 **        b. sequence A for other sampling(VDC, VSP, etc.), by TRG1 from SCCRUL
 **     2. single-SHUNT
 **        a. sequence B for current sampling
 **           - TRG0 from SCCRVH, 1st trigger
 **           - TRG1 from SCCRVL, 2nd trigger
 **        b. sequence A for other sampling(VDC, VSP, etc.), by software start
 **
 ** \param [in] uint16_t u16AdcTrigCnt: ADC scan conversion trigger count
 **
 ** \retval     none
 ******************************************************************************/
#if(USE_REG_DRIVER_MODE == USE_DRIVER_MODE)
void InitMcu_Adc(uint16_t u16AdcTrigCnt)
{
    uint8_t u8SSTR = 25;        /** 25: ~417ns @60MHz */
    uint8_t *const u8SSTRx = ((uint8_t *) 0x40040020UL);                  /** address of ADC_SSTR0 */

    CM_PWC->FCG0PC = 0xA5A50001UL;                                                /** unlock FCG0                           */
    MODIFY_REG32(CM_PWC->FCG0, PWC_FCG0_AOS, 0x0U << PWC_FCG0_AOS_POS);         /** enable AOS clock                      */
    CM_PWC->FCG0PC = 0xA5A50000UL;                                                /** lock FCG0                             */
    MODIFY_REG32(CM_PWC->FCG3, PWC_FCG3_ADC1, 0x0U << PWC_FCG3_ADC1_POS);       /** enable ADC clock                      */

    CM_ADC1->STR = 0;                                                           /** 0x0: stop ADC conversion              */
    MODIFY_REG16(CM_ADC1->CR0, ADC_CR0_ACCSEL, 0x0U << ADC_CR0_ACCSEL_POS);     /** 0x0: 12-bit resolution                */

    bCM_ADC1->CR0_b.DFMT = 0;        /** 0x0: data format as right align       */
    bCM_ADC1->CR0_b.CLREN = 1;       /** 0x1: clear data after CPU/DMA reading */

    CM_ADC1->CHSELRA = 0;          /** 0x0: clear register                   */
    CM_ADC1->CHSELRB = 0;          /** 0x0: clear register                   */
    MODIFY_REG16(CM_ADC1->CR0, ADC_CR0_MS, 0x2U << ADC_CR0_MS_POS);     /** 0x2: sequence A&B, single conversion  */

    /***************************************************************************
     * always use sequence A for VDC,  VSP, etc. sampling
     **************************************************************************/
    InitMcu_SetGPO2ANI(u8_ADC_CH_VDC);
    CM_ADC1->CHSELRA = (1UL << (u8_ADC_CH_VDC));
    u8SSTRx[u8_ADC_CH_VDC] = u8SSTR;
#if(TRUE == EN_VSP_CMD)
    InitMcu_SetGPO2ANI(u8_ADC_CH_VSP);
    CM_ADC1->CHSELRA |= (1UL << (u8_ADC_CH_VSP));
    u8SSTRx[u8_ADC_CH_VSP] = u8SSTR;
#endif

    /***************************************************************************
     * sequence B only for current sampling, both single- and two-SHUNT
     **************************************************************************/
    InitMcu_SetGPO2ANI(u8_ADC_CH_IU);

    CM_ADC1->CHSELRB = (1UL << (u8_ADC_CH_IU));
    u8SSTRx[u8_ADC_CH_IU] = u8SSTR;

    InitMcu_SetGPO2ANI(u8_ADC_CH_IV);
    CM_ADC1->CHSELRB |= (1UL << (u8_ADC_CH_IV));
    u8SSTRx[u8_ADC_CH_IV] = u8SSTR;

    /***************************************************************************
     * initialize TRG0(SCCRUH)
     **************************************************************************/
    MTR1_bTIM4x->SCSRUH_b.LMC = 1;     /** 0x1: link buffer with interrupt mask  */
    MODIFY_REG16(MTR1_TIM4x->SCSRUH, TMR4_SCSR_BUFEN, 0x1U << TMR4_SCSR_BUFEN_POS);     /** 0x1: enable SCCR buffer               */
    MODIFY_REG16(MTR1_TIM4x->SCSRUH, TMR4_SCSR_EVTOS, 0x0U << TMR4_SCSR_EVTOS_POS);     /** 0x0: enable special event 0           */
    MTR1_bTIM4x->SCSRUH_b.EVTMS = 0;   /** 0x0: work in compare mode             */
    MTR1_bTIM4x->SCSRUH_b.ZEN = 1;     /** 1b: enable zero compare               */
    MTR1_bTIM4x->SCSRUH_b.UEN = 0;     /** 0b: disable up-count mode compare     */
    MTR1_bTIM4x->SCSRUH_b.PEN = 0;     /** 0b: disable peak compare              */
    MTR1_bTIM4x->SCSRUH_b.DEN = 1;     /** 1b: enable down-count mode compare    */
    MODIFY_REG16(MTR1_TIM4x->SCMRUH, TMR4_SCMR_AMC, 0x0U << TMR4_SCMR_AMC_POS);     /** start ADC when ZIC = 0                */
    MTR1_bTIM4x->SCMRUH_b.MPCE = 0;    /** 0x0: disable AMC compare with PIC     */
    MTR1_bTIM4x->SCMRUH_b.MZCE = 1;    /** 0x0: disable AMC compare with ZIC     */

    /***************************************************************************
     * initialize TRG1(SCCRUL)
     **************************************************************************/
    MTR1_bTIM4x->SCSRUL_b.LMC = 1;     /** 0x1: link buffer with interrupt mask  */
    MODIFY_REG16(MTR1_TIM4x->SCSRUL, TMR4_SCSR_BUFEN, 0x1U << TMR4_SCSR_BUFEN_POS);     /** 0x1: enable SCCR buffer               */
    MODIFY_REG16(MTR1_TIM4x->SCSRUL, TMR4_SCSR_EVTOS, 0x1U << TMR4_SCSR_EVTOS_POS);     /** 0x1: enable special event 0           */
    MTR1_bTIM4x->SCSRUL_b.EVTMS = 0;   /** 0x0: work in compare mode             */
    MTR1_bTIM4x->SCSRUL_b.ZEN = 1;     /** 1b: enable zero compare               */
    MTR1_bTIM4x->SCSRUL_b.UEN = 0;     /** 0b: disable up-count mode compare     */
    MTR1_bTIM4x->SCSRUL_b.PEN = 0;     /** 0b: disable peak compare              */
    MTR1_bTIM4x->SCSRUL_b.DEN = 1;     /** 1b: enable down-count mode compare    */
    MODIFY_REG16(MTR1_TIM4x->SCMRUL, TMR4_SCMR_AMC, 0x0U << TMR4_SCMR_AMC_POS);     /** start ADC when ZIC = 0                */
    MTR1_bTIM4x->SCMRUL_b.MPCE = 0;    /** 0x0: disable AMC compare with PIC     */
    MTR1_bTIM4x->SCMRUL_b.MZCE = 1;    /** 0x0: disable AMC compare with ZIC     */

    /** initialize trigger source of TRG0(SCCRUH) and TRG1(SCMUL) *************/
#if(1 == MTR1_TIM4_UNIT)
    CM_AOS->ADC1_TRGSEL0 = (uint32_t)EVT_SRC_TMR4_1_GCMP_UH;   /** SCMUH as source of IN_TGR0 */
    CM_AOS->ADC1_TRGSEL1 = (uint32_t)EVT_SRC_TMR4_1_GCMP_UL;   /** SCMUL as source of IN_TGR1 */
#elif(2 == MTR1_TIM4_UNIT)
    CM_AOS->ADC1_TRGSEL0 = (uint32_t)EVT_SRC_TMR4_2_GCMP_UH;   /** SCMUH as source of IN_TGR0 */
    CM_AOS->ADC1_TRGSEL1 = (uint32_t)EVT_SRC_TMR4_2_GCMP_UL;   /** SCMUL as source of IN_TGR1 */
#elif(3 == MTR1_TIM4_UNIT)
    CM_AOS->ADC1_TRGSEL0 = (uint32_t)EVT_SRC_TMR4_3_GCMP_UH;   /** SCMUH as source of IN_TGR0 */
    CM_AOS->ADC1_TRGSEL1 = (uint32_t)EVT_SRC_TMR4_3_GCMP_UL;   /** SCMUL as source of IN_TGR1 */
#endif
    /** initialize TRG0(SCCRUH) for sequence B ********************************/
    MTR1_TIM4x->SCCRUH = u16AdcTrigCnt;       /** TRG0(SCCRUH) for sequence B */
    bCM_ADC1->TRGSR_b.TRGENB = 1;    /** 0x1: enable trigger for sequence B    */
    MODIFY_REG16(CM_ADC1->TRGSR, ADC_TRGSR_TRGSELB, 0x1U << ADC_TRGSR_TRGSELB_POS);         /** 0x1: use IN_TRG0 as trigger           */

    /** initialize TRG1(SCCRUL) for sequence A ********************************/
    MTR1_TIM4x->SCCRUL = 0;
    bCM_ADC1->TRGSR_b.TRGENA = 1;    /** 0x1: use internal source as trigger */
    MODIFY_REG16(CM_ADC1->TRGSR, ADC_TRGSR_TRGSELA, 0x2U << ADC_TRGSR_TRGSELA_POS);         /** 0x1: use IN_TRG1 as trigger         */
    CM_ADC1->ICR = 0x0;             /** 0x0: disable interrupt              */
}
#elif(USE_DDL_DRIVER_MODE == USE_DRIVER_MODE)
void InitMcu_Adc(uint16_t u16AdcTrigCnt)
{

    stc_tmr4_evt_init_t stcTmr4EventInit;
    stc_adc_init_t stcAdcInit;
    uint8_t u8SSTR = 25;        /** 25: ~417ns @60MHz */
    

    CM_PWC->FCG0PC = 0xA5A50001U;                                                /** unlock FCG0                           */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);                            /** enable AOS clock                      */
    CM_PWC->FCG0PC = 0xA5A50000U;                                                /** lock FCG0                             */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);                           /** enable ADC clock                      */

    ADC_DeInit(CM_ADC1);

    ADC_Stop(CM_ADC1);                                                          /** 0x0: stop ADC conversion              */

    ADC_DataRegAutoClearCmd(CM_ADC1, ENABLE);                                   /** 0x1: clear data after CPU/DMA reading */

    stcAdcInit.u16DataAlign = ADC_DATAALIGN_RIGHT;                              /** 0x0: data format as right align       */
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;                            /** 0x0: 12-bit resolution                */
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SEQB_SINGLESHOT;                       /** 0x2: sequence A&B, single conversion  */
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    /***************************************************************************
     * always use sequence A for VDC,  VSP, etc. sampling
     **************************************************************************/
    InitMcu_SetGPO2ANI(u8_ADC_CH_VDC);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH_VDC, ENABLE);
    ADC_SetSampleTime(CM_ADC1, ADC_CH_VDC, u8SSTR);
#if(TRUE == EN_VSP_CMD)
    InitMcu_SetGPO2ANI(u8_ADC_CH_VSP);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH_VSP, ENABLE);
    ADC_SetSampleTime(CM_ADC1, ADC_CH_VSP, u8SSTR);
#endif

    /***************************************************************************
     * sequence B only for current sampling, both single- and two-SHUNT
     **************************************************************************/
    InitMcu_SetGPO2ANI(u8_ADC_CH_IU);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_B, ADC_CH_IU, ENABLE);
    ADC_SetSampleTime(CM_ADC1, ADC_CH_IU, u8SSTR);

    InitMcu_SetGPO2ANI(u8_ADC_CH_IV);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_B, ADC_CH_IV, ENABLE);
    ADC_SetSampleTime(CM_ADC1, ADC_CH_IV, u8SSTR);

    /***************************************************************************
     * initialize TRG0(SCCRUH)
     **************************************************************************/
    stcTmr4EventInit.u16Mode = TMR4_EVT_MD_CMP;                                 /** 0x0: work in compare mode             */
    stcTmr4EventInit.u16CompareValue = u16AdcTrigCnt;                           /** TRG0(SCCRUH) for sequence B */
    stcTmr4EventInit.u16OutputEvent = TMR4_EVT_OUTPUT_EVT0;                      /** 0x0: enable special event 0  */
    stcTmr4EventInit.u16MatchCond = TMR4_EVT_MATCH_CNT_DOWN | TMR4_EVT_MATCH_CNT_VALLEY;    // enable zero compare and down-count mode compare
    (void)TMR4_EVT_Init(MTR1_TIM4x, TMR4_EVT_CH_UH, &stcTmr4EventInit);

    TMR4_EVT_BufIntervalResponseCmd(MTR1_TIM4x, TMR4_EVT_CH_UH, ENABLE);         /** 0x1: link buffer with interrupt mask  */
    TMR4_EVT_SetCompareBufCond(MTR1_TIM4x, TMR4_EVT_CH_UH, TMR4_EVT_BUF_COND_VALLEY);   /** 0x1: enable SCCR buffer */

    TMR4_EVT_SetMaskTime(MTR1_TIM4x, TMR4_EVT_CH_UH, 0);                        /** start ADC when ZIC = 0                */
    TMR4_EVT_EventIntervalResponseCmd(MTR1_TIM4x, TMR4_EVT_CH_UH, TMR4_EVT_MASK_VALLEY, ENABLE);     /** 0x1: enable AMC compare with ZIC     */  
    TMR4_EVT_EventIntervalResponseCmd(MTR1_TIM4x, TMR4_EVT_CH_UH, TMR4_EVT_MASK_PEAK, DISABLE);      /** 0x0: disable AMC compare with PIC     */

    /***************************************************************************
     * initialize TRG1(SCCRUL)
     **************************************************************************/
    stcTmr4EventInit.u16CompareValue = 0;
    stcTmr4EventInit.u16OutputEvent = TMR4_EVT_OUTPUT_EVT1;                     /** 0x1: enable special event 1 */
    (void)TMR4_EVT_Init(MTR1_TIM4x, TMR4_EVT_CH_UL, &stcTmr4EventInit);

    TMR4_EVT_BufIntervalResponseCmd(MTR1_TIM4x, TMR4_EVT_CH_UL, ENABLE);         /** 0x1: link buffer with interrupt mask  */
    TMR4_EVT_SetCompareBufCond(MTR1_TIM4x, TMR4_EVT_CH_UL, TMR4_EVT_BUF_COND_VALLEY);   /** 0x1: enable SCCR buffer */
    TMR4_EVT_SetMaskTime(MTR1_TIM4x, TMR4_EVT_CH_UL, 0);                        /** start ADC when ZIC = 0           */
    TMR4_EVT_EventIntervalResponseCmd(MTR1_TIM4x, TMR4_EVT_CH_UL, TMR4_EVT_MASK_VALLEY, ENABLE);     /** 0x1: enable AMC compare with ZIC     */
    TMR4_EVT_EventIntervalResponseCmd(MTR1_TIM4x, TMR4_EVT_CH_UL, TMR4_EVT_MASK_PEAK, DISABLE);      /** 0x0: disable AMC compare with PIC     */

    /** initialize trigger source of TRG0(SCCRUH) and TRG1(SCMUL) *************/
#if(1 == MTR1_TIM4_UNIT)
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_TMR4_1_GCMP_UH);             /** SCMUH as source of IN_TGR0 */
    AOS_SetTriggerEventSrc(AOS_ADC1_1, EVT_SRC_TMR4_1_GCMP_UL);             /** SCMUL as source of IN_TGR1 */
#elif(2 == MTR1_TIM4_UNIT)
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_TMR4_2_GCMP_UH);             /** SCMUH as source of IN_TGR0 */
    AOS_SetTriggerEventSrc(AOS_ADC1_1, EVT_SRC_TMR4_2_GCMP_UL);             /** SCMUL as source of IN_TGR1 */
#elif(3 == MTR1_TIM4_UNIT)
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_TMR4_3_GCMP_UH);             /** SCMUH as source of IN_TGR0 */
    AOS_SetTriggerEventSrc(AOS_ADC1_1, EVT_SRC_TMR4_3_GCMP_UL);             /** SCMUL as source of IN_TGR1 */
#endif
    /** initialize TRG0(SCCRUH) for sequence B ********************************/
    ADC_TriggerCmd(CM_ADC1, ADC_SEQ_B, ENABLE);                                 /** 0x1: enable trigger for sequence B    */
    ADC_TriggerConfig(CM_ADC1, ADC_SEQ_B, ADC_HARDTRIG_EVT0);                               /** 0x1: use IN_TRG0 as trigger */

    /** initialize TRG1(SCCRUL) for sequence A ********************************/
    ADC_TriggerCmd(CM_ADC1, ADC_SEQ_A, ENABLE);                                 /** 0x1: use internal source as trigger */
    ADC_TriggerConfig(CM_ADC1, ADC_SEQ_A, ADC_HARDTRIG_EVT1);                    /** 0x2: use IN_TRG1 as trigger */

    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA | ADC_INT_EOCB, DISABLE);                  /** 0x0: disable interrupt              */
}

#endif


/**
 ******************************************************************************
 ** \brief start/stop timer-4 operation
 **
 ** \param [in] bEnable: TRUE:start timer-4, FALSE: stop timer-4
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_StartMotorTimer(boolean_t bEnable)
{
    if (TRUE == bEnable)
    {
        MTR1_bTIM4x->CCSR_b.STOP = 0;      // start timer 4
    }
    else
    {
        MTR1_bTIM4x->CCSR_b.STOP = 1;      // stop timer 4
    }
}


/**
 ******************************************************************************
 ** \brief set Zero-Match-Interrupt mask count
 **
 ** \param [in] u8ZIM: mask count of Zero-Match-Interrupt
 **
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_SetZeroIsrMaskCnt(uint8_t u8ZIM)
{
    MODIFY_REG16(MTR1_TIM4x->CVPR, TMR4_CVPR_ZIM, ((uint16_t)u8ZIM << TMR4_CVPR_ZIM_POS));
}

/**
 ******************************************************************************
 ** \brief read Zero-Match-Interrupt mask count
 **
 ** \param [in] none
 **
 ** \retval     uint8_t ZIM value
 ******************************************************************************/
uint8_t InitMcu_ReadZeroIsrMaskCnt(void)
{
    uint16_t u16ZIM = (MTR1_TIM4x->CVPR & 0x0FU);
    uint8_t u8ZIM = (uint8_t)u16ZIM;
    return u8ZIM;
}


/**
 ******************************************************************************
 ** \brief read ATVR0
 **
 ** \param [in] none
 **
 ** \retval     uint16_t ATVR0 value
 ******************************************************************************/
uint16_t InitMcu_ReadAtvr0(void)
{
    uint16_t u16Atvr0 = MTR1_TIM4x->SCCRUH;
    return u16Atvr0;
}

/**
 ******************************************************************************
 ** \brief set ATVR0
 **
 ** \param [in] uint16_t ATVR0 value
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_SetAtvr0(uint16_t u16ATVR0)
{
    MTR1_TIM4x->SCCRUH = u16ATVR0;
}

/**
 ******************************************************************************
 ** \brief set ATVR1
 **
 ** \param [in] uint16_t ATVR1 value
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_SetAtvr1(uint16_t u16ATVR1)
{
    MTR1_TIM4x->SCCRVH = u16ATVR1;
}

/**
 ******************************************************************************
 ** \brief set ATVR2
 **
 ** \param [in] uint16_t ATVR2 value
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_SetAtvr2(uint16_t u16ATVR2)
{
    MTR1_TIM4x->SCCRVL = u16ATVR2;
}

/**
 ******************************************************************************
 ** \brief update period register of motor PWM timer
 **
 ** \param [in] u16_Cpsr: new CPSR register value
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_UpdateMotorPwmCycle(uint16_t u16_Cpsr)
{
    MTR1_TIM4x->CPSR = u16_Cpsr;
}

/**
 ******************************************************************************
 ** \brief read period register of motor PWM timer
 **
 ** \param [in] none
 **
 ** \retval     CPSR value
 ******************************************************************************/
uint16_t InitMcu_ReadMotorPwmCycle(void)
{
    uint16_t u16_Cpsr = MTR1_TIM4x->CPSR;
    return u16_Cpsr;
}

/**
 ******************************************************************************
 ** \brief write compare register of motor PWM timer
 **
 ** \param [in] compare value of PWM
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_UpdateMotorPwmDuty(uint16_t u16_Occr0, uint16_t u16_Occr1, uint16_t u16_Occr2,
                                uint16_t u16_Occr3, uint16_t u16_Occr4, uint16_t u16_Occr5)
{
    MTR1_TIM4x->OCCRUH = u16_Occr0;
    MTR1_TIM4x->OCCRUL = u16_Occr1;
    MTR1_TIM4x->OCCRVH = u16_Occr2;
    MTR1_TIM4x->OCCRVL = u16_Occr3;
    MTR1_TIM4x->OCCRWH = u16_Occr4;
    MTR1_TIM4x->OCCRWL = u16_Occr5;
}

/**
 ******************************************************************************
 ** \brief enable/disable timer-4 output PWM
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_EnMotorPwm(boolean_t bEnable)
{
    CM_GPIO->PWPR = 0xA501;        /** unlock write protection of PFSR    */
    if (TRUE == bEnable)
    {
        SET_MTR1_UH_PWM;
        SET_MTR1_UL_PWM;
        SET_MTR1_VH_PWM;
        SET_MTR1_VL_PWM;
        SET_MTR1_WH_PWM;
        SET_MTR1_WL_PWM;
    }
    else
    {
        SET_MTR1_UH_GPO;
        SET_MTR1_UL_GPO;
        SET_MTR1_VH_GPO;
        SET_MTR1_VL_GPO;
        SET_MTR1_WH_GPO;
        SET_MTR1_WL_GPO;
    }
    CM_GPIO->PWPR = 0x0000;        /** lock write protection of PFSR      */
}

/**
 ******************************************************************************
 ** \brief enable/disable each PWM output of timer-4
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_EnEachMotorPwm(boolean_t bEnPwm0, boolean_t bEnPwm1, boolean_t bEnPwm2,
                            boolean_t bEnPwm3, boolean_t bEnPwm4, boolean_t bEnPwm5)
{
    CM_GPIO->PWPR = 0xA501;        /** unlock write protection of PFSR    */

    if(0 == bEnPwm0) {SET_MTR1_UH_GPO;} else {SET_MTR1_UH_PWM;}
    if(0 == bEnPwm1) {SET_MTR1_UL_GPO;} else {SET_MTR1_UL_PWM;}

    if(0 == bEnPwm2) {SET_MTR1_VH_GPO;} else {SET_MTR1_VH_PWM;}
    if(0 == bEnPwm3) {SET_MTR1_VL_GPO;} else {SET_MTR1_VL_PWM;}

    if(0 == bEnPwm4) {SET_MTR1_WH_GPO;} else {SET_MTR1_WH_PWM;}
    if(0 == bEnPwm5) {SET_MTR1_WL_GPO;} else {SET_MTR1_WL_PWM;}

    CM_GPIO->PWPR = 0x0000;        /** lock write protection of PFSR      */
}

/**
 ******************************************************************************
 ** \brief initialized GPIOs
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void InitMcu_Gpio(void)
{
#if(TRUE == EN_TEST_IO)
    INIT_TEST_GPIO_OUT(0);
#endif
}



