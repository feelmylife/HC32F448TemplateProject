/**
 *******************************************************************************
 * @file  init_mcu.h
 * @brief This file provides peripheral function initialization.
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

#ifndef MCU_CONFIG_INIT_MCU_H_
#define MCU_CONFIG_INIT_MCU_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_common/mcu_include.h"
#include "motor_midwares/module_include.h"
#include "motor_source/motor_control.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/
/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/
typedef struct {
    int32_t i32_AHBClk_Hz;          // max@200MHz, CPU, DMA, EFM, SRAM, MPU, GPIO, DCU, INTC, QSPI
    int32_t i32_APB0Clk_Hz;         // max@200MHz, TMR6(counter)
    int32_t i32_APB1Clk_Hz;         // max@100MHz, USARTn, USBFS(control), TMR6(control),TMR0/TMRA/TMR4/EMB/CRC/HASH/AES/I2S
    int32_t i32_APB2Clk_Hz;         // max@60MHz, ADC(conversion)
    int32_t i32_APB3Clk_Hz;         // max@50MHz, RTC, CMP, WDT, SWDT control
    int32_t i32_APB4Clk_Hz;         // max@100MHz, ADC(control), TRNG, __PGA
    int32_t i32_ExClk_Hz;           // max@100MHz, SDIOn, CAN
    int32_t i32_UClk_Hz;            // 48MHz, USBFS(communication)
    int32_t i32_CANClk_Hz;          // 4-24MHz, CAN(communication)
} stc_SysClk_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
extern stc_SysClk_t   gstcSysClk;

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/


/**
 ******************************************************************************
 ** \brief initialize clocks of MCU
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
#if(USE_REG_DRIVER_MODE == USE_DRIVER_MODE)
//extern void SystemInit(void);
#endif
/**
 ******************************************************************************
 ** \brief initialize NVIC
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_Nvic(void);

/**
 ******************************************************************************
 ** \brief initialize clocks of MCU, use PLL as main clock
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern uint32_t InitMcu_Clock(void);

/**
 ******************************************************************************
 ** \brief initialize system timer for low priority motor control
 **
 ** \param [in] i32FreqHz: counting frequency of system ticker
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_SystemTicker(int32_t i32FreqHz);

/**
 ******************************************************************************
 ** \brief start/stop system ticker
 **
 ** \param [in] bStart: TRUE-start, FALSE-stop
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_StartSystemTicker(boolean_t bStart);

/**
 ******************************************************************************
 ** \brief initialize software watch-dog timer. the Hardware watch-dog uses
 **        APB0 clock as clock source
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_Wdg(void);

/**
 ******************************************************************************
 ** \brief feed software watch-dog
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_FeedWdg(void);

/**
 ******************************************************************************
 ** \brief initialized timer-4 for SVPWM generation
 **
 ** \param [in] u16_CPSR: CPSR register value (peak value in up-down count mode)
 ** \param [in] u16_DeadTimeCnt: count value of dead-time
 ** \param [in] u8ZeroIsrMaskCnt: zero interrupt mask count of TIMER 4
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_InitMotorTimer(uint16_t u16PeakCnt, uint16_t u16DeadTimeCnt, uint8_t u8ZeroIsrMaskCnt);



/**
 ******************************************************************************
 ** \brief set GPIO work as input of ADC
 ** \param [in] uint8_t u8AniCH: ADC channel
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_SetGPO2ANI(uint8_t u8AniCh);

/**
 ******************************************************************************
 ** \brief initialized ADC for signal sampling
 **
 ** \param [in] uint16_t u16AdcTrigCnt: ADC scan conversion trigger count
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_Adc(uint16_t u16AdcTrigCnt);

/**
 ******************************************************************************
 ** \brief start/stop timer-4 operation
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_StartMotorTimer(boolean_t bEnable);

/**
 ******************************************************************************
 ** \brief set Zero-Match-Interrupt mask count
 **
 ** \param [in] u8ZIM: mask count of Zero-Match-Interrupt
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_SetZeroIsrMaskCnt(uint8_t u8ZIM);

/**
 ******************************************************************************
 ** \brief read Zero-Match-Interrupt mask count
 **
 ** \param [in] none
 **
 ** \retval     uint8_t ZIM value
 ******************************************************************************/
extern uint8_t InitMcu_ReadZeroIsrMaskCnt(void);

/**
 ******************************************************************************
 ** \brief read ATVR0
 **
 ** \param [in] none
 **
 ** \retval     uint16_t ATVR0 value
 ******************************************************************************/
extern uint16_t InitMcu_ReadAtvr0(void);

/**
 ******************************************************************************
 ** \brief set ATVR0
 **
 ** \param [in] uint16_t ATVR0 value
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_SetAtvr0(uint16_t u16ATVR0);

/**
 ******************************************************************************
 ** \brief set ATVR1
 **
 ** \param [in] uint16_t ATVR1 value
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_SetAtvr1(uint16_t u16ATVR1);

/**
 ******************************************************************************
 ** \brief set ATVR2
 **
 ** \param [in] uint16_t ATVR2 value
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_SetAtvr2(uint16_t u16ATVR2);

/**
 ******************************************************************************
 ** \brief update period register of motor PWM timer
 **
 ** \param [in] u16_Tccp: new TCCP register value
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_UpdateMotorPwmCycle(uint16_t u16_Cpsr);

/**
 ******************************************************************************
 ** \brief read period register of motor PWM timer
 **
 ** \param [in] none
 **
 ** \retval     CPSR value
 ******************************************************************************/
extern uint16_t InitMcu_ReadMotorPwmCycle(void);

/**
 ******************************************************************************
 ** \brief write compare register of motor PWM timer
 **
 ** \param [in] compare value of PWM
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_UpdateMotorPwmDuty(uint16_t u16_Occr0, uint16_t u16_Occr1, uint16_t u16_Occr2,
                                       uint16_t u16_Occr3, uint16_t u16_Occr4, uint16_t u16_Occr5);

/**
 ******************************************************************************
 ** \brief enable/disable timer-4 output PWM
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_EnMotorPwm(boolean_t bEnable);

/**
 ******************************************************************************
 ** \brief enable/disable each PWM output of timer-4
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_EnEachMotorPwm(boolean_t bEnPwm0, boolean_t bEnPwm1, boolean_t bEnPwm2,
                                   boolean_t bEnPwm3, boolean_t bEnPwm4, boolean_t bEnPwm5);

/**
 ******************************************************************************
 ** \brief initialized GPIOs
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void InitMcu_Gpio(void);


#endif /* MCU_CONFIG_INIT_MCU_H_ */
