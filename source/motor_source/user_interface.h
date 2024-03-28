/**
 *******************************************************************************
 * @file  user_interface.h
 * @brief This file contains user interface parameter function.
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
#ifndef MOTOR_SOURCE_USER_INTERFACE_H_
#define MOTOR_SOURCE_USER_INTERFACE_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/module_include.h"

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
#define     EN_FIELD_WEAKEN     TRUE            /** field weakening control */
#define     EN_BRAKE_CTRL       TRUE            /** electric brake function */
#define     EN_DEAD_TIME_COMP   FALSE           /** dead time compensation */

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/*******************************************************************************
 ** basic drive information
 ******************************************************************************/
extern int32_t     ui_i32CarrierFreqHz;         /** (Hz) carrier frequency of PWM */
extern int32_t     ui_i32SysTickerFreqHz;       /** (Hz) frequency of system ticker */
extern float32_t   ui_f32DeadTimeUs;            /** (us) dead time of inverter */
extern float32_t   ui_f32MaxDutyRatio;          /** PWM maximum output duty ratio*/
extern float32_t   ui_f32AdcTrigTimeUs;     // (us) ADC trigger time before zero-match of timer-4
/*******************************************************************************
 ** ADC sampling
 ******************************************************************************/
extern float32_t   ui_f32IuvwSampleRs;
extern float32_t   ui_f32IuvwSampleK;
extern float32_t   ui_i32AdcCheckDelayMs;
extern float32_t   ui_f32OffsetCheckTimeMs;
extern int32_t     ui_i32IuvwRefOffset;
extern int32_t     ui_i32IuvwMaxOffsetBias;
extern float32_t   ui_f32VbusSampleK;

/*******************************************************************************
 ** motor parameters
 ******************************************************************************/
extern float32_t   ui_f32BaseVoltage;           /** (V) base voltage */
extern float32_t   ui_f32BaseCurrent;           /** (A) base current */
extern float32_t   ui_f32BaseFrequency;         /** (Hz) base frequency */

extern int32_t     ui_i32PolePairs;             /** Motor's pole pairs */
extern float32_t   ui_f32Rs;                    /** Ohm */
extern float32_t   ui_f32Ld;                    /** mH */
extern float32_t   ui_f32Lq;                    /** mH */
extern float32_t   ui_f32Ke;                    /** V/kRPM */

extern int32_t     ui_i32MinSpeedRpm;           /**  (RPM) minimum speed */
extern int32_t     ui_i32MaxSpeedRpm;           /**  (RPM) maximum speed */
extern float32_t   ui_f32MaxNormaIs;
extern float32_t   ui_f32MaxStartIs;
extern float32_t   ui_f32MaxBrakeIs;

/** acceleration and deceleration rate ****************************************/
extern float32_t   ui_f32AccHzPerSec;           /** acceleration rate(elec. Hz/sec) */
extern float32_t   ui_f32DecHzPerSec;           /** deceleration rate(elec. Hz/sec) */

/*******************************************************************************
 * protections
 ******************************************************************************/
extern float32_t   ui_f32AbnormalHighVbus;
extern float32_t   ui_f32AbnormalLowVbus;
extern int32_t     ui_i32AbnormalVbusTimeUs;

extern float32_t   ui_f32OverVoltageThold;
extern int32_t     ui_i32OverVoltageTimeUs;

extern float32_t   ui_f32UnderVoltageThold;
extern int32_t     ui_i32UnderVoltageTimeUs;

extern float32_t   ui_f32PeakOverCurrThold;
extern int32_t     ui_i32PeakOverCurrTimeUs;
extern int32_t     ui_i32ErrClearTimeMs;


/*******************************************************************************
 ** Observer Parameters
 ******************************************************************************/
extern float32_t   ui_f32FluxObsGamma;          /** observer learn rate */
extern float32_t   ui_f32FluxObsPLLKp;          /** PLL Kp */
extern float32_t   ui_f32FluxObsPLLKi;          /** PLL Ki */


/*******************************************************************************
 ** protections
 ******************************************************************************/


/*******************************************************************************
 ** regulator parameters
 ******************************************************************************/
/** control period settings ***************************************************/
extern float32_t   ui_f32SpdRegPeriodMs;        /** (ms) speed regulation period */

/** startup_control parameters ************************************************/
extern float32_t   ui_f32ObsStableTimeMs;       /** (ms) observer stable time */
extern float32_t   ui_f32MinObserableWr;        /** (elec. Hz) minimum observable speed */

/*******************************************************************************
 ** stand-still start-up parameters, 4 modes available
 **     1. align       ==> force-drive ==> close-loop   :
 ******************************************************************************/
extern float32_t   ui_f32AlignCurrent;          /** (A) align current */
extern float32_t   ui_f32_1stAlignTimeMs;       /** (ms) first align time, increasing current from 0 to align-current */
extern float32_t   ui_f32_2ndAlignTimeMs;       /** (ms) second align time */
extern float32_t   ui_f32_3rdAlignTimeMs;       /** (ms) third align time, hold rotor at final align position */
extern float32_t   ui_f32_2ndAlignFwdTheta;     /** (deg) forward angle in second align */

extern float32_t   ui_f32ForceCurrentSlop;      /** (A/s) current slop when apply force drive(with ui_f32MaxStartIs) */
extern float32_t   ui_f32ForceAccRate;          /** (elec. Hz/s) acceleration rate when force-drive */
extern float32_t   ui_f32MaxForceSpd;           /** (elec. Hz/s) maximum force drive speed */

/*******************************************************************************
 ** head-wind start-up parameters
 **          0              | short brake ==> open-loop   ==> close-loop
 ******************************************************************************/
extern float32_t   ui_f32HeadWindStartCurrent;  /** (A) open-loop current */
extern float32_t   ui_f32HeadWindMaxForceSpd;   /** (elec. Hz/s) maximum force drive speed */
extern float32_t   ui_f32HeadWindForceAccRate;  /** (elec. Hz/s) open-loop acceleration rate */

extern float32_t   ui_f32MinShortBrakeTimeMs;   /** (ms) minimum short brake time */
extern float32_t   ui_f32MaxShortBrakeTimeMs;   /** (ms) maximum short brake time */
extern float32_t   ui_f32ShortBrakeEndIs;       /** (A)  current to finish short braking */

/** speed PI regulator ********************************************************/
extern float32_t   ui_f32SpdKp;
extern float32_t   ui_f32SpdKi;
extern float32_t   ui_f32MaxSpdErr;

/** d-axis current PI regulator ***********************************************/
extern float32_t   ui_f32IdKp;
extern float32_t   ui_f32IdKi;
extern float32_t   ui_f32MaxIdErr;

/** q-axis current PI regulator ***********************************************/
extern float32_t   ui_f32IqKp;
extern float32_t   ui_f32IqKi;
extern float32_t   ui_f32MaxIqErr;

/** braking parameters ********************************************************/


/*******************************************************************************
 ** miscellaneous control parameters
 ******************************************************************************/
extern int32_t     ui_i32VdLpfFc;
extern int32_t     ui_i32VqLpfFc;
#endif /* MOTOR_SOURCE_USER_INTERFACE_H_ */
