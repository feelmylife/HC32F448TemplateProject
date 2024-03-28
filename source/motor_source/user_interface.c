/**
 *******************************************************************************
 * @file  user_interface.c
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

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_source/user_interface.h"
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
int32_t     ui_i32CarrierFreqHz		= 16000;		/** (Hz) carrier frequency of PWM */
int32_t     ui_i32SysTickerFreqHz	= 5000;			/** (Hz) frequency of system ticker */
float32_t   ui_f32DeadTimeUs		= 2.0f;			/** (us) dead time of inverter */
float32_t   ui_f32MaxDutyRatio 		= 0.75f;		/** PWM maximum output duty ratio*/

float32_t   ui_f32AdcTrigTimeUs     = 0.5f;     // (us) ADC trigger time before zero-match of timer-4
/*******************************************************************************
 ** ADC sampling
 ******************************************************************************/
float32_t   ui_f32IuvwSampleRs      = 0.047f;    // (Ohm) sample resistance
float32_t   ui_f32IuvwSampleK       = 5.0f;    // current sample gain
float32_t   ui_i32AdcCheckDelayMs   = 50.0f;    // (ms) ADC offset check delay time after power on
float32_t   ui_f32OffsetCheckTimeMs = 50.0;     // (ms) ADC offset check time
int32_t     ui_i32IuvwRefOffset     = 2048;     // ADC reference offset
int32_t     ui_i32IuvwMaxOffsetBias = 80;       // maximum offset bias

float32_t   ui_f32VbusSampleK       = 10.36f;   // DC bus voltage sample gain
/*******************************************************************************
 ** motor parameters
 ******************************************************************************/
float32_t   ui_f32BaseVoltage       = 24.0f;       /** (V) base voltage */
float32_t   ui_f32BaseCurrent       = 4.0f;         /** (A) base current */
float32_t   ui_f32BaseFrequency     = 500.0f;        /** (Hz) base frequency */

int32_t     ui_i32PolePairs         = 7;
float32_t   ui_f32Rs                = 0.11f;        /** Ohm */
float32_t   ui_f32Ld                = 0.005f;       /** mH */
float32_t   ui_f32Lq                = 0.005f;       /** mH */
float32_t   ui_f32Ke                = 0.34f;       /** V/kRPM */

int32_t     ui_i32MinSpeedRpm       = 800;       // (RPM) minimum speed
int32_t     ui_i32MaxSpeedRpm       = 5000;      // (RPM) maximum speed
float32_t   ui_f32MaxNormaIs        = 2.0f;     // (A) maximum normal operation current
float32_t   ui_f32MaxStartIs        = 1.5f;     // (A) maximum start current(zero/low speed)
float32_t   ui_f32MaxBrakeIs        = 0.8f;     // (A) maximum braking current


/** acceleration and deceleration rate ****************************************/

float32_t   ui_f32AccHzPerSec       = 20.0f;  		/** acceleration rate(elec. Hz/sec) */
float32_t   ui_f32DecHzPerSec       = 10.0f;  		/** deceleration rate(elec. Hz/sec) */



/*******************************************************************************
 * protections
 ******************************************************************************/
/** DC bus voltage protection *************************************************/
float32_t   ui_f32AbnormalHighVbus  = 28.0f;   // V
float32_t   ui_f32AbnormalLowVbus   = 6.0f;   // V
int32_t     ui_i32AbnormalVbusTimeUs= 500;      // us

float32_t   ui_f32OverVoltageThold  = 26.0f;   // V
int32_t     ui_i32OverVoltageTimeUs = 1000;     // us

float32_t   ui_f32UnderVoltageThold = 8.0f;   // V
int32_t     ui_i32UnderVoltageTimeUs= 1000;     // us

/** over current protection ***************************************************/
float32_t   ui_f32PeakOverCurrThold = 5.0f;     // A, threshold of peak over current
int32_t     ui_i32PeakOverCurrTimeUs= 500;      // us, peak OC response time
int32_t     ui_i32ErrClearTimeMs    = 5000;     // ms




/*******************************************************************************
 ** Observer Parameters
 ******************************************************************************/
float32_t   ui_f32FluxObsGamma      = 0.02f;     // observer learn rate
float32_t   ui_f32FluxObsPLLKp      = 0.55f;    // PLL Kp
float32_t   ui_f32FluxObsPLLKi      = 0.35f;   // PLL Ki


/*******************************************************************************
 ** regulator parameters
 ******************************************************************************/
/** control period settings ***************************************************/
float32_t   ui_f32SpdRegPeriodMs    = 2.0f;         /** (ms) speed regulation period */

/** startup_control parameters ************************************************/
float32_t   ui_f32ObsStableTimeMs   = 100.0f;   // (ms) observer stable time
float32_t   ui_f32MinObserableWr    = 10.5f;     // (elec. Hz) minimum observable speed


/*******************************************************************************
 ** stand-still start-up parameters, 4 modes available
 **     1. align       ==> force-drive ==> close-loop   :
 ******************************************************************************/
float32_t   ui_f32AlignCurrent      = 1.0f;    // (A) align current
float32_t   ui_f32_1stAlignTimeMs   = 50.0f;     // (ms) first align time, increasing current from 0 to align-current
float32_t   ui_f32_2ndAlignTimeMs   = 50.0f;     // (ms) second align time,
float32_t   ui_f32_3rdAlignTimeMs   = 500.0f;     // (ms) third align time, hold rotor at final align position
float32_t   ui_f32_2ndAlignFwdTheta = 0.0f;     // (deg) forward angle in second align

float32_t   ui_f32ForceCurrentSlop  = 10.0f;    // (A/s) current slop when apply force drive(with ui_f32MaxStartIs)
float32_t   ui_f32ForceAccRate      = 50.0f;     // (elec. Hz/s) acceleration rate when force-drive
float32_t   ui_f32MaxForceSpd       = 10.0f;    // (elec. Hz/s) maximum force drive speed

/*******************************************************************************
 ** head-wind start-up parameters
 ******************************************************************************/
float32_t   ui_f32HeadWindStartCurrent      = 0.5f;     		// (A) open-loop current
float32_t   ui_f32HeadWindMaxForceSpd       = 15.0f;     		// (elec. Hz/s) maximum force drive speed
float32_t   ui_f32HeadWindForceAccRate      = 5.0f;     		// (elec. Hz/s) open-loop acceleration rate

float32_t   ui_f32MinShortBrakeTimeMs   = 100.0f;   		// (ms) minimum short brake time
float32_t   ui_f32MaxShortBrakeTimeMs   = 25000.0f; 		// (ms) maximum short brake time
float32_t   ui_f32ShortBrakeEndIs       = 0.040f;   		// (A)  current to finish short braking



/** speed PI regulator ********************************************************/
float32_t   ui_f32SpdKp         = 0.025;
float32_t   ui_f32SpdKi         = 0.055;
float32_t   ui_f32MaxSpdErr     = 20.0f;        // (elec. Hz)

/** d-axis current PI regulator ***********************************************/
float32_t   ui_f32IdKp          = 0.1f;
float32_t   ui_f32IdKi          = 4.0f;
float32_t   ui_f32MaxIdErr      = 1.0f;        // A

/** q-axis current PI regulator ***********************************************/
float32_t   ui_f32IqKp          = 0.1f;
float32_t   ui_f32IqKi          = 4.0f;
float32_t   ui_f32MaxIqErr      = 1.0f;        // A

/** braking parameters ********************************************************/


/*******************************************************************************
 ** miscellaneous control parameters
 ******************************************************************************/
int32_t     ui_i32VdLpfFc       = 100;          //(Hz) cut-off frequency of VdRef LPF
int32_t     ui_i32VqLpfFc       = 100;          //(Hz) cut-off frequency of VqRef LPF

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/
