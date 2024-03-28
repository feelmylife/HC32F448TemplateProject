/**
 *******************************************************************************
 * @file  motor_control.h
 * @brief This file contains FOC contol function.
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
#ifndef MOTOR_SOURCE_MOTOR_CONTROL_H_
#define MOTOR_SOURCE_MOTOR_CONTROL_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "mcu_common/mcu_include.h"
#include "motor_midwares/module_include.h"
#include "motor_midwares/adc_sample.h"
#include "motor_midwares/startup_control.h"
#include "motor_midwares/stop_control.h"
#include "motor_midwares/speed_set.h"
#include "motor_midwares/protection.h"
#include "motor_source/user_interface.h"
#include "motor_source/cmd_set_analog_io.h"


/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/
/** p.u. format motor parameters */
typedef struct {
    int32_t   i32PolePairs;     /** pole pairs of motor */
    float32_t f32Rs;            /** (p.u.) phase stator resistance */
    float32_t f32Ld;            /** (p.u.) d-axis inductance */
    float32_t f32Lq;            /** (p.u.) q-axis inductance */
    float32_t f32LambdaF;       /** (p.u.) PM flux linkage constant */
} stc_motor_para_t;


/** motor running parameters */
typedef struct {
    int32_t i32Q12_CmdWr;               /** command speed, positive only, p.u. */
    int32_t i32Q12_TgtWr;               /** target speed,  positive only, p.u. */
    int32_t i32Q12_RealTimeWr;          /** real-time speed, both positive and negative, p.u. */
    int32_t i32CmdWrDir;                /** direction of command speed */
    int32_t i32TgtWrDir;                /** direction of target speed */
    volatile uint32_t   u32FaultCode;   /** fault code */
    volatile uint32_t   u32MotorStatus; /** control status */
    volatile boolean_t  bSysRdyFlag;    /** control system ready flag */
    volatile boolean_t  bEnSpdTraj;     /** enable speed trajectory set */
    volatile boolean_t  bEnSpdPid;      /** enable speed regulation */
    volatile boolean_t  bEnCurrentPid;  /** enable d-q current regulation */
    volatile boolean_t  bEnBrake;       /** enable brake control */
    int32_t i32Q12_Vbus;                /** DC bus voltage */
    int32_t i32Q12_AvgVbus;             /** average DC bus voltage */
    int32_t i32Q24_DrvAngle;            /** Motor drive angle */
    int32_t i32Fs;                      /** sample frequency */

} stc_motor_run_para_t;

/** current and voltage limitation parameters *********************************/
typedef struct {
    int32_t i32Q12_MaxIs;               // maximum stator current of motor
    int32_t i32Q12_MaxVs;               // maximum available Vs
    int32_t i32Q12_MaxVsK;              // maximum Vs = Vbus * MaxVsK
} stc_motor_cv_limit_t;

/* soft-timer for motor control */
typedef struct {
    volatile int32_t    i32_1msCnt;
    int32_t             i32_1msTime;
    volatile int32_t    i32_50msCnt;
    int32_t             i32_50msTime;
    volatile int32_t    i32SpdRegCnt;
    int32_t             i32SpdRegTime;
} stc_motor_timer_t;


/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/** motor AD sample ***********************************************************/
extern stc_adc_sample_t        g_stcAdcSample;

/** motor parameters (p.u. format) ********************************************/
extern stc_motor_para_t        g_stcMotorPara;

/** motor currents ************************************************************/
extern stc_uvw_t               g_stcMotorIuvw;
extern stc_ab_t                g_stcMotorIab;
extern stc_dq_t                g_stcMotorIdq;
extern stc_dq_t                g_stcMotorIdqRef;

/** motor voltage *************************************************************/
extern stc_dq_t                g_stcMotorVdqRef;
extern stc_ab_t                g_stcMotorVabRef;
extern stc_ab_t                g_stcMotorVabRefZ;

/** regulators ****************************************************************/
extern stc_pos_pid_t           g_stcMotorWrPid;
extern stc_inc_pid_t           g_stcMotorIdPid;
extern stc_inc_pid_t           g_stcMotorIqPid;

/** speed set parameters ******************************************************/
extern stc_cmd_speed_set_t     g_stcMotorCmdSpdSet;
extern stc_speed_traj_t        g_stcMotorSpdTraj;

/** protections ***************************************************************/
extern stc_prot_vbus_t         g_stcMotorProtVbus;
extern stc_prot_oc_t           g_stcMotorOverCurrent;

/** soft timer ***************************************************************/
extern stc_motor_timer_t       g_stcMotorTimer;

/** flux observer *************************************************************/
extern stc_flux_observer_t     g_stcMotorFluxObs;

/** SVPWM *********************************************************************/
extern stc_svpwm_calc_t        g_stcMotorSvpwm;
extern stc_pwm_gen_t           g_stcMotorPwmGen;

/** motor run *****************************************************************/
extern stc_motor_run_para_t    g_stcMotorRunPara;

/** motor startup *************************************************************/
extern stc_startup_t           g_stcMotorStartup;
extern stc_stop_ctrl_t         g_stcMotorStopCtrl;
/** miscellaneous *************************************************************/
extern stc_period_avg_t        g_stcMotorVbusAvgF;      // average DC bus filter
extern stc_1stLpf_t            g_stcMotorVdLpf;
extern stc_1stLpf_t            g_stcMotorVqLpf;
extern stc_motor_cv_limit_t    g_stcMotorLimit;


/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize hardware for motor drive
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Motor_HwInit(void);


/**
 ******************************************************************************
 ** \brief initialize software for motor drive
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Motor_SwInit(void);





/**
 ******************************************************************************
 ** \brief stop motor immediately, clear motor control states
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Motor_FastStop(void);


/**
 ******************************************************************************
 ** \brief voltage and current limitation
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Motor_CVLimit(void);

/**
 ******************************************************************************
 ** \brief LOW-priority motor control process, called in system ticker ISR
 **
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Motor_SysTickerIsrHandler(void);


/**
 ******************************************************************************
 ** \brief HIGH-priority motor control process, called in TIMER-4 zero match ISR
 **        WARNING: case add code here, be aware of the execution time
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
extern void Motor_MainIsrHandler(void);


#endif /* MOTOR_SOURCE_MOTOR_CONTROL_H_ */
