/**
 *******************************************************************************
 * @file  motor_control.c
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
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_source/motor_control.h"

/*****************************************************************************/
/* local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/** motor AD sample ***********************************************************/
stc_adc_sample_t        g_stcAdcSample;

/** motor parameters (p.u. format) ********************************************/
stc_motor_para_t        g_stcMotorPara;

/** motor currents ************************************************************/
stc_uvw_t               g_stcMotorIuvw;      // sampled u-v-w current
stc_ab_t                g_stcMotorIab;       // sampled alpha-beta current
stc_dq_t                g_stcMotorIdq;       // sampled d-q current
stc_dq_t                g_stcMotorIdqRef;    // reference d-q current


/** motor voltage *************************************************************/
stc_dq_t                g_stcMotorVdqRef;    // reference d-q voltage
stc_ab_t                g_stcMotorVabRef;    // reference alpha-beta voltage
stc_ab_t                g_stcMotorVabRefZ;   // previous reference alpha-beta voltage

/** regulators ****************************************************************/
stc_pos_pid_t           g_stcMotorWrPid;     //  PID
stc_inc_pid_t           g_stcMotorIdPid;     // Id current PID
stc_inc_pid_t           g_stcMotorIqPid;     // Iq current PID

/** speed set parameters ******************************************************/
stc_cmd_speed_set_t     g_stcMotorCmdSpdSet;
stc_speed_traj_t        g_stcMotorSpdTraj;

/** protections ***************************************************************/
stc_prot_vbus_t         g_stcMotorProtVbus;
stc_prot_oc_t           g_stcMotorOverCurrent;

/** soft timer ****************************************************************/
stc_motor_timer_t       g_stcMotorTimer;

/** flux observer *************************************************************/
stc_flux_observer_t     g_stcMotorFluxObs;

/** motor brake ***************************************************************/


/** protections ***************************************************************/


/** SVPWM *********************************************************************/
stc_svpwm_calc_t        g_stcMotorSvpwm;
stc_pwm_gen_t           g_stcMotorPwmGen;

/** motor run *****************************************************************/
stc_motor_run_para_t    g_stcMotorRunPara;

/** motor startup *************************************************************/
stc_startup_t           g_stcMotorStartup;
stc_stop_ctrl_t         g_stcMotorStopCtrl;

/** miscellaneous *************************************************************/
stc_period_avg_t        g_stcMotorVbusAvgF;      /** average DC bus filter */
stc_1stLpf_t            g_stcMotorVdLpf;         /** Vd low-pass filter */
stc_1stLpf_t            g_stcMotorVqLpf;         /** Vq low-pass filter */
stc_motor_cv_limit_t    g_stcMotorLimit;

/*****************************************************************************/
/* function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize hardware for motor drive
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Motor_HwInit(void)
{

    uint16_t u16PeakCnt;       // peak count of up-down counter of TIMER-4
    uint16_t u16DeadTimeCnt;     // count of dead time of TIMER-4
    uint8_t  u8ZeroIsrMaskCnt;      // Zero ISR mask count of TIMER-4
    int32_t i32Temp;
    int32_t i32AdcTrigCnt;        // compare count of ADC trigger before zero-match
    float32_t f32Temp;
    boolean_t bResult;

    stc_sample_cfg_t        stcSampleCfg;   // parameters for sampling

    /***************************************************************************
     ** check carrier frequency and set control frequency and ZeroIsrMaskCnt
     **     1. legal carrier frequency is set between 4kHz - 32kHz
     **     1. maximum allowed sample/control frequency is 8kHz
     **     2. thus only use 0, 1, 2, 3 as zero ISR mask count of TIMER-4
     **************************************************************************/
    if (ui_i32CarrierFreqHz <= 4 * 1000) {
        ui_i32CarrierFreqHz = 4 * 1000;
        u8ZeroIsrMaskCnt = 0;
        g_stcMotorRunPara.i32Fs = ui_i32CarrierFreqHz;
    } else if (ui_i32CarrierFreqHz <= 8 * 1000) {
        u8ZeroIsrMaskCnt = 0;
        g_stcMotorRunPara.i32Fs = ui_i32CarrierFreqHz;
    } else if (ui_i32CarrierFreqHz <= 16 * 1000) {
        u8ZeroIsrMaskCnt = 0;
        g_stcMotorRunPara.i32Fs = ui_i32CarrierFreqHz;
    } else if (ui_i32CarrierFreqHz <= 24 * 1000) {
        u8ZeroIsrMaskCnt = 0;
        g_stcMotorRunPara.i32Fs = ui_i32CarrierFreqHz;
    } else if (ui_i32CarrierFreqHz <= 32 * 1000) {
        u8ZeroIsrMaskCnt = 1;
        g_stcMotorRunPara.i32Fs = ui_i32CarrierFreqHz / 2;
    } else {
        ui_i32CarrierFreqHz = 32 * 1000;
        u8ZeroIsrMaskCnt = 1;
        g_stcMotorRunPara.i32Fs = ui_i32CarrierFreqHz / 2;

    }
    /***************************************************************************
     ** check dead time of inverter, the minimum dead time is 1 us
     **************************************************************************/
    if (ui_f32DeadTimeUs < 1.0f) {
        ui_f32DeadTimeUs = 1.0f;
    }

    /***************************************************************************
     ** calculate peak count and dead-time count of TIMER-4
     **************************************************************************/
    i32Temp = ((gstcSysClk.i32_APB1Clk_Hz / ui_i32CarrierFreqHz) >> 1);
    u16PeakCnt = (uint16_t)i32Temp;

    f32Temp = ui_f32DeadTimeUs * (float32_t)gstcSysClk.i32_APB1Clk_Hz / 1000000.0f;
    u16DeadTimeCnt = (uint16_t)f32Temp;

    i32Temp = gstcSysClk.i32_APB1Clk_Hz / 1000000;
    f32Temp = ui_f32AdcTrigTimeUs * (float32_t)i32Temp;
    i32AdcTrigCnt = (int32_t)f32Temp;
    /***************************************************************************
     ** initialize TIMER-4 for SVPWM generation
     **************************************************************************/
    InitMcu_InitMotorTimer(u16PeakCnt, u16DeadTimeCnt, u8ZeroIsrMaskCnt);

    i32Temp = Q15(ui_f32MaxDutyRatio);
    bResult = Svpwm_InitCalc((int32_t)u16PeakCnt, i32Temp, &g_stcMotorSvpwm);
    if(FALSE == bResult)
    {
        return;
    }

    /***************************************************************************
    ** initialize ADC0 for sampling
    **************************************************************************/
    InitMcu_Adc((uint16_t)i32AdcTrigCnt);

    /***********************************************************************
     ** initialize sample function
    **********************************************************************/
    InitMcu_Adc((uint16_t)i32AdcTrigCnt);
    stcSampleCfg.f32Fs = (float32_t)g_stcMotorRunPara.i32Fs;
    stcSampleCfg.f32IuvwK = 1.0f / (ui_f32IuvwSampleRs * ui_f32IuvwSampleK * ui_f32BaseCurrent);
    stcSampleCfg.f32VbusK = (1.0f * ui_f32VbusSampleK) / (ui_f32BaseVoltage);
    stcSampleCfg.i32RefIuvwOffset = ui_i32IuvwRefOffset;
    stcSampleCfg.i32MaxIuvwOffsetBias = ui_i32IuvwMaxOffsetBias;
    stcSampleCfg.f32DetectDelayTimeMs = ui_i32AdcCheckDelayMs;
    stcSampleCfg.f32OffsetCheckTimeMs = ui_f32OffsetCheckTimeMs;
    Adc_InitSample(stcSampleCfg, &g_stcAdcSample);

    /***************************************************************************
     ** initialize system ticker for LOW-priority control
     **************************************************************************/
    InitMcu_SystemTicker(ui_i32SysTickerFreqHz);

    /***************************************************************************
     ** other peripherals
     **************************************************************************/

}


/**
 ******************************************************************************
 ** \brief initialize software for motor drive
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Motor_SwInit(void)
{
    stc_period_avg_cfg_t    stcPeriodAvgCfg;// parameters for average filter
    stc_pid_cfg_t           stcPidCfg;      // parameters for PID regulators
    stc_1stLpf_cfg_t        stcLpfCfg;      // parameters for low-pass filters
    stc_stop_ctrl_cfg_t     stcStopCtrlCfg; // parameters for stop motor control
    float32_t f32Temp;
    /***************************************************************************
     ** initialize p.u. format motor parameters
     **************************************************************************/
    float32_t f32InvVB = 1.0f / ui_f32BaseVoltage;      // reciprocal of base voltage
    float32_t f32InvIB = 1.0f / ui_f32BaseCurrent;      // reciprocal of base current
    float32_t f32InvFB = 1.0f / ui_f32BaseFrequency;    // reciprocal of base frequency
    float32_t f32InvLB = CONST_F32_2PI * ui_f32BaseFrequency * ui_f32BaseCurrent * f32InvVB;    // reciprocal of base inductance
    float32_t f32InvFluxB = (CONST_F32_2PI * ui_f32BaseFrequency) / ui_f32BaseVoltage;          // reciprocal of base flux linkage

    /***************************************************************************
     ** initialize p.u. format motor parameters
     **************************************************************************/
    g_stcMotorPara.i32PolePairs = ui_i32PolePairs;
    g_stcMotorPara.f32Rs = ui_f32Rs * ui_f32BaseCurrent * f32InvVB;
    g_stcMotorPara.f32Ld = ui_f32Ld * f32InvLB * 0.001f;
    g_stcMotorPara.f32Lq = ui_f32Lq * f32InvLB * 0.001f;
    g_stcMotorPara.f32LambdaF = (MOTOR_KE_2_LAMBDA * ui_f32Ke / (float32_t)ui_i32PolePairs) * f32InvFluxB;

    if (FALSE == g_stcMotorRunPara.bSysRdyFlag) {
        /***********************************************************************
         ** initialize counter in system-ticker
         **********************************************************************/
        g_stcMotorTimer.i32_1msTime = (int32_t)(1 * ui_i32SysTickerFreqHz / 1000);
        g_stcMotorTimer.i32_50msTime = (int32_t)(50 * ui_i32SysTickerFreqHz / 1000);

        f32Temp = ui_f32SpdRegPeriodMs * (float32_t)ui_i32SysTickerFreqHz / 1000.0f;
        g_stcMotorTimer.i32SpdRegTime = (int32_t)f32Temp;

        ui_f32SpdRegPeriodMs = (1000.0f * (float32_t)g_stcMotorTimer.i32SpdRegTime) / (float32_t)ui_i32SysTickerFreqHz;

        g_stcMotorTimer.i32_1msCnt = 0;
        g_stcMotorTimer.i32_50msCnt = 0;
        g_stcMotorTimer.i32SpdRegCnt = 0;

        /***********************************************************************
         ** initialize system protections
         **********************************************************************/
        // DC bus voltage protection
        stc_prot_vbus_cfg_t stcProtVbusCfg;
        stcProtVbusCfg.f32Fs = (float32_t)g_stcMotorRunPara.i32Fs;
        stcProtVbusCfg.f32AbnormalHighVbus = ui_f32AbnormalHighVbus * f32InvVB;
        stcProtVbusCfg.f32AbnormalLowVbus = ui_f32AbnormalLowVbus * f32InvVB;
        stcProtVbusCfg.f32AbnormalTimeUs = (float32_t)ui_i32AbnormalVbusTimeUs;
        stcProtVbusCfg.f32OverVoltageThold = ui_f32OverVoltageThold * f32InvVB;
        stcProtVbusCfg.f32OverVoltageTimeUs = (float32_t)ui_i32OverVoltageTimeUs;
        stcProtVbusCfg.f32UnderVoltageThold = ui_f32UnderVoltageThold * f32InvVB;
        stcProtVbusCfg.f32UnderVoltageTimeUs = (float32_t)ui_i32UnderVoltageTimeUs;
        stcProtVbusCfg.f32ErrClearTimeMs = (float32_t)ui_i32ErrClearTimeMs;
        Prot_InitMonitorVbus(stcProtVbusCfg, &g_stcMotorProtVbus);

        // over-current protection
        stc_prot_oc_cfg_t   stcProtOcCfg;
        stcProtOcCfg.f32Fs = (float32_t)g_stcMotorRunPara.i32Fs;
        stcProtOcCfg.f32PeakOcpThold = ui_f32PeakOverCurrThold * f32InvIB;
        stcProtOcCfg.f32PeakOcpTimeUs = (float32_t)ui_i32PeakOverCurrTimeUs;
        stcProtOcCfg.f32ErrClearTimeMs = (float32_t)ui_i32ErrClearTimeMs;
        Prot_InitOverCurrent(stcProtOcCfg, &g_stcMotorOverCurrent);

        /***********************************************************************
         ** initialize command speed/torque settings
         **********************************************************************/
        Speed_InitCmdSpdSet(&g_stcMotorCmdSpdSet);

        /***********************************************************************
         ** initialize test codes for auto-run mode
         **********************************************************************/

        /***********************************************************************
         ** initialize motor running parameters
         **********************************************************************/
        g_stcMotorRunPara.i32Q12_CmdWr = 0;
        g_stcMotorRunPara.i32CmdWrDir = ROTOR_DIR_CCW;
        g_stcMotorRunPara.i32Q12_TgtWr = 0;
        g_stcMotorRunPara.i32TgtWrDir = ROTOR_DIR_CCW;
        g_stcMotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
        g_stcMotorRunPara.u32FaultCode = ERR_NONE;

#if(TRUE == EN_VSP_CMD)
        AnaSpd_Init(&g_stcMotorAnaSpdSet);
#endif


        g_stcMotorLimit.i32Q12_MaxVsK = Q12(0.57735f);

        g_stcMotorRunPara.bSysRdyFlag = TRUE;

    }

    /***************************************************************************
     ** disable all control target
     **************************************************************************/
    g_stcMotorRunPara.bEnSpdPid = FALSE;
    g_stcMotorRunPara.bEnSpdTraj = FALSE;
    g_stcMotorRunPara.bEnCurrentPid = FALSE;
    g_stcMotorRunPara.bEnBrake = FALSE;

    // average DC bus voltage filter, call in system ticker
    stcPeriodAvgCfg.f32Fs = (float32_t)ui_i32SysTickerFreqHz;
    stcPeriodAvgCfg.f32PeriodMs = 10.0f;                // for 50Hz AC supply
    stcPeriodAvgCfg.f32Y0 = (float32_t)g_stcMotorRunPara.i32Q12_Vbus / 4096.0f;
    Filter_InitPeriodAvg(stcPeriodAvgCfg, &g_stcMotorVbusAvgF);

    /***************************************************************************
     ** initialize speed regulation functions
     **************************************************************************/
    Speed_InitSpdTraj(&g_stcMotorSpdTraj);

    /***************************************************************************
     ** initialize flux observer
     **************************************************************************/
    stc_flux_obs_cfg_t stcFluxObsCfg;
    stcFluxObsCfg.f32FB = ui_f32BaseFrequency;
    stcFluxObsCfg.f32Fs = (float32_t)g_stcMotorRunPara.i32Fs;
    stcFluxObsCfg.f32GammaHalf = ui_f32FluxObsGamma;
    stcFluxObsCfg.f32L0 = ((g_stcMotorPara.f32Ld + g_stcMotorPara.f32Lq) / 2.0f);
    stcFluxObsCfg.f32L1 = ((g_stcMotorPara.f32Ld - g_stcMotorPara.f32Lq) / 2.0f);
    stcFluxObsCfg.f32Rs = g_stcMotorPara.f32Rs;
    stcFluxObsCfg.f32LambdaF = g_stcMotorPara.f32LambdaF;
    stcFluxObsCfg.f32PLLKp = ui_f32FluxObsPLLKp;
    stcFluxObsCfg.f32PLLKi = ui_f32FluxObsPLLKi;
    stcFluxObsCfg.i32WrLpfFc = 50;
    Flux_Init(stcFluxObsCfg, &g_stcMotorFluxObs);

    /***************************************************************************
     ** initialize PI regulators
     **************************************************************************/
    // speed regulator
    stcPidCfg.f32Fs = 1000.0f / ui_f32SpdRegPeriodMs;                /** sample frequency of PI */
    stcPidCfg.f32Kp = ui_f32SpdKp * ui_f32BaseFrequency * f32InvIB; /** Kp of PI */
    stcPidCfg.f32Ki = ui_f32SpdKi * ui_f32BaseFrequency * f32InvIB; /** Ki of PI */
    stcPidCfg.f32OutMin = 0.0f;                                        /** minimum output of PI */
    stcPidCfg.f32OutMax = ui_f32MaxNormaIs * f32InvIB;              /** maximum output of PI */
    stcPidCfg.f32ErrMax = ui_f32MaxSpdErr * f32InvFB;               /** maximum error */
    stcPidCfg.f32Y0 = 0.0f;                                            /** initial output */
    PID_InitPosPid(stcPidCfg, &g_stcMotorWrPid);

    // d-axis current regulator
    stcPidCfg.f32Fs = (float32_t)g_stcMotorRunPara.i32Fs;                       /** sample frequency of PI */
    stcPidCfg.f32Kp = ui_f32IdKp * ui_f32BaseCurrent * f32InvVB;    /** Kp of PI */
    stcPidCfg.f32Ki = ui_f32IdKi * ui_f32BaseCurrent * f32InvVB;    /** Ki of PI */
    stcPidCfg.f32OutMin = -1.0f;                                    /** minimum output of PI */
    stcPidCfg.f32OutMax = 1.0f;                                     /** maximum output of PI */
    stcPidCfg.f32ErrMax = ui_f32MaxIdErr * f32InvIB;                /** maximum error */
    stcPidCfg.f32Y0 = 0.0f;                                            /** initial output */
    PID_InitIncPid(stcPidCfg, &g_stcMotorIdPid);

    // q-axis current regulator
    stcPidCfg.f32Fs = (float32_t)g_stcMotorRunPara.i32Fs;                       /** sample frequency of PI */
    stcPidCfg.f32Kp = ui_f32IqKp * ui_f32BaseCurrent * f32InvVB;    /** Kp of PI */
    stcPidCfg.f32Ki = ui_f32IqKi * ui_f32BaseCurrent * f32InvVB;    /** Ki of PI */
    stcPidCfg.f32OutMin = -1.0f;                                    /** minimum output of PI */
    stcPidCfg.f32OutMax = 1.0f;                                     /** maximum output of PI */
    stcPidCfg.f32ErrMax = ui_f32MaxIqErr * f32InvIB;                /** maximum error */
    stcPidCfg.f32Y0 = 0.0f;                                            /** initial output */
    PID_InitIncPid(stcPidCfg, &g_stcMotorIqPid);

    /***************************************************************************
     * miscellaneous control parameters
     **************************************************************************/
    //LPF filters
    stcLpfCfg.f32Fs = (float32_t)ui_i32SysTickerFreqHz;        // (Hz) 1kHz
    stcLpfCfg.f32Fc = (float32_t)ui_i32VdLpfFc;                // (Hz) cut-off frequency
    stcLpfCfg.f32Y0 = 0.0f;                            // initial output
    Filter_InitFirstLpf(stcLpfCfg, &g_stcMotorVdLpf);

    //LPF filters
    stcLpfCfg.f32Fs = (float32_t)ui_i32SysTickerFreqHz;        // (Hz) 1kHz
    stcLpfCfg.f32Fc = (float32_t)ui_i32VqLpfFc;                // (Hz) cut-off frequency
    stcLpfCfg.f32Y0 = 0.0f;                            // initial output
    Filter_InitFirstLpf(stcLpfCfg, &g_stcMotorVqLpf);

    /***************************************************************************
     ** initialize startup control
     **************************************************************************/
    Startup_Init(&g_stcMotorStartup);



    /***************************************************************************
     ** initialize stop control
     **************************************************************************/
    stcStopCtrlCfg.bEnBrakeStop = FALSE;
    Stop_Init(stcStopCtrlCfg, &g_stcMotorStopCtrl);
}


/**
 ******************************************************************************
 ** \brief stop motor immediately, clear motor control states
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Motor_FastStop(void)
{
//    /***************************************************************************
//     ** disable PWM output
//     **************************************************************************/
//    InitMcu_EnMotorPwm(FALSE);
//    InitMcu_UpdateMotorPwmDuty(0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff);

//    /***************************************************************************
//     ** reset motor run parameters
//     **************************************************************************/
//    g_stcMotorRunPara.u32MotorStatus = MOTOR_STUS_IDLE;
//    g_stcMotorRunPara.bEnSpdPid = FALSE;
//    g_stcMotorRunPara.bEnSpdTraj = FALSE;
//    g_stcMotorRunPara.bEnCurrentPid = FALSE;
//    g_stcMotorRunPara.bEnBrake = FALSE;
//    g_stcMotorRunPara.i32Q12_TgtWr = 0;
//    g_stcMotorRunPara.i32Q12_RealTimeWr = 0;

//    /***************************************************************************
//     ** clear voltage and current information
//     **************************************************************************/
//    g_stcMotorIdq.i32Q12_D = 0;
//    g_stcMotorIdq.i32Q12_Q = 0;
//    g_stcMotorIdqRef.i32Q12_D = 0;
//    g_stcMotorIdqRef.i32Q12_Q = 0;

//    g_stcMotorVabRefZ.i32Q12_Alpha = 0;
//    g_stcMotorVabRefZ.i32Q12_Beta = 0;
//    g_stcMotorVabRef.i32Q12_Alpha = 0;
//    g_stcMotorVabRef.i32Q12_Beta = 0;
//    g_stcMotorVdqRef.i32Q12_D = 0;
//    g_stcMotorVdqRef.i32Q12_Q = 0;

//    /***************************************************************************
//     ** clear SVPWM calculation and PWM generation parameters
//     **************************************************************************/
//    g_stcMotorPwmGen.u16Uon = 0xffff;
//    g_stcMotorPwmGen.u16Uoff = 0xffff;
//    g_stcMotorPwmGen.u16Von = 0xffff;
//    g_stcMotorPwmGen.u16Voff = 0xffff;
//    g_stcMotorPwmGen.u16Won = 0xffff;
//    g_stcMotorPwmGen.u16Woff = 0xffff;
//    
//    g_stcMotorAnaSpdSet.bRdy = FALSE;
}


/**
 ******************************************************************************
 ** \brief voltage and current limitation
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Motor_CVLimit(void)
{
    int32_t i32Q12_Temp;
    int32_t i32Q24_Temp;
    int32_t i32Q12_VdLpf;

    /** DC bus voltage filtering **********************************************/
    g_stcMotorRunPara.i32Q12_AvgVbus = Filter_PeriodAvg(g_stcMotorRunPara.i32Q12_Vbus, &g_stcMotorVbusAvgF);

    i32Q12_VdLpf = Filter_FirstLpf(g_stcMotorVdqRef.i32Q12_D, &g_stcMotorVdLpf);
    /* Filter_FirstLpf(g_stcMotorVdqRef.i32Q12_Q, &g_stcMotorVqLpf); */

    /** update maximum available Vs ***************************************/
    g_stcMotorLimit.i32Q12_MaxVs = ((g_stcMotorLimit.i32Q12_MaxVsK * g_stcMotorRunPara.i32Q12_AvgVbus) >> 12);

    /** set maximum VdRef as maximum Vs ***************************************/
    g_stcMotorIdPid.i32Q27_OutMax = Q15(g_stcMotorLimit.i32Q12_MaxVs);
    g_stcMotorIdPid.i32Q27_OutMin = -g_stcMotorIdPid.i32Q27_OutMax;

    i32Q12_Temp = abs(i32Q12_VdLpf);
    i32Q24_Temp = g_stcMotorLimit.i32Q12_MaxVs * g_stcMotorLimit.i32Q12_MaxVs - i32Q12_Temp * i32Q12_Temp;
    i32Q12_Temp = Math_Sqrt(i32Q24_Temp);

    g_stcMotorIqPid.i32Q27_OutMax = Q15(i32Q12_Temp);
    g_stcMotorIqPid.i32Q27_OutMin = -g_stcMotorIqPid.i32Q27_OutMax;

}







/**
 ******************************************************************************
 ** \brief LOW-priority motor control process, called in system ticker ISR
 **
 **
 ** \param [in] none
 **
 ** \retval     none
 ******************************************************************************/
void Motor_SysTickerIsrHandler(void)
{
    /***************************************************************************
     * timer counting
     **************************************************************************/
    g_stcMotorTimer.i32_1msCnt++;
    g_stcMotorTimer.i32_50msCnt++;
    g_stcMotorTimer.i32SpdRegCnt++;

    /***************************************************************************
     * motor control functions in every system-ticker ISR
     **************************************************************************/
    /** 1. functions that DO NOT care motor status ****************************/
    Prot_MonitorVbus(g_stcMotorRunPara.i32Q12_Vbus, &g_stcMotorProtVbus);

    Motor_CVLimit();

#if(TRUE == EN_VSP_CMD)
    AnaSpd_SetCmdSpd(&g_stcMotorAnaSpdSet);
    g_stcMotorCmdSpdSet.i32CmdRpm = g_stcMotorAnaSpdSet.i32CmdRpm;
#endif

    /** 2. functions only when motor-status != MOTOR_STUS_IDLE ****************/
    if (MOTOR_STUS_IDLE != g_stcMotorRunPara.u32MotorStatus) {
        if (TRUE == g_stcMotorRunPara.bEnBrake) {
            //TODO
        }
    }

    /***************************************************************************
     * 1ms period
     **************************************************************************/
    if (g_stcMotorTimer.i32_1msCnt >= g_stcMotorTimer.i32_1msTime) {
        /** restart counting **************************************************/
        g_stcMotorTimer.i32_1msCnt = 0;
    }

    /***************************************************************************
     * 50ms period
     **************************************************************************/
    if (g_stcMotorTimer.i32_50msCnt >= g_stcMotorTimer.i32_50msTime) {
        /** restart counting **************************************************/
        g_stcMotorTimer.i32_50msCnt = 0;

    }

    /***************************************************************************
     * speed regulation period
     **************************************************************************/
    if (g_stcMotorTimer.i32SpdRegCnt >= g_stcMotorTimer.i32SpdRegTime) {
        /** speed trajectory setting and speed regulation *********************/
        if (TRUE == g_stcMotorRunPara.bEnSpdTraj) {
            g_stcMotorRunPara.i32Q12_TgtWr = Speed_TgtSpdTraj(g_stcMotorRunPara.i32Q12_CmdWr, &g_stcMotorSpdTraj);
        }
        if (TRUE == g_stcMotorRunPara.bEnSpdPid) {
            g_stcMotorIdqRef.i32Q12_Q = PID_PosPid(g_stcMotorRunPara.i32Q12_TgtWr, g_stcMotorRunPara.i32Q12_RealTimeWr, &g_stcMotorWrPid);
        }

        /** restart counting **************************************************/
        g_stcMotorTimer.i32SpdRegCnt = 0;
    }
}

/******************************************************************************
** \brief HIGH-priority motor control process, called in TIMER-4 zero match ISR
**        WARNING: case add code here, be aware of the execution time
**
** \param [in] none
**
** \retval     none
******************************************************************************/
void Motor_MainIsrHandler(void)
{
    /***************************************************************************
    * protections (a sample time delay)
    **************************************************************************/
    Prot_MotorOverCurrent(g_stcMotorIuvw, &g_stcMotorOverCurrent);

    /***********************************************************************
    * handle fault before driving motor: assure no PWM action with fault
    **********************************************************************/
    if (ERR_NONE != g_stcMotorRunPara.u32FaultCode)
    {
//        Motor_FastStop();
    }

    /***************************************************************************
      * convert ADC data, and apply Clarke transformation
      **************************************************************************/
     Adc_Sample(&g_stcAdcSample);
     Transf_Clarke(g_stcMotorIuvw, &g_stcMotorIab);

    /***************************************************************************
     * when running motor is required
     **************************************************************************/
    if (MOTOR_STUS_IDLE != g_stcMotorRunPara.u32MotorStatus)
    {
        Flux_Observer(g_stcMotorVabRefZ, g_stcMotorIab, &g_stcMotorFluxObs);
        g_stcMotorRunPara.i32Q12_RealTimeWr = g_stcMotorFluxObs.i32Q12_WrF;

        /***********************************************************************
         * startup/stop control
         **********************************************************************/
        if (MOTOR_STUS_DRV_RUN == g_stcMotorRunPara.u32MotorStatus)
        {
            Startup_Control(&g_stcMotorStartup);     // MOTOR_STUS_DRV_RUN
        }
        else
        {
            Stop_Control(&g_stcMotorStopCtrl);       // MOTOR_STUS_DRV_STOP
        }

        /***********************************************************************
         * current-loop control
         **********************************************************************/
        if (TRUE == g_stcMotorRunPara.bEnCurrentPid)
        {
            /** sine/cosine updating for voltage and current transformation ***/
            Math_SinCos(g_stcMotorRunPara.i32Q24_DrvAngle, &g_stcMotorIdq.i32Q15_Sin, &g_stcMotorIdq.i32Q15_Cos);
            g_stcMotorVdqRef.i32Q15_Sin = g_stcMotorIdq.i32Q15_Sin;
            g_stcMotorVdqRef.i32Q15_Cos = g_stcMotorIdq.i32Q15_Cos;

            /** current transformation ****************************************/
            Transf_Park(g_stcMotorIab, &g_stcMotorIdq);

            /** current regulation ********************************************/
            g_stcMotorVdqRef.i32Q12_D = PID_IncPid(g_stcMotorIdqRef.i32Q12_D, g_stcMotorIdq.i32Q12_D, &g_stcMotorIdPid);
            g_stcMotorVdqRef.i32Q12_Q = PID_IncPid(g_stcMotorIdqRef.i32Q12_Q, g_stcMotorIdq.i32Q12_Q, &g_stcMotorIqPid);

            /** store previous voltage, and inverse PARK transformation *******/
            g_stcMotorVabRefZ.i32Q12_Alpha = g_stcMotorVabRef.i32Q12_Alpha;
            g_stcMotorVabRefZ.i32Q12_Beta = g_stcMotorVabRef.i32Q12_Beta;
            Transf_InvPark(g_stcMotorVdqRef, &g_stcMotorVabRef);

            /** PWM calculation, and generate switch pattern of SVPWM *********/
            Svpwm_CalcDuration(g_stcMotorVabRef.i32Q12_Alpha, g_stcMotorVabRef.i32Q12_Beta,
                               g_stcMotorRunPara.i32Q12_Vbus,  &g_stcMotorSvpwm);
            Svpwm_7SegSymmPwm(g_stcMotorSvpwm.u8Sector, g_stcMotorSvpwm.u16T0, g_stcMotorSvpwm.u16T1,
                              g_stcMotorSvpwm.u16T2, &g_stcMotorPwmGen);
        }
        if (ROTOR_DIR_CCW == g_stcMotorRunPara.i32TgtWrDir)
        {
            InitMcu_UpdateMotorPwmDuty(g_stcMotorPwmGen.u16Uon, g_stcMotorPwmGen.u16Uoff,
                                       g_stcMotorPwmGen.u16Von, g_stcMotorPwmGen.u16Voff,
                                       g_stcMotorPwmGen.u16Won, g_stcMotorPwmGen.u16Woff);
        }
        else if (ROTOR_DIR_CW == g_stcMotorRunPara.i32TgtWrDir)
        {
            InitMcu_UpdateMotorPwmDuty(g_stcMotorPwmGen.u16Von, g_stcMotorPwmGen.u16Voff,
                                       g_stcMotorPwmGen.u16Uon, g_stcMotorPwmGen.u16Uoff,
                                       g_stcMotorPwmGen.u16Won, g_stcMotorPwmGen.u16Woff);
        }
        else
        {
            InitMcu_UpdateMotorPwmDuty(0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff);

        }
    }
    else
    {
        Motor_FastStop();       // always apply fast-stop
    }
}

