/**
 *******************************************************************************
 * @file  protect.h
 * @brief This file contains motor run protect function.
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
#ifndef MOTOR_MIDWARES_PROTECTION_H_
#define MOTOR_MIDWARES_PROTECTION_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/module_include.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/
// configuration parameters of DC bus voltage protection
typedef struct {
    float32_t f32Fs;                    // (Hz) sample frequency
    float32_t f32AbnormalHighVbus;
    float32_t f32AbnormalLowVbus;
    float32_t f32AbnormalTimeUs;
    float32_t f32OverVoltageThold;
    float32_t f32OverVoltageTimeUs;
    float32_t f32UnderVoltageThold;
    float32_t f32UnderVoltageTimeUs;
    float32_t f32ErrClearTimeMs;
} stc_prot_vbus_cfg_t;

// DC bus voltage protection
typedef struct {
    int32_t i32AbnormalCnt;
    int32_t i32OvpCnt;
    int32_t i32UvpCnt;
    int32_t i32AbnormalTime;
    int32_t i32OvpTime;
    int32_t i32UvpTime;
    int32_t i32ErrClrTime;
    int32_t i32Q12_OvpThreshold;
    int32_t i32Q12_UvpThreshold;
    int32_t i32Q12_AbnormalHighVbus;
    int32_t i32Q12_AbnormalLowVbus;
} stc_prot_vbus_t;

// configuration parameters of over current protection
typedef struct {
    float32_t f32Fs;                // (Hz) sample frequency
    float32_t f32PeakOcpThold;
    float32_t f32PeakOcpTimeUs;
    float32_t f32ErrClearTimeMs;
} stc_prot_oc_cfg_t;

// motor software over-current protection
typedef struct {
    int32_t i32PeakOcpCnt;              // peak over-current counter
    int32_t i32PeakOcpTime;             // peak over-current response time
    int32_t i32Q12_PeakOcpThold;        // peak over-current threshold
    int32_t i32ErrClrTime;              // over-current error clear time
} stc_prot_oc_t;



/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize function of DC bus voltage protection
 **
 ** \param [in] stcCfg: configuration parameters
 ** \param [in] *pstc:: pointer to DC bus voltage protection
 **
 ** \retval     none
 ******************************************************************************/
extern void Prot_InitMonitorVbus(stc_prot_vbus_cfg_t stcCfg, stc_prot_vbus_t *pstc);

/**
 ******************************************************************************
 ** \brief monitor DC bus voltage
 **             monitor over voltage, under voltage, and abnormal voltage,
 **             record protection reason and do protection process,
 **             clear fault code is system recovers to normal after certain time
 **
 ** \param [in] i32Q8_Vbus: DC bus voltage
 ** \param [in] *pstc:     : DC bus voltage protection structure pointer
 **
 ** \retval     none
 ******************************************************************************/
extern void Prot_MonitorVbus(int32_t i32Q12_Vbus, stc_prot_vbus_t *pstc);

/**
 ******************************************************************************
 ** \brief initialize function of over current protection
 **
 ** \param [in] stcCfg: configuration parameters
 ** \param [in] *pstc:: pointer to over current protection
 **
 ** \retval     none
 ******************************************************************************/
extern void Prot_InitOverCurrent(stc_prot_oc_cfg_t stcCfg, stc_prot_oc_t *pstc);

/**
 ******************************************************************************
 ** \brief motor over current protection
 **             record fault reason and do protection process,
 **             clear fault code is system recovers to normal after certain time
 **
 ** \param [in] stcIuvw: three-phase motor current
 ** \param [in] *pstc: : over current protection structure
 **
 ** \retval     none
 ******************************************************************************/
extern void Prot_MotorOverCurrent(stc_uvw_t stcIuvw, stc_prot_oc_t *pstc);

#endif /* MOTOR_MIDWARES_PROTECTION_H_ */
