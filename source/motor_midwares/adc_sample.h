/**
 *******************************************************************************
 * @file  adc_sample.h
 * @brief This file contains mcu adc sample data conversion and use function.
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
#ifndef MOTOR_MIDWARES_ADC_SAMPLE_H_
#define MOTOR_MIDWARES_ADC_SAMPLE_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/module_include.h"
#include "mcu_common/mcu_include.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/
#define ADC_CH_AMOUNT   16

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/

/** configuration parameters for sampling *************************************/
typedef struct {
    float32_t f32Fs;
    float32_t f32IuvwK;
    float32_t f32VbusK;
    float32_t f32DetectDelayTimeMs;
    float32_t f32OffsetCheckTimeMs;
    int32_t   i32RefIuvwOffset;
    int32_t   i32MaxIuvwOffsetBias;
} stc_sample_cfg_t;

/** ADC sample parameters *****************************************************/
typedef struct {
    boolean_t bAdcRdy;
    int32_t i32ShuntNum;
    int32_t i32AdcValue1;
    int32_t i32AdcValue2;
    int32_t i32OffsetDetectCnt;
    int32_t i32OffsetDetectTime;
    int32_t i32DelayTime;
    int32_t i32IuOffset;
    int32_t i32IvOffset;
    int32_t i32IwOffset;
    int32_t i32RefOffset;
    int32_t i32MaxOffsetBias;
    int32_t i32Q24_VbusK;
    int32_t i32Q24_IuvwK;
} stc_adc_sample_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief initialize ADC sample parameters
 **
 ** \param [in] stcCfg: configuration parameters
 ** \param [in] *pstc: pointer to sample structure
 **
 ** \retval     none
 ******************************************************************************/
extern void Adc_InitSample(stc_sample_cfg_t stcCfg, stc_adc_sample_t *pstc);

/**
 ******************************************************************************
 ** \brief ADC sampling, sample motor current and DC bus voltage
 **
 ** \param [in] *pstc: pointer to sample structure
 **
 ** \retval     none
 ******************************************************************************/
extern void Adc_Sample(stc_adc_sample_t *pstc);

/**
 ******************************************************************************
 ** \brief check the offset of current sampling
 **
 ** \param [in] *pstc: pointer to sample structure
 **
 ** \retval     none
 ******************************************************************************/
extern void Adc_CheckOffset(stc_adc_sample_t *pstc);

#endif /* MOTOR_MIDWARES_ADC_SAMPLE_H_ */
