/**
 *******************************************************************************
 * @file  module_include.h
 * @brief global definitions for motor control, basic function including.
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

#ifndef MOTOR_MIDWARES_MODULE_INCLUDE_H_
#define MOTOR_MIDWARES_MODULE_INCLUDE_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/base_types.h"
#include "motor_midwares/coordinate_transform.h"
#include "motor_midwares/module_math.h"
#include "motor_midwares/filter.h"
#include "motor_midwares/flux_observer.h"
#include "motor_midwares/svpwm.h"
#include "motor_midwares/pid.h"

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
/** rotation direction ********************************************************/
#define ROTOR_DIR_CCW       0           /** counter-clockwise rotation */
#define ROTOR_DIR_CW        1           /** clockwise rotation */

/** define motor state ********************************************************/
#define MOTOR_STUS_IDLE         0x0000U  /** idle status */
#define MOTOR_STUS_DRV_RUN      0x0001U  /** drive in running status */
#define MOTOR_STUS_DRV_STOP     0x0002U  /** drive in stopping status */

/** constant for converting Ke(Vrms/krpm) to lambda(Wb) ***********************/
#define MOTOR_KE_2_LAMBDA       0.007799f

/** define error codes in motor control ***************************************/
#define ERR_NONE        0x0000U          /** no error */
#define ERR_AD_OFFSET   0x0001U          /** AD offset error */
#define ERR_OC_PEAK     0x0002U          /** over-current: peak current */
#define ERR_OC_RMS      0x0004U          /** over-current: RMS current */
#define ERR_OC_HW       0x0008U          /** over-current: hardware over current */
#define ERR_VDC_OV      0x0010U          /** DC bus over voltage */
#define ERR_VDC_UV      0x0020U          /** DC bus under voltage */
#define ERR_VDC_ABNORM  0x0040U          /** abnormal DC bus voltage */
#define ERR_MOTOR_OP    0x0080U          /** motor over power */
#define ERR_MOTOR_OT    0x0100U          /** motor over temperature */
#define ERR_IPM_OT      0x0200U          /** IPM over temperature */
#define ERR_LOCK_ROTOR  0x0400U          /** motor rotor locked */
#define ERR_LACK_PHASE  0x0800U          /** motor lack phase */
#define ERR_COMM        0x1000U          /** communication error */
#define ERR_SWWD_INT    0x2000U          /** software watch-dog interrupt */
#define ERR_HWWD_INT    0x4000U          /** hardware watch-dog interrupt */
#define ERR_UNDEF_INT   0x8000U          /** undefined interrupt */
#define ERR_DMA_FAIL    0x00010000U      /** DMA transfer failed */
#define ERR_SYS_CLK     0x00020000U      /** system clock error */
#define ERR_IPD_ERR     0x00040000U      /** initial-position-detection error */

/** transform angle from degree to Q24(p.u.) format ***************************/
#define DEG_TO_Q24(X)   ((int32_t)((X) * 46603))
#define Q24_TO_DEG(X)   ((((X) >> 12) * 360L) >> 12)

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

#endif /* MOTOR_MIDWARES_MODULE_INCLUDE_H_ */
