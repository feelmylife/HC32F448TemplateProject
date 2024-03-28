/**
 *******************************************************************************
 * @file  module_math.h
 * @brief This file contains math function.
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
#ifndef MOTOR_MIDWARES_MODULE_MATH_H_
#define MOTOR_MIDWARES_MODULE_MATH_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include "motor_midwares/base_types.h"

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

extern const uint16_t u16Q15_SinTbl[257];
/**
 ******************************************************************************
 ** \brief sine function by look-up table
 **
 ** \param [in] i32Q24_angle: Q24 format P.U. angle
 **
 ** \retval     Q15-sin(angle)
 ******************************************************************************/
extern int32_t Math_Sin(int32_t i32Q24_angle);

/**
 ******************************************************************************
 ** \brief cosine function by look-up table
 **
 ** \param [in] i32Q24_angle: Q24 format P.U. angle
 **
 ** \retval     Q15-cos(angle)
 ******************************************************************************/
extern int32_t Math_Cos(int32_t i32Q24_angle);

/**
 ******************************************************************************
 ** \brief sine/cosine function by look-up table
 **
 ** \param [in] i32Q24_angle: Q24 format P.U. angle
 **
 ** \retval     Q15-sin(angle), Q15-cos(angle)
 ******************************************************************************/
extern void Math_SinCos(int32_t i32Q24_angle, int32_t *pi32Q15_sin, int32_t *pi32Q15_cos);

/**
 ******************************************************************************
 ** \brief Math_Atan2(x, y) function
 **         1. the input x, and y should have the same Q-format
 **         2. both x and y should less than 2^24, otherwise precision degrades
 **
 ** \param [in] i32_x: x-axis value, < 2^24
 ** \param [in] i32_y: y-axis value, < 2^24
 **
 ** \retval     int32_t i32Q24_angle: Q24 format P.U. angle
 ******************************************************************************/
extern int32_t Math_Atan2(int32_t i32_x, int32_t i32_y);

/**
 ******************************************************************************
 ** \brief Math_Sqrt(i32_x) function
 **
 ** \param [in] integer between 0x0000_0000 ~ 0x7FFF_FFFF
 **
 ** \retval     square root of i32_x
 ******************************************************************************/
extern int32_t Math_Sqrt(int32_t i32_x);

#endif /* MOTOR_MIDWARES_MODULE_MATH_H_ */
