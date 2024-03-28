/**
 *******************************************************************************
 * @file  base_types.h
 * @brief This file contains data types and constant definitions.
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
#ifndef MOTOR_MIDWARES_BASE_TYPES_H_
#define MOTOR_MIDWARES_BASE_TYPES_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

#include "hc32_ll_def.h"
/******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/
/** logical data type (only values are TRUE and FALSE) */
typedef uint32_t     boolean_t;


/** ASCCI character for string generation (8 bit) */
typedef char         char_t;



/** function pointer type to void/uint8_t function */
typedef void         (*func_ptr_arg1_t)(uint8_t);

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/
#define TRUE            (1)
#define FALSE           (0)

/* define sign() function *****************************************************/
#define SIGN(X)         (((X) > 0) ? (1) : ((X) < 0 ? (-1) : (0)))


#define Q15_SQRT3       56756L          /** = Q15(1.732)    */
#define Q15_INV_SQRT3   18919L          /** = Q15(0.577)    */
#define Q15_2INV_SQRT3  37837L          /** = Q15(1.155)    */
#define Q15_1P5         49152L          /** = Q15(1.5)      */
#define Q15_INV_3       10923L          /** = Q15(1/3)      */
#define CONST_F32_2PI   (6.2831853f)    /** = 2*pi          */



/** QN math defines ***********************************************************/
#define Q0_BASE     0x00000001L
#define Q1_BASE     0x00000002L
#define Q2_BASE     0x00000004L
#define Q3_BASE     0x00000008L
#define Q4_BASE     0x00000010L
#define Q5_BASE     0x00000020L
#define Q6_BASE     0x00000040L
#define Q7_BASE     0x00000080L
#define Q8_BASE     0x00000100L
#define Q9_BASE     0x00000200L
#define Q10_BASE    0x00000400L
#define Q11_BASE    0x00000800L
#define Q12_BASE    0x00001000L
#define Q13_BASE    0x00002000L
#define Q14_BASE    0x00004000L
#define Q15_BASE    0x00008000L
#define Q16_BASE    0x00010000L
#define Q17_BASE    0x00020000L
#define Q18_BASE    0x00040000L
#define Q19_BASE    0x00080000L
#define Q20_BASE    0x00100000L
#define Q21_BASE    0x00200000L
#define Q22_BASE    0x00400000L
#define Q23_BASE    0x00800000L
#define Q24_BASE    0x01000000L
#define Q25_BASE    0x02000000L
#define Q26_BASE    0x04000000L
#define Q27_BASE    0x08000000L
#define Q28_BASE    0x10000000L
#define Q29_BASE    0x20000000L
#define Q30_BASE    0x40000000L

#define Q0(X)       ((int32_t)((Q0_BASE)*(X)))
#define Q1(X)       ((int32_t)((Q1_BASE)*(X)))
#define Q2(X)       ((int32_t)((Q2_BASE)*(X)))
#define Q3(X)       ((int32_t)((Q3_BASE)*(X)))
#define Q4(X)       ((int32_t)((Q4_BASE)*(X)))
#define Q5(X)       ((int32_t)((Q5_BASE)*(X)))
#define Q6(X)       ((int32_t)((Q6_BASE)*(X)))
#define Q7(X)       ((int32_t)((Q7_BASE)*(X)))
#define Q8(X)       ((int32_t)((Q8_BASE)*(X)))
#define Q9(X)       ((int32_t)((Q9_BASE)*(X)))
#define Q10(X)      ((int32_t)((Q10_BASE)*(X)))
#define Q11(X)      ((int32_t)((Q11_BASE)*(X)))
#define Q12(X)      ((int32_t)((Q12_BASE)*(X)))
#define Q13(X)      ((int32_t)((Q13_BASE)*(X)))
#define Q14(X)      ((int32_t)((Q14_BASE)*(X)))
#define Q15(X)      ((int32_t)((Q15_BASE)*(X)))
#define Q16(X)      ((int32_t)((Q16_BASE)*(X)))
#define Q17(X)      ((int32_t)((Q17_BASE)*(X)))
#define Q18(X)      ((int32_t)((Q18_BASE)*(X)))
#define Q19(X)      ((int32_t)((Q19_BASE)*(X)))
#define Q20(X)      ((int32_t)((Q20_BASE)*(X)))
#define Q21(X)      ((int32_t)((Q21_BASE)*(X)))
#define Q22(X)      ((int32_t)((Q22_BASE)*(X)))
#define Q23(X)      ((int32_t)((Q23_BASE)*(X)))
#define Q24(X)      ((int32_t)((Q24_BASE)*(X)))
#define Q25(X)      ((int32_t)((Q25_BASE)*(X)))
#define Q26(X)      ((int32_t)((Q26_BASE)*(X)))
#define Q27(X)      ((int32_t)((Q27_BASE)*(X)))
#define Q28(X)      ((int32_t)((Q28_BASE)*(X)))
#define Q29(X)      ((int32_t)((Q29_BASE)*(X)))
#define Q30(X)      ((int32_t)((Q30_BASE)*(X)))

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

#endif /* MOTOR_MIDWARES_BASE_TYPES_H_ */
