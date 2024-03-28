/**
 *******************************************************************************
 * @file  module_math.c
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

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "motor_midwares/module_math.h"

/******************************************************************************/
/* Local pre-processor symbols/macros('define')                               */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with ('extern')       */
/******************************************************************************/
const uint16_t u16Q15_SinTbl[] = {  0x0000,                                         /** 0 */
    0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057F, 0x0648, 0x0711, 0x07D9, /** 10 */
    0x08A2, 0x096B, 0x0A33, 0x0AFB, 0x0BC4, 0x0C8C, 0x0D54, 0x0E1C, 0x0EE4, 0x0FAB, /** 20 */
    0x1073, 0x113A, 0x1201, 0x12C8, 0x138F, 0x1455, 0x151C, 0x15E2, 0x16A8, 0x176E, /** 30 */
    0x1833, 0x18F9, 0x19BE, 0x1A83, 0x1B47, 0x1C0C, 0x1CD0, 0x1D93, 0x1E57, 0x1F1A, /** 40 */
    0x1FDD, 0x209F, 0x2162, 0x2224, 0x22E5, 0x23A7, 0x2467, 0x2528, 0x25E8, 0x26A8, /** 50 */
    0x2768, 0x2827, 0x28E5, 0x29A4, 0x2A62, 0x2B1F, 0x2BDC, 0x2C99, 0x2D55, 0x2E11, /** 60 */
    0x2ECC, 0x2F87, 0x3042, 0x30FC, 0x31B5, 0x326E, 0x3327, 0x33DF, 0x3497, 0x354E, /** 70 */
    0x3604, 0x36BA, 0x3770, 0x3825, 0x38D9, 0x398D, 0x3A40, 0x3AF3, 0x3BA5, 0x3C57, /** 80 */
    0x3D08, 0x3DB8, 0x3E68, 0x3F17, 0x3FC6, 0x4074, 0x4121, 0x41CE, 0x427A, 0x4326, /** 90 */
    0x43D1, 0x447B, 0x4524, 0x45CD, 0x4675, 0x471D, 0x47C4, 0x486A, 0x490F, 0x49B4, /** 100 */
    0x4A58, 0x4AFB, 0x4B9E, 0x4C40, 0x4CE1, 0x4D81, 0x4E21, 0x4EC0, 0x4F5E, 0x4FFB, /** 110 */
    0x5098, 0x5134, 0x51CF, 0x5269, 0x5303, 0x539B, 0x5433, 0x54CA, 0x5560, 0x55F6, /** 120 */
    0x568A, 0x571E, 0x57B1, 0x5843, 0x58D4, 0x5964, 0x59F4, 0x5A82, 0x5B10, 0x5B9D, /** 130 */
    0x5C29, 0x5CB4, 0x5D3E, 0x5DC8, 0x5E50, 0x5ED7, 0x5F5E, 0x5FE4, 0x6068, 0x60EC, /** 140 */
    0x616F, 0x61F1, 0x6272, 0x62F2, 0x6371, 0x63EF, 0x646C, 0x64E9, 0x6564, 0x65DE, /** 150 */
    0x6657, 0x66D0, 0x6747, 0x67BD, 0x6832, 0x68A7, 0x691A, 0x698C, 0x69FD, 0x6A6E, /** 160 */
    0x6ADD, 0x6B4B, 0x6BB8, 0x6C24, 0x6C8F, 0x6CF9, 0x6D62, 0x6DCA, 0x6E31, 0x6E97, /** 170 */
    0x6EFB, 0x6F5F, 0x6FC2, 0x7023, 0x7083, 0x70E3, 0x7141, 0x719E, 0x71FA, 0x7255, /** 180 */
    0x72AF, 0x7308, 0x735F, 0x73B6, 0x740B, 0x7460, 0x74B3, 0x7505, 0x7556, 0x75A6, /** 190 */
    0x75F4, 0x7642, 0x768E, 0x76D9, 0x7723, 0x776C, 0x77B4, 0x77FB, 0x7840, 0x7885, /** 200 */
    0x78C8, 0x790A, 0x794A, 0x798A, 0x79C9, 0x7A06, 0x7A42, 0x7A7D, 0x7AB7, 0x7AEF, /** 210 */
    0x7B27, 0x7B5D, 0x7B92, 0x7BC6, 0x7BF9, 0x7C2A, 0x7C5A, 0x7C89, 0x7CB7, 0x7CE4, /** 220 */
    0x7D0F, 0x7D3A, 0x7D63, 0x7D8A, 0x7DB1, 0x7DD6, 0x7DFB, 0x7E1E, 0x7E3F, 0x7E60, /** 230 */
    0x7E7F, 0x7E9D, 0x7EBA, 0x7ED6, 0x7EF0, 0x7F0A, 0x7F22, 0x7F38, 0x7F4E, 0x7F62, /** 240 */
    0x7F75, 0x7F87, 0x7F98, 0x7FA7, 0x7FB5, 0x7FC2, 0x7FCE, 0x7FD9, 0x7FE2, 0x7FEA, /** 250 */
    0x7FF1, 0x7FF6, 0x7FFA, 0x7FFE, 0x7FFF, 0x8000                                  /** 256 */
};

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief sine function by look-up table
 **
 ** \param [in] i32Q24_angle: Q24 format P.U. angle
 **
 ** \retval     Q15-sin(angle)
 ******************************************************************************/
int32_t Math_Sin(int32_t i32Q24_angle)
{
    int32_t i32Q15_sin;
    int32_t i32Q10_angle = (i32Q24_angle + Q13_BASE) >> 14;
    i32Q10_angle &= 0x3FF;

    if (i32Q10_angle <= 512) {
        if (i32Q10_angle <= 256) {
            i32Q15_sin = ((int32_t)(u16Q15_SinTbl[i32Q10_angle]));
        } else {
            i32Q10_angle = 512 - i32Q10_angle;
            i32Q15_sin = ((int32_t)(u16Q15_SinTbl[i32Q10_angle]));
        }
    } else {
        if (i32Q10_angle <= 768) {
            i32Q10_angle -= 512;
            i32Q15_sin = - ((int32_t)(u16Q15_SinTbl[i32Q10_angle]));
        } else {
            i32Q10_angle = 1024 - i32Q10_angle;
            i32Q15_sin = -((int32_t)(u16Q15_SinTbl[i32Q10_angle]));
        }
    }
    return (i32Q15_sin);
}

/**
 ******************************************************************************
 ** \brief cosine function by look-up table
 **
 ** \param [in] i32Q24_angle: Q24 format P.U. angle
 **
 ** \retval     Q15-cos(angle)
 ******************************************************************************/
int32_t Math_Cos(int32_t i32Q24_angle)
{
    return (Math_Sin(i32Q24_angle + 0x400000));
}

/**
 ******************************************************************************
 ** \brief sine/cosine function by look-up table
 **
 ** \param [in] i32Q24_angle: Q24 format P.U. angle
 **
 ** \retval     Q15-sin(angle), Q15-cos(angle)
 ******************************************************************************/
void Math_SinCos(int32_t i32Q24_angle, int32_t *pi32Q15_sin, int32_t *pi32Q15_cos)
{
    int32_t i32Q10_angle = (i32Q24_angle + Q13_BASE) >> 14;
    i32Q10_angle &= 0x3FF;

    if (i32Q10_angle <= 512) {
        if (i32Q10_angle <= 256) {
            *pi32Q15_sin = ((int32_t)(u16Q15_SinTbl[i32Q10_angle]));
            *pi32Q15_cos = ((int32_t)(u16Q15_SinTbl[256 - i32Q10_angle]));
        } else {
            *pi32Q15_sin = ((int32_t)(u16Q15_SinTbl[512 - i32Q10_angle]));
            *pi32Q15_cos = -((int32_t)(u16Q15_SinTbl[i32Q10_angle - 256]));
        }
    } else {
        if (i32Q10_angle <= 768) {
            *pi32Q15_sin = -((int32_t)(u16Q15_SinTbl[i32Q10_angle - 512]));
            *pi32Q15_cos = -((int32_t)(u16Q15_SinTbl[768 - i32Q10_angle]));
        } else {
            *pi32Q15_sin = -((int32_t)(u16Q15_SinTbl[1024 - i32Q10_angle]));
            *pi32Q15_cos = ((int32_t)(u16Q15_SinTbl[i32Q10_angle - 768]));
        }
    }
}

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
int32_t Math_Atan2(int32_t i32_x, int32_t i32_y)
{
    int32_t i32Q24_angle;
    int32_t i32Q16_Tan;

    int32_t i32X = i32_x;
    int32_t i32Y = i32_y;

    /** special case handling *************************************************/
    if(0 == i32X)
    {
        if (i32Y > 0)
        {
            i32Q24_angle = 0x400000;        // 90 degree
        }
        else if (i32Y < 0)
        {
            i32Q24_angle = 0xC00000;        // 270 degree
        }
        else
        {
            i32Q24_angle = 0;           // 0 degree
        }

        return i32Q24_angle;
    }
    if(0 == i32Y)
    {
        if (i32X > 0)
        {
            i32Q24_angle = 0;           // 0 degree
        }
        else
        {
            i32Q24_angle = 0x800000;     // 180 degree
        }

        return i32Q24_angle;
    }

    /** atan(a) = a/8 + [0.2731*a* (1-|a|)]/(2*pi), in PU, with -1<a<1 ********/
    if (abs(i32_x) >= abs(i32_y))
    {
        i32Q24_angle = ((Q30_BASE / i32X) * i32Y) >> 9; // a/8
        i32Q16_Tan = i32Q24_angle >> 5;
        i32Q16_Tan = (i32Q16_Tan * (Q16_BASE - abs(i32Q16_Tan))) >> 16; //remain term, <0.25
        i32Q24_angle += (i32Q16_Tan * 11394) >> 10;  // 11394 = Q18(0.2731/2pi)

        if (i32_x > 0)
        {
            i32Q24_angle &= 0xFFFFFF;
        }
        else
        {
            i32Q24_angle += 0x800000;
        }
    }
    else
    {
        i32Q24_angle = ((Q30_BASE / i32Y) * i32_x) >> 9; // a/8
        i32Q16_Tan = i32Q24_angle >> 5;
        i32Q16_Tan = (i32Q16_Tan * (Q16_BASE - abs(i32Q16_Tan))) >> 16;  //remain term, <0.25
        i32Q24_angle += (i32Q16_Tan * 11394) >> 10;  // 11394 = Q18(0.2731/2pi)

        if (i32_y > 0)
        {
            i32Q24_angle = 0x400000 - i32Q24_angle;
        }
        else
        {
            i32Q24_angle = 0xC00000 - i32Q24_angle;
        }
    }

    return i32Q24_angle;
}

/**
 ******************************************************************************
 ** \brief Math_Sqrt(i32_x) function
 **
 ** \param [in] integer between 0x0000_0000 ~ 0x7FFF_FFFF
 **
 ** \retval     square root of i32_x
 ******************************************************************************/
int32_t Math_Sqrt(int32_t i32_x)
{
    if(i32_x <= 0)
    {
        return 0;
    }

    uint32_t u32Input = (uint32_t)(i32_x);
    uint32_t u32SqrtX = 0;
    uint32_t u32Temp;

    u32Temp = u32SqrtX + (1U << 30);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 30);
    }

    u32Temp = u32SqrtX + (1U << 28);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 28);
    }

    u32Temp = u32SqrtX + (1U << 26);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 26);
    }

    u32Temp = u32SqrtX + (1U << 24);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 24);
    }

    u32Temp = u32SqrtX + (1U << 22);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 22);
    }

    u32Temp = u32SqrtX + (1U << 20);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 20);
    }

    u32Temp = u32SqrtX + (1U << 18);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 18);
    }

    u32Temp = u32SqrtX + (1U << 16);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 16);
    }

    u32Temp = u32SqrtX + (1U << 14);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 14);
    }

    u32Temp = u32SqrtX + (1U << 12);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 12);
    }

    u32Temp = u32SqrtX + (1U << 10);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 10);
    }

    u32Temp = u32SqrtX + (1U << 8);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 8);
    }

    u32Temp = u32SqrtX + (1U << 6);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 6);
    }

    u32Temp = u32SqrtX + (1U << 4);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 4);
    }
    u32Temp = u32SqrtX + (1U << 2);
    u32SqrtX  >>= 1;
    if (u32Temp <= u32Input)
    {
        u32Input -= u32Temp;
        u32SqrtX += (1U << 2);
    }
    u32Temp = u32SqrtX + (1U << 0);
    u32SqrtX  >>= 1;

    if (u32Temp <= u32Input)
    {
        /*u32Input -= u32Temp;*/
        u32SqrtX += (1U << 0);
    }
    return ((int32_t)(u32SqrtX));
}




