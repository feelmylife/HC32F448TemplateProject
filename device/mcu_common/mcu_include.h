/**
 *******************************************************************************
 * @file  mcu_include.h
 * @brief Chip related configuration including.
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

#ifndef MCU_COMMON_MCU_INCLUDE_H_
#define MCU_COMMON_MCU_INCLUDE_H_

/*******************************************************************************
** Includes files
*******************************************************************************/
#define USE_DDL_DRIVER_MODE         0x01
#define USE_REG_DRIVER_MODE         0x02

#define USE_DRIVER_MODE             USE_REG_DRIVER_MODE//USE_DDL_DRIVER_MODE//


#include "mcu_driver/hc32f448.h"
#include "hc32_ll_def.h"
#include "mcu_driver/inc/hc32_ll.h"
#include "mcu_config/hardware_definitions.h"
#include "mcu_config/hardware_config.h"
#include "mcu_config/hardware_mapping.h"
#include "mcu_config/isr.h"
#include "mcu_config/init_mcu.h"

/*******************************************************************************
** Global pre-processor symbols/macros ('#define')
*******************************************************************************/

/*******************************************************************************
** Global type definitions ('typedef')
*******************************************************************************/

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

#endif /* DEVICE_MCU_INCLUDE_H_ */
