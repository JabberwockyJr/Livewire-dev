/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
//#include "mod_fatfs_chan.h"
#include "cli.h"
#include "task.h"
#include "sbe.h"
//#include "hall_eff.h"

//
typedef struct main_app_struct
{
    bool app_done;
    bool acq_task_flag;
    bool logging_task_flag;
    bool streaming_task_flag;
    main_acq_struct acq_struct;
    main_logging_struct logging_struct;
    main_streaming_struct streaming_struct;
    main_sbe_struct sbe_struct;
}main_app_struct, *main_app_struct_ptr;



/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
int app_init(void);
int app_cleanup(void);

#endif  // APP_H
