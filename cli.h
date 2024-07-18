/***************************************************************************//**
 * @file
 * @brief cli micrium os kernel examples functions
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

#ifndef CLI_MICRIUMOS_H
#define CLI_MICRIUMOS_H

#define M_PI     3.14159265358979323846
#define WHEEL_RADIUS 0.0381 //radius [m] (1.5 in)

#include <stdbool.h>
#include "task.h"
//2023 Feb 7 eb reformatting cli.h to include function prototypes
#include <string.h>
#include <stdio.h>
#include "cli.h"
#include "sl_cli.h"
#include "sl_cli_instances.h"
#include "sl_cli_arguments.h"
#include "sl_cli_handles.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "os.h"
#include "ff.h"
#include "sdio.h"
#include "diskio.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#define CLI_DEMO_TASK_STACK_SIZE     256
#define CLI_DEMO_TASK_PRIO            15
/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

//void echo_str(sl_cli_command_arg_t *arguments);
//void echo_int(sl_cli_command_arg_t *arguments);
//void led_cmd(sl_cli_command_arg_t *arguments);
void wtest_cmd_f(sl_cli_command_arg_t *arguments);
void cli_ls_cmd_f(sl_cli_command_arg_t *arguments);
//FRESULT scan_files_1 (char* path );
void mountfs_cmd_f(sl_cli_command_arg_t *arguments);
void unmountfs_cmd_f(sl_cli_command_arg_t *arguments);
void catfs_cmd_f(sl_cli_command_arg_t *arguments);
void cli_rm_cmd_f (sl_cli_command_arg_t *arguments);
void cli_mkdir_cmd_f (sl_cli_command_arg_t *arguments);
void cli_mv_cmd_f (sl_cli_command_arg_t *arguments);
void a2d_start_cmd_f (sl_cli_command_arg_t *arguments);
void a2d_stop_cmd_f (sl_cli_command_arg_t *arguments);

void lw_prof_cmd_f (sl_cli_command_arg_t *arguments);
void lw_hover_cmd_f (sl_cli_command_arg_t *arguments);
void lw_pwm_cmd_f (sl_cli_command_arg_t *arguments);
void lw_zero_dy_cmd_f (sl_cli_command_arg_t *arguments);

void echo_str(sl_cli_command_arg_t *arguments);
void echo_int(sl_cli_command_arg_t *arguments);
void write_to_sd_f(sl_cli_command_arg_t *arguments);
void calibrate_f(sl_cli_command_arg_t *arguments);
void read_sbe(sl_cli_command_arg_t *arguments);

// 5/24/24 GSCHELLENBERG - Add odrive control commands
void odrive_vel_cmd_f (sl_cli_command_arg_t *arguments);
void odrive_torque_cmd_f (sl_cli_command_arg_t *arguments);
void odrive_read_cmd_f (sl_cli_command_arg_t *arguments);
void odrive_write_cmd_f (sl_cli_command_arg_t *arguments);

// 5/30/24 GSCHELLENBERG - Add time setting command
void set_time_cmd_f (sl_cli_command_arg_t *arguments);

// 6/6/24 GSCHELLENBERG - Add data streaming toggle command
void stream_cmd_f (sl_cli_command_arg_t *arguments);

// 7/11/24 mbarrera - more odrive commands
void odrive_set_gain_cmd_f(sl_cli_command_arg_t *arguments);
void odrive_control_mode_cmd_f (sl_cli_command_arg_t *arguments);

/*******************************************************************************
 *********************   LOCAL FUNCTION VARIABLES   ***************************
 ******************************************************************************/

extern int state_machine_select; //toggles state machine in the control loop
extern int prof_upper_limit; //perhaps use float type for more precision
extern int prof_lower_limit; //these limits bound the profiling mission
extern int sd_card_state; //decides if data is written to sd card
extern int new_file_state; //decides if data is written to new file
extern int calibrate_state; //toggles odrive calibration
extern int upper_limit_display;
extern int lower_limit_display;
extern int streaming_toggle_state; //decides if data is streamed
extern int streaming_rate_state; //decides if data is streamed
extern int64_t time_offset;
extern float reference_vel;

/***************************************************************************//**
 * Initialize example
 ******************************************************************************/
bool cli_app_init(void);

#endif  // CLI_MICRIUMOS_H
