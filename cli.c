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
//#include "mod_fatfs_chan.h"
#include "pwm.h"
//#include "sl_pwm.h"
//#include "sl_pwm_init_pwm1_config.h"
#include "em_gpio.h"
#include "btl_interface.h"
#include "odrive_comms.h"
#include "task.h"

#include "sl_iostream.h"
#include "sl_iostream_init_instances.h"
#include "sl_iostream_handles.h"

//#include "mod_fatfs_chan.h"

int state_machine_select = 1; //defaults to encoder state machine
int prof_upper_limit = 0; //initialize profiling boundaries
int prof_lower_limit = 0;
int upper_limit_display = 0;
int lower_limit_display = 0;
int sd_card_state = 0;
int new_file_state = 0;
int calibrate_state = 0;
int streaming_toggle_state = 0;
int streaming_rate_state = 0;
float reference_vel = 0;
uint8_t duty_cycle = 0;

/*
 * Bootloading variable initialization
 */
#define CLI_DEMO_TASK_STACK_SIZE     256
#define CLI_DEMO_TASK_PRIO            15
FIL superfile;
uint32_t bytes_read;
//uint8_t* buffer_t;
volatile uint8_t buffer_t[52100];
#define BTL_PARSER_CTX_SZ  0x200
//static uint8_t parserContext[BTL_PARSER_CTX_SZ];
//static BootloaderParserCallbacks_t parserCallbacks;
uint32_t total_bytes = 0;

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

//gschellen 5/28/24 - odrive cmds
char odrive_read_buffer[80];
extern bool run_custom_odrive_command;
extern char custom_odrive_command[80];

int64_t time_offset = 0;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES for bootloading  ***************************
 ******************************************************************************/


/*******************************************************************************
 ***************************  LOCAL VARIABLES for bootloading ********************************
 ******************************************************************************/

static const sl_cli_command_info_t cmd__echostr = \
  SL_CLI_COMMAND(echo_str,
                 "echoes string arguments to the output",
                 "Just a string...",
                 {  SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__echoint = \
  SL_CLI_COMMAND(echo_int,
                 "echoes integer arguments to the output",
                 "Just a number...",
                 {  SL_CLI_ARG_END, });


///*******************************************************************************
// *******************************   DEFINES   ***********************************
// ******************************************************************************/
//
//#define CLI_DEMO_TASK_STACK_SIZE     256
//#define CLI_DEMO_TASK_PRIO            15

///*******************************************************************************
// *********************   LOCAL FUNCTION PROTOTYPES   ***************************
// ******************************************************************************/
//
////void echo_str(sl_cli_command_arg_t *arguments);
////void echo_int(sl_cli_command_arg_t *arguments);
////void led_cmd(sl_cli_command_arg_t *arguments);
//void wtest_cmd_f(sl_cli_command_arg_t *arguments);
//void cli_ls_cmd_f(sl_cli_command_arg_t *arguments);
////FRESULT scan_files_1 (char* path );
//void mountfs_cmd_f(sl_cli_command_arg_t *arguments);
//void unmountfs_cmd_f(sl_cli_command_arg_t *arguments);
//void catfs_cmd_f(sl_cli_command_arg_t *arguments);
//void cli_rm_cmd_f (sl_cli_command_arg_t *arguments);
//void cli_mkdir_cmd_f (sl_cli_command_arg_t *arguments);
//void cli_mv_cmd_f (sl_cli_command_arg_t *arguments);
//void a2d_start_cmd_f (sl_cli_command_arg_t *arguments);
//void a2d_stop_cmd_f (sl_cli_command_arg_t *arguments);

#ifdef TASK_TEST
/* Example Task Defines:            */
#define  APP_EXAMPLE_TASK_PRIO            21u  /*   Task Priority.                 */
#define  APP_EXAMPLE_TASK_STK_SIZE       256u  /*   Stack size in CPU_STK.         */

                                                        /* Example Task Data:      */
OS_TCB   App_ExampleTaskTCB;                            /*   Task Control Block.   */
CPU_STK  App_ExampleTaskStk[APP_EXAMPLE_TASK_STK_SIZE]; /*   Stack.                */

                                                        /* Example Task Code:      */
void  App_ExampleTask (void  *p_arg);                   /*   Function.             */

void  App_TaskCreate (void)
{
    RTOS_ERR     err;

    OSTaskCreate(&App_ExampleTaskTCB,                /* Pointer to the task's TCB.  */
                 "Example Task.",                    /* Name to help debugging.     */
                 &App_ExampleTask,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  APP_EXAMPLE_TASK_PRIO,             /* Task's priority.            */
                 &App_ExampleTaskStk[0],             /* Pointer to base of stack.   */
                 (APP_EXAMPLE_TASK_STK_SIZE / 10u),  /* Stack limit, from base.     */
                  APP_EXAMPLE_TASK_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  10u,                               /* Messages in task queue.     */
                  0u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CHK,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) {
        /* Handle error on task creation. */
    }
}

void  App_ExampleTask (void  *p_arg)
{
  static int i=0;
  OS_RATE_HZ os_tick_rate;
  RTOS_ERR p_err;

  int run_count = 0;

    /* Use argument. */
   (void)&p_arg;
   os_tick_rate = OSTimeTickRateHzGet (&p_err);


    while (DEF_TRUE) {
        printf("example task activation count %d \r\n",i++);
        OSTimeDly (os_tick_rate/10, OS_OPT_TIME_PERIODIC,&p_err);
        /* All tasks should be written as infinite loops. */
        run_count++;
        if (run_count == 10) break;
    }
}

#endif

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
/*
static const sl_cli_command_info_t cmd__echostr = \
  SL_CLI_COMMAND(echo_str,
                 "echoes string arguments to the output",
                 "Just a string...",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__echoint = \
  SL_CLI_COMMAND(echo_int,
                 "echoes integer arguments to the output",
                 "Just a number...",
                 { SL_CLI_ARG_INT8, SL_CLI_ARG_ADDITIONAL, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__led = \
  SL_CLI_COMMAND(led_cmd,
                 "Change an led status",
                 "led number: 0 or 1"SL_CLI_UNIT_SEPARATOR "instruction: on, off, or toggle",
                 { SL_CLI_ARG_UINT8, SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });
*/
static const sl_cli_command_info_t cmd__wtest = \
  SL_CLI_COMMAND(wtest_cmd_f,
                 "tests file system",
                 "enter a file name and a string to store",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__ls = \
  SL_CLI_COMMAND(cli_ls_cmd_f,
                 "tests file system ls cmd",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 2 Aug 2022
static const sl_cli_command_info_t cmd__mount = \
  SL_CLI_COMMAND(mountfs_cmd_f,
                 "mount file system cmd",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 2 Aug 2022
static const sl_cli_command_info_t cmd__unmount = \
  SL_CLI_COMMAND(unmountfs_cmd_f,
                 "unmount file system",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 2 Aug 2022
static const sl_cli_command_info_t cmd__cat = \
  SL_CLI_COMMAND(catfs_cmd_f,
                 "cat the file",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 2 Aug 2022
static const sl_cli_command_info_t cmd__rm = \
  SL_CLI_COMMAND(cli_rm_cmd_f,
                 "remove the file",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 3 Aug 2022
static const sl_cli_command_info_t cmd__mdir = \
  SL_CLI_COMMAND(cli_mkdir_cmd_f,
                 "make a new directory",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 3 Aug 2022
static const sl_cli_command_info_t cmd__mv = \
  SL_CLI_COMMAND(cli_mv_cmd_f,
                 "rename the file",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });

// maibui - 10 Sept 2022
/*
static const sl_cli_command_info_t cmd__a2d_start = \
  SL_CLI_COMMAND(a2d_start_cmd_f,
                 "a2d_start log",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });
                 */
/*
// maibui - 10 Sept 2022
static const sl_cli_command_info_t cmd__a2d_stop = \
  SL_CLI_COMMAND(a2d_stop_cmd_f,
                 "a2d_stop log",
                 "",
                 { SL_CLI_ARG_WILDCARD, SL_CLI_ARG_END, });
                 */

// eb - 7 Feb 2023
static const sl_cli_command_info_t cmd__lw_prof = \
  SL_CLI_COMMAND(lw_prof_cmd_f,
                 "lw profiling command",
                 "enter 'profile sbe or hall upperlimit lowerlimit'",
                 { SL_CLI_ARG_INT32, SL_CLI_ARG_INT32, SL_CLI_ARG_INT32, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__lw_hover = \
  SL_CLI_COMMAND(lw_hover_cmd_f,
                 "lw hover command",
                 "enter 'hover targetdepth'",
                 { SL_CLI_ARG_INT32, SL_CLI_ARG_END, });

// eb - 11 April 2023
static const sl_cli_command_info_t cmd__lw_pwm = \
  SL_CLI_COMMAND(lw_pwm_cmd_f,
                 "lw pwm command",
                 "",
                 { SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

// eb - 24 May 2023
static const sl_cli_command_info_t cmd__lw_zero_dy = \
  SL_CLI_COMMAND(lw_zero_dy_cmd_f,
                 "lw zero displacement command",
                 "enter 'zero'",
                 { SL_CLI_ARG_END, });

//mbarrera - 31 July 2023
static const sl_cli_command_info_t cmd_sd_write = \
  SL_CLI_COMMAND(write_to_sd_f,
                 "lw writing to sd card (toggle, new file)",
                 "",
                 {  SL_CLI_ARG_INT8, SL_CLI_ARG_INT8, SL_CLI_ARG_END, });

//gschellen - 6/6/24 - add streaming toggling
static const sl_cli_command_info_t cmd_stream = \
  SL_CLI_COMMAND(stream_cmd_f,
                 "lw streaming (toggle)",
                 "",
                 {  SL_CLI_ARG_INT8, SL_CLI_ARG_INT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd_calibrate = \
  SL_CLI_COMMAND(calibrate_f,
                 "calibrate displacement using SBE49",
                 "",
                 { SL_CLI_ARG_INT8, SL_CLI_ARG_INT32, SL_CLI_ARG_INT32, SL_CLI_ARG_END, });

/*static const sl_cli_command_info_t cmd_read_sbe = \
  SL_CLI_COMMAND(read_sbe,
                 "read sbe",
                 "no arguments",
                 {  SL_CLI_ARG_END, });
*/

// 5/24/24 GSCHELLENBERG - Add odrive control commands
static const sl_cli_command_info_t cmd__odrive_vel = \
  SL_CLI_COMMAND(odrive_vel_cmd_f,
                 "odrive velocity command",
                 "enter 'vel target speed'",
                 { SL_CLI_ARG_INT32, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__odrive_torque = \
  SL_CLI_COMMAND(odrive_torque_cmd_f,
                 "odrive torque command",
                 "enter 'vel target torque'",
                 { SL_CLI_ARG_INT32, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__odrive_gain = \
  SL_CLI_COMMAND(odrive_set_gain_cmd_f,
                 "odrive set gain command",
                 "enter 'velocity_gain vel_integrator_gain'",
                 { SL_CLI_ARG_INT32, SL_CLI_ARG_INT32, SL_CLI_ARG_END});

static const sl_cli_command_info_t cmd__odrive_control_mode = \
  SL_CLI_COMMAND(odrive_control_mode_cmd_f,
                 "odrive control mode command",
                 "enter '1 for idle, 8 for closed-loop control'",
                 { SL_CLI_ARG_UINT8, SL_CLI_ARG_END});

static const sl_cli_command_info_t cmd__odrive_read = \
  SL_CLI_COMMAND(odrive_read_cmd_f,
                 "odrive read command",
                 "enter 'odrvread target'",
                 { SL_CLI_ARG_STRING, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__odrive_write = \
  SL_CLI_COMMAND(odrive_write_cmd_f,
                 "odrive write command",
                 "enter 'odrvwrite target data'",
                 { SL_CLI_ARG_STRING, SL_CLI_ARG_STRING, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cmd__set_time = \
  SL_CLI_COMMAND(set_time_cmd_f,
                 "set time command",
                 "enter 'settime time (UNIX ms)'",
                 { SL_CLI_ARG_STRING, SL_CLI_ARG_END, });

/*
static sl_cli_command_entry_t a_table[] = {
  { "echo_str", &cmd__echostr, false },
  { "echo_int", &cmd__echoint, false },
  { "led", &cmd__led, false },
  { "wtest", &cmd__fswtest, false },
  { "ls", &cmd__ls_cmd, false },
  { NULL, NULL, false },
};
*/
static sl_cli_command_entry_t a_table[] = {
  { "mount_fs", &cmd__mount, false },
  { "unmount_fs", &cmd__unmount, false },
  { "ls", &cmd__ls, false },
  { "wtest", &cmd__wtest, false },
  { "cat", &cmd__cat, false },
  { "rm", &cmd__rm, false },
  { "mkdir", &cmd__mdir, false },
  { "mv", &cmd__mv, false },
//  { "a2d_start", &cmd__a2d_start, false},
//  { "a2d_stop", &cmd__a2d_stop, false},
  { "profile", &cmd__lw_prof, false},
  { "hover", &cmd__lw_hover, false},
  { "pwm", &cmd__lw_pwm, false},
  { "zero", &cmd__lw_zero_dy, false},
  { "bootload_uart", &cmd__echostr,false },
  { "bootload_sd", &cmd__echoint, false },
  { "write_sd", &cmd_sd_write, false},
  { "stream", &cmd_stream, false},
//  { "calibrate", &cmd_calibrate, false},
  { "vel", &cmd__odrive_vel, false},
  { "torque", &cmd__odrive_torque, false},
  { "set_gain", &cmd__odrive_gain, false},
  { "odrv_control_mode", &cmd__odrive_control_mode,false},
  { "odrvread", &cmd__odrive_read, false},
  { "odrvwrite", &cmd__odrive_write, false},
  { "settime", &cmd__set_time, false},
  { NULL, NULL, false },
};

static sl_cli_command_group_t a_group = {
  { NULL },
  false,
  a_table
};

/*******************************************************************************
 *************************  EXPORTED VARIABLES   *******************************
 ******************************************************************************/

sl_cli_command_group_t *command_group = &a_group;

/*******************************************************************************
 *************************   LOCAL FUNCTIONS   *********************************
 ******************************************************************************/

/***************************************************************************//**
 * Callback for wtest
 *
 * This function is used as a callback when the wtest command is called
 * in the cli. It simply writing a provided strings in to the provided file
 ******************************************************************************/
void wtest_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;
  char *filename;

  printf("<<fswrite command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=2)
  {
    printf("command: wtest filename str_to_write\n");
    return;
  }

  // command: fswtest filename string => filename = arg0, str = arg1

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
  {
 //     if (i==0)
        filename = sl_cli_get_argument_string(arguments, 0);
 //     if (i==1)
        ptr_string = sl_cli_get_argument_string(arguments, 1);
  }
  printf("fn = %s, str = %s\r\n",filename, ptr_string);

  fs_write_tst_f(filename,ptr_string);  // define in mod_fatfs_chan.c

}

void mountfs_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;

  printf("<<mount command>>\r\n");

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++) {
    ptr_string = sl_cli_get_argument_string(arguments, i);
  }
  printf("%s\r\n", ptr_string);

  fs_mount_f();

}

void unmountfs_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;

  printf("<<unmount command>>\r\n");

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++) {
    ptr_string = sl_cli_get_argument_string(arguments, i);
  }
  printf("%s\r\n", ptr_string);

  fs_unmount_f();

}

void catfs_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;
  char *filename;

  printf("<<cat command>>\r\n");
  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("command: cat filename\n");
    return;
  }

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++) {
    ptr_string = filename = sl_cli_get_argument_string(arguments, i);
  }

  printf("%s\r\n", ptr_string);

  fs_cat_f(filename);

}


void cli_ls_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;

  printf("<<ls command>>\r\n");

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++) {
    ptr_string = sl_cli_get_argument_string(arguments, i);
  }
  printf("%s\r\n", ptr_string);

  fs_ls_cmd_f();

}

/***************************************************************************//**
 * Callback for rm (remove the file)
 *
 * This function is used as a callback when the rm command is called
 * in the cli. It simply remove the provided file
 ******************************************************************************/
void cli_rm_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;

  printf("<<rm command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: rm filename\n");
    return;
  }

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
  {
        ptr_string = sl_cli_get_argument_string(arguments, i);
  }
  printf("fn = %s\r\n",ptr_string);

  fs_rm_cmd_f(ptr_string);  // define in mod_fatfs_chan.c

}

/***************************************************************************//**
 * Callback for mkdir (make a new directory)
 *
 * This function is used as a callback when the mkdir command is called
 * in the cli. It simply make a new directory
 ******************************************************************************/

void cli_mkdir_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *dir_name;

  printf("<<mkdir command>>\r\n");
  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: mkdir dirname\n");
    return;
  }

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
  {
      dir_name = sl_cli_get_argument_string(arguments, i);
  }
  printf("dirname = %s\r\n",dir_name);

  if (dir_name == NULL)
  {
    printf("usage: mkdir <dirname>");
    return;
  }
  fs_mkdir_cmd_f(dir_name);
}

/***************************************************************************//**
 * Callback for mkdir (make a new directory)
 *
 * This function is used as a callback when the mkdir command is called
 * in the cli. It simply make a new directory
 ******************************************************************************/

void cli_mv_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *old_fn;
  char *new_fn;

  printf("<<mv command>>\r\n");
  if (sl_cli_get_argument_count(arguments)!=2)
  {
    printf("usage: mv old_filename new_filename\n");
    return;
  }

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
  {
      old_fn = sl_cli_get_argument_string(arguments, 0);
      new_fn = sl_cli_get_argument_string(arguments, 1);
  }
  printf("old_fname = %s, new_fname = %s\r\n",old_fn, new_fn);

  fs_mv_cmd_f(old_fn, new_fn);
}
/***************************************************************************//**
 * Callback for a2d start
 *
 * This function is used as a callback when the rm command is called
 * in the cli. It simply remove the provided file
 ******************************************************************************/
/*
void a2d_start_cmd_f (sl_cli_command_arg_t *arguments)
{
  char *ptr_string;

  printf("<<a2d_start command>>\r\n");

  if (sl_cli_get_argument_count(arguments)>1)
  {
    printf("usage: a2d_start\n");
    return;
  }

  // Read all the arguments provided as strings and print them back
  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
  {
        ptr_string = sl_cli_get_argument_string(arguments, i);
  }
  printf("command = %s\r\n",ptr_string);

//  fs_a2d_start_cmd_f(ptr_string);  // define in mod_fatfs_chan.c

}
*/
/***************************************************************************//**
 * Callback for a2d start
 *
 * This function is used as a callback when the rm command is called
 * in the cli. It simply remove the provided file
 ******************************************************************************/
/*
void a2d_stop_cmd_f (sl_cli_command_arg_t *arguments)
{
//  char *ptr_string;
//
//  printf("<<a2d_stop command>>\r\n");
//
//  if (sl_cli_get_argument_count(arguments)>1)
//  {
//    printf("usage: a2d_stop\n");
//    return;
//  }
//
//  // Read all the arguments provided as strings and print them back
//  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
//  {
//        ptr_string = sl_cli_get_argument_string(arguments, i);
//  }
//  printf("command = %s\r\n",ptr_string);

//  fs_a2d_stop_cmd_f(ptr_string);  // define in mod_fatfs_chan.c

}
*/
/***************************************************************************//**
 * Callback for lw profiling
 *
 * This function is used as a callback when the profile command is called
 * in the cli.
 ******************************************************************************/
void lw_prof_cmd_f (sl_cli_command_arg_t *arguments)
{
  int32_t fsm_type; //set sbe-based or hall-based state machine to active
  int32_t cli_in_lower_limit; //profile lower limit
  int32_t cli_in_upper_limit; //profile upper limit

//  int state_machine_select; //global,toggles state machine in the control loop
//  int prof_upper_limit; //global,perhaps use float type for more precision
//  int prof_lower_limit; //global,these limits bound the profiling mission

  printf("<<lw profiling command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=3)
  {
    printf("usage: profile mode (0 for sbe, 1 for odrive), upper limit (m), lower limit (m), velocity (m/s) \r\n");
    return;
  }

//  // Read all the arguments provided as strings and print them back
//  for (int i = 0; i < sl_cli_get_argument_count(arguments); i++)
//  {
//        ptr_string = sl_cli_get_argument_string(arguments, i);
//  }
//  printf("command = %s\r\n",ptr_string);

  fsm_type = sl_cli_get_argument_int32(arguments,0);
  cli_in_upper_limit = sl_cli_get_argument_int32(arguments,1);
  cli_in_lower_limit = sl_cli_get_argument_int32(arguments,2);

  // transform m/s cli input reference velocity into rps



  //simply assign the inputs to their global counterparts
  state_machine_select = (int)fsm_type;

  /*
   * All upper and lower limits will be in units of meters.
   *
   * CTD pressure data is currently recorded in centibars, which originates from
   * the sbe_hex_to_binary_f function in task.c
   *
   * The control task controls direction by directly comparing prof_upper_limit and prof_lower_limit to either
   * distance or pressure
   *
   * Therefore, we must convert the input from meters to centibars or centimeters
   */
  if(fsm_type == sbe49_profiling){ //convert input in meters to depth in centibar
      prof_upper_limit = (int)(cli_in_upper_limit*10);
      prof_lower_limit = (int)(cli_in_lower_limit*10);

      upper_limit_display = cli_in_upper_limit;
      lower_limit_display = cli_in_lower_limit;
  }
  else if(fsm_type == encoder_profiling){ //convert input in meters to depth in centimeters
      prof_upper_limit = (int)(cli_in_upper_limit*100);
      prof_lower_limit = (int)(cli_in_lower_limit*100);

      upper_limit_display = cli_in_upper_limit;
      lower_limit_display = cli_in_lower_limit;
  }
  else{
      prof_upper_limit = (int)cli_in_upper_limit;
      prof_lower_limit = (int)cli_in_lower_limit;

      upper_limit_display = cli_in_upper_limit;
      lower_limit_display = cli_in_lower_limit;
  }

  printf("arg0 = %d, arg1 = %d, arg2 = %d\r\n", state_machine_select, cli_in_upper_limit, cli_in_lower_limit);
}

/***************************************************************************//**
 * Callback for lw hover
 *
 * This function is used as a callback when the profile command is called
 * in the cli.
 ******************************************************************************/
void lw_hover_cmd_f (sl_cli_command_arg_t *arguments)
{
  int32_t target_depth;

  printf("<<lw hover command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: hover, enter a number for target depth (decibars)\n");
    return;
  }

  target_depth = sl_cli_get_argument_int32(arguments,0);
  prof_upper_limit = (int)target_depth;
  prof_lower_limit = (int)target_depth; //hover command analogous to profiling in 1 spot


}

/***************************************************************************//**
 * Callback for lw hover
 *
 * This function is used as a callback when the profile command is called
 * in the cli.
 ******************************************************************************/
void lw_pwm_cmd_f (sl_cli_command_arg_t *arguments)
{

  uint8_t cli_pwm_in = 0;
  printf("<<lw pwm command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: pwm, enter an integer between 0 and 100 for pwm duty cycle (m)\n");
    return;
  }

  cli_pwm_in = sl_cli_get_argument_uint8(arguments,0);

  duty_cycle = cli_pwm_in;

  sl_pwm_stop(&sl_pwm_control);

  sl_pwm_set_duty_cycle(&sl_pwm_control, duty_cycle);
  printf("%d\r\n", duty_cycle);

  sl_pwm_start(&sl_pwm_control);

}

/***************************************************************************//**
 * Callback for lw zero dy (sets displacement to zero)
 *
 * This function is used as a callback when the profile command is called
 * in the cli.
 ******************************************************************************/
void lw_zero_dy_cmd_f (sl_cli_command_arg_t *arguments)
{

  printf("<<lw zero hall displacement command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=0)
  {
    printf("usage: zero\n");
    return;
  }

  displacement_origin = displacement_abs;
  total_distance = 0;
  distance_since_calibration = 0;

}

/***************************************************************************//**
 * Callback for write to sd
 *
 * This function is used as a callback when the profile command is called
 * in the cli.
 ******************************************************************************/
void write_to_sd_f (sl_cli_command_arg_t *arguments)
{
  uint8_t cli_sd_in = 0;
  uint8_t cli_new_file_in = 0;

  printf("<<write to sd card command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=2)
  {
    printf("arg1: enter 1 to begin writing, enter 0 to stop writing\r\narg2: enter 1 for new file, enter 0 for same file");
    return;
  }

  cli_sd_in = sl_cli_get_argument_uint8(arguments,0);
  sd_card_state = cli_sd_in;

  cli_new_file_in = sl_cli_get_argument_uint8(arguments,1);
  new_file_state = cli_new_file_in;

  printf("arg0 = %d, arg1 = %d\r\n", sd_card_state, new_file_state);

}

void stream_cmd_f (sl_cli_command_arg_t *arguments)
{
  uint8_t cli_stream_in = 0;

  //printf("<<streaming command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=2)
  {
    printf("arg1: enter 1 to begin streaming, enter 0 to stop streaming. arg2: Stream once per (input) logs\r\n");
    return;
  }

  cli_stream_in = sl_cli_get_argument_uint8(arguments,0);
  streaming_toggle_state = cli_stream_in;

  cli_stream_in = sl_cli_get_argument_uint8(arguments,1);
  streaming_rate_state = cli_stream_in;

  //printf("arg0 = %d\r\n", streaming_toggle_state);
  //printf("arg0 = %d\r\n", streaming_rate_state);

}

void calibrate_f (sl_cli_command_arg_t *arguments)
{
  uint32_t cli_press_in;
  uint32_t cli_calibrate_in;
  uint8_t cli_calibrate_toggle_in;


  if(sl_cli_get_argument_count(arguments)!= 3){
      printf("arg0 = toggle on/off, arg1 = pressure at zero in dbar, arg2 = increment at which encoder displacement will calibrate using SBE49\r\n");
  }

  cli_calibrate_toggle_in = sl_cli_get_argument_uint8(arguments, 0);
  calibrate_state = cli_calibrate_toggle_in;

  cli_press_in = sl_cli_get_argument_uint32(arguments, 1);
  press_at_zero = (int)cli_press_in*10;//convert from dbar to centibar

  cli_calibrate_in = sl_cli_get_argument_uint32(arguments, 2);
  calibration_distance = (int) cli_calibrate_in * 100; //convert from meters to cm


  printf("arg0 = %d, arg1 = %d, arg2 = %d\r\n", calibrate_state, press_at_zero, calibration_distance);


}

void odrive_vel_cmd_f (sl_cli_command_arg_t *arguments)
{
  int32_t target_vel;
  char odrv_cmd[80];

  printf("<<odrive vel command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: vel, enter a number for target speed (mRPS)\n");
    return;
  }

  target_vel = sl_cli_get_argument_int32(arguments,0);

//  sprintf(odrv_cmd, "v 0 %.3f!", ((float)target_vel)/1000);
  printf("<<vel: %.3f>>\r\n",((float)target_vel)/1000/(2*M_PI*WHEEL_RADIUS));
//  sl_iostream_write(sl_iostream_odrive_handle, odrv_cmd, strlen(odrv_cmd));
  set_vel_f((float)target_vel/1000/(2*M_PI*WHEEL_RADIUS));
  reference_vel = (float) target_vel/1000.0/(2*M_PI*WHEEL_RADIUS);

}

void odrive_torque_cmd_f (sl_cli_command_arg_t *arguments)
{
  int32_t target_torque;
  char odrv_cmd[80];

  printf("<<odrive torque command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: torque, enter a number for target torque (Nm)\n");
    return;
  }

  target_torque = sl_cli_get_argument_int32(arguments,0);

  sprintf(odrv_cmd, "c 0 %.3f!", ((float)target_torque)/1000);
  printf("<<torque: %.3f>>\r\n",((float)target_torque)/1000);

  sl_iostream_write(sl_iostream_odrive_handle, odrv_cmd, strlen(odrv_cmd));

}

void odrive_set_gain_cmd_f (sl_cli_command_arg_t *arguments)
{
  int vel_gain;
  int vel_integrator_gain;

  if(sl_cli_get_argument_count(arguments)!= 2)
    {
      printf("usage: arg0 = velocity gain, arg1 = velocity integrator gain");
    }

  vel_gain = sl_cli_get_argument_int32(arguments, 0);
  vel_integrator_gain = sl_cli_get_argument_int32(arguments, 1);

  //inputs to set_gain functions are divided by 1000
  set_vel_integrator_gain_f(vel_integrator_gain);
  set_vel_gain_f(vel_gain);



}


void odrive_control_mode_cmd_f (sl_cli_command_arg_t *arguments)
{
  char odrv_control_mode_cmd[80];
  int control_mode_cli_in;

  if(sl_cli_get_argument_count(arguments) != 1){
      printf("usage: 1 for idle, 8 for closed-loop control");
  }

  control_mode_cli_in = (int)sl_cli_get_argument_uint8(arguments, 0);
  set_control_mode_f(control_mode_cli_in);

//  sprintf(odrv_control_mode_cmd, "w axis0.requested_state %s!", control_mode_cli_in);
//  sl_iostream_write(sl_iostream_odrive_handle, odrv_control_mode_cmd,strlen(odrv_control_mode_cmd));
}

void odrive_read_cmd_f (sl_cli_command_arg_t *arguments)
{
  char* target_parameter;

  //pause reading in other places (Odrive_Daqa_Task)
  //OSTaskSuspend(&Odrive_Daqa_TaskTCB, &err);

  printf("<<odrive read command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=1)
  {
    printf("usage: odrvread, enter a target parameter\n");
    return;
  }

  target_parameter = sl_cli_get_argument_string(arguments,0);

  sprintf(custom_odrive_command, "r %s!", target_parameter);
  printf("<<reading: %s>>\r\n", target_parameter);
  printf("<<sending: %s>>\r\n", custom_odrive_command);

  run_custom_odrive_command = true;
}

void odrive_write_cmd_f (sl_cli_command_arg_t *arguments)
{
  char odrv_cmd[80];
  char* target_parameter;
  char* target_value;

  printf("<<odrive write command>>\r\n");

  if (sl_cli_get_argument_count(arguments)!=2)
  {
    printf("usage: odrvwrite, enter a target parameter, enter its new value\n");
    return;
  }

  target_parameter = sl_cli_get_argument_string(arguments,0);
  target_value = sl_cli_get_argument_string(arguments,1);

  sprintf(odrv_cmd, "w %s %s!", target_parameter, target_value);
  printf("<<writing: %s to %s>>\r\n", target_value, target_parameter);
  printf("<<sending: %s>>\r\n", odrv_cmd);

  sl_iostream_write(sl_iostream_odrive_handle, odrv_cmd, strlen(odrv_cmd));
}

//Time synchronization - gschellenberg 5/30/24

void set_time_cmd_f (sl_cli_command_arg_t *arguments)
{
  char* laptop_time;
    uint64_t tick_cnt = 0;
    uint64_t timestamp;
    tick_cnt = sl_sleeptimer_get_tick_count64();  // get tick count
    sl_sleeptimer_tick64_to_ms(tick_cnt, &timestamp);  // get timestamp
    if (sl_cli_get_argument_count(arguments)!=1)
    {
      printf("usage: timeset, enter the current time in UNIX ms\n");
      return;
    }
    laptop_time = sl_cli_get_argument_string(arguments,0);
    time_offset = strtoull(laptop_time,NULL,10)-timestamp;
    tick_cnt = sl_sleeptimer_get_tick_count64();  // get tick count
    sl_sleeptimer_tick64_to_ms(tick_cnt, &timestamp);  // get timestamp
    printf("Set time\r\n");
}

/***************************************************************************//**
 * bootload_uart
 *
 * bootload from uart terminal command and xmodem file transfer
 ******************************************************************************/
void echo_str(sl_cli_command_arg_t *arguments)
{
  bootloader_init();
  bootloader_eraseStorageSlot(0); // Have to erase the slot in order to enter uart mode if slot preoccupied.
  bootloader_rebootAndInstall();
}
#define MAX_METADATA_LENGTH   512
uint8_t metadata[MAX_METADATA_LENGTH];


void echo_int(sl_cli_command_arg_t *arguments)
{
  int32_t flag; //flag for bootloader
  BootloaderStorageSlot_t info_s; // info pointer for bootloader
  FRESULT retval; //return value for FAT FS
  uint32_t remain_bytes = 51200; //Byte Upper Limit for file

  memset(buffer_t,0,51200); //initialize the buffer with 0
  char file_name[] = "MOD_API_lw_beta_dev_2023_011_08.gbl";//"new_checker.gbl"; // name of the file
  retval = f_open (&superfile, file_name,
                            FA_READ); //open the file
  if(retval!=FR_OK){
      printf("ERROR!!!ERROR!!!\n"); //Error Opening the file
  }
  printf("retval: %d\n",retval);
  int i=0;
  while(remain_bytes>0){//Read the file incrementally as FAT-FS can't handle all at once
      retval = f_read(&superfile,buffer_t+64*i,64,&bytes_read);

      total_bytes+=bytes_read;
      remain_bytes-=bytes_read;
      if(bytes_read==0){
          break;
      }
      i++;
  }
  if(retval!=FR_OK){
      printf("ERROR!!!ERROR!!!\n");
  }
  printf("bytes_read: %d\n",total_bytes);//Check with size of file
  flag=bootloader_init(); //init the bootloader
  printf("Init: %d\n",flag);


  bootloader_getStorageSlotInfo(0,&info_s); //get the storage info !not important

  flag=bootloader_eraseWriteStorage(0,0,buffer_t,total_bytes); // Erase the storage and start writing all into storage
  printf("Write Storage: %d\n",flag);

  if(bootloader_storageIsBusy()){
      osDelay(10);  //make sure it is done
  }
  bootloader_getStorageSlotInfo(0,&info_s); // Get the storage info !not important

   flag = bootloader_verifyImage(0,NULL); // Verify the image in the storage
   printf("Verify Image: %d\n",flag);


   bool flag_v = bootloader_verifyApplication(0x100000); // verify app
   printf("Verify App: %d\n",flag_v);


   flag=bootloader_setImageToBootload(0); // Select slot 0 as the place to run and reboot.
   printf("Set Image: %d\n",flag);





   osDelay(10);
  bootloader_rebootAndInstall();//reboot and start the new program.
}


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/*******************************************************************************
 * Initialize cli example.
 ******************************************************************************/

bool cli_app_init(void)
{
  bool status = 0;
  int retval = 0;

  status = sl_cli_command_add_command_group(sl_cli_inst_handle, command_group);
  EFM_ASSERT(status);


  return retval;
}
