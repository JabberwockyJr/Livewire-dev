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

#include "app.h"
#include "rtos_err.h"
#include "lib_mem.h"
#include "pwm.h"

 main_app_struct_ptr app_struct_ptr;


/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
int app_init(void)
{
  int retval = 0;
  bool status = 0;
  RTOS_ERR p_err;

//  volatile int i = 0;
//  for (i = 0; i < 10000000;i++);
//  i = 0;
  printf("\r\nStarted MOD cli fatfs micrium OS... ver: 18 Oct 2022\r\n\r\n");



//  // adding initialize for fatfs - mb 12 Sept 2022
  retval = fs_bsp_init();
//  if (retval) printf("failed initialize fatfs from app_init()\n");
//  printf("\r\n Done initialize bsp\r\n");
//
  //allocate memory for the main data struct .. which isn't currently used
  app_struct_ptr = (main_app_struct*)Mem_SegAlloc("LW Data Ptr",DEF_NULL,sizeof(main_app_struct),&p_err);
//  if ((app_struct_ptr = (main_app_struct*)malloc(sizeof(main_app_struct))) == NULL){
//  printf("Failed to allocate the main structure\r\n");
  //info: https://docs.silabs.com/micrium/latest/micrium-general-concepts/04-memory-segments-and-lib-heap

  //initialize the cli
  status = cli_app_init();

  //create the text file to store data
  file_setup();

  if(LOGGING_TASK)
    logging_task_create(&app_struct_ptr->logging_struct);
  control_task_create();
  //hall_displacement_task_create();
  odrive_daqa_task_create();
  pwm_init();

  sbe_acq_task_create(&app_struct_ptr->sbe_struct);

  if(status)
    printf("failed initialize cli_app_init() from app_init()\n");
  printf("\r\nDone initialize cli\r\n");

    return retval;
}

int app_cleanup(void)
{
  int retval = 0;
  if (app_struct_ptr)
    {
      free(app_struct_ptr);
      app_struct_ptr = NULL;
    }
  return retval;

}
