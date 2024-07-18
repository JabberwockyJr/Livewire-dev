/***************************************************************************//**
 * @file main.c
 * @brief main() function.
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
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#include "os.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT
#include "unistd.h"

int main(void) {
	int retval = 0;
	RTOS_ERR err;
	// Initialize Silicon Labs device, system, service(s) and protocol stack(s).
	// Note that if the kernel is present, processing task(s) will be created by
	// this call.
	sl_system_init();

	//gschellenberg 6/5/2024 - need this delay, something to do with SD card bootup power?
	//for loop count worked, time delay doesnt???? - kernal uninitilized
	//OSTimeDlyHMSM ( 0,0,10,0, OS_OPT_TIME_DLY, &err);
	uint64_t current_time = OSTimeGet(&err);
	while (OSTimeGet(&err) < (current_time + 4000));

	// Initialize the application. For example, create periodic timer(s) or
	// task(s) if the kernel is present.
	retval = app_init();
	if (retval)
		printf("Failed app_init() in main\r\n");

#if defined(SL_CATALOG_KERNEL_PRESENT)
	// Start the kernel. Task(s ) created in app_init() will start running.
	sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action();

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}
