/*
 * task.c
 *
 *  Created on: Sep 12, 2022
 *      Author: maibui
 */
#include <string.h>
#include <stdio.h>
#include "cli.h"
#include "sl_cli.h"
#include "sl_cli_instances.h"
#include "sl_cli_arguments.h"
#include "sl_cli_handles.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_iostream.h"
#include "sl_iostream_init_instances.h"
#include "sl_iostream_handles.h"
#include "os.h"
#include "ff.h"
#include "sdio.h"
#include "diskio.h"
#include "stdbool.h"
#include "math.h"
#include "sl_emlib_gpio_init_control_pin_1_config.h"
#include "em_gpio.h"
#include "task.h"

// maibui - 12 Oct 2022: adding adc
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_usart.h"

//modifying control loop to have pwm - ebohannon - 11 April 2023
#include "pwm.h"

// integrating odrive communication - mbarrera - 17 August 2023
#include "odrive_comms.h"

// DMA to read from SBE49
#include "dmadrv.h"

// adding INA219 current monitoring board - mbaarrera 24 May 2024
#include "adafruit_INA219.h"

// for adc - maibui - 12 Oct 2022
#define adcFreq   16000000
volatile uint32_t adc_sample;
volatile int millivolts;

volatile uint32_t adc1_sample;
volatile int battery_voltage;
float battery_voltage_sample;

//global lw data struct vars - ebohannon - 24 Oct 2022
//static char sbe_str[125] = "\0";
static main_lw_struct lw_data;
static int indx;
static int read_indx;
static FIL superfile;

odrive_data_t consumption_odrive_data;
odrive_data_t production_odrive_data;

enum profiling_state prof_state;
enum state_machines fsm_type;
enum ctd_sync_state sync_state;

//starting file_name - mbarrera - 27 July 2023
char file_name[] = "LW_data_000000.txt";
uint8_t num_file = 0; //file tracking
int current_press;

//Odrive transparent link - gschellenberg 5/29/24
#include "stdbool.h"
bool run_custom_odrive_command = false;
char custom_odrive_command[80] = "\0";

OS_TCB Logging_TaskTCB; /*   Task Control Block.   */
CPU_STK Logging_TaskStk[APP_EXAMPLE_TASK_STK_SIZE]; /*   Stack.                */

//Does the streaming task exist????
OS_TCB Streaming_TaskTCB; /*   Task Control Block.   */
CPU_STK Streaming_TaskStk[APP_EXAMPLE_TASK_STK_SIZE]; /*   Stack.                */

OS_TCB Sbe_TaskTCB; /*   Task Control Block.   */
CPU_STK Sbe_TaskStk[SBE_TASK_STK_SIZE]; /*   Stack.                */

OS_TCB Control_TaskTCB; /*   Task Control Block.   */
CPU_STK Control_TaskStk[CONTROL_TASK_STK_SIZE]; /*   Stack.                */

OS_TCB Odrive_Daqa_TaskTCB; /*   Task Control Block.   */
CPU_STK Odrive_Daqa_TaskStk[CONTROL_TASK_STK_SIZE]; /*   Stack.                */

char logging_hex1[17]; //time // Assuming 32-bit integers, so each integer will be at most 8 characters in hexadecimal representation
char logging_hex2[9]; //pres
char logging_hex3[9]; //disp
char logging_hex4[9]; //vel
char logging_hex5[9]; //cur
char logging_hex6[9]; //volt
char logging_hex7[9]; //err
char logging_hex8[2]; //state
char logging_hex9[2]; //mode
char logging_hex10[9]; //velp
char logging_hex11[9]; //torquep
char logging_hex12[9]; //velg
char logging_hex13[9]; //velig
char logging_hex14[9]; //upper limit
char logging_hex15[9]; //lower limit
char logging_hex16[2]; //profiling state


/* Example Task Code:      */

void initADC0(void) {
	// Enable ADC0 clock
	CMU_ClockEnable(cmuClock_ADC0, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

//  initSingle.diff       = false;        // single ended
	initSingle.diff = true;        // differential
	initSingle.reference = adcRef5V;    // internal 5V reference
	initSingle.resolution = adcRes12Bit;  // 12-bit resolution
	initSingle.acqTime = adcAcqTime4; // set acquisition time to meet minimum requirement

	// Select ADC input.
	initSingle.posSel = adcPosSelAPORT1XCH10; //PA10
	initSingle.negSel = adcNegSelAPORT1YCH11; //PA11
	init.timebase = ADC_TimebaseCalc(0);

	//LW uses PA10 and PA11 for diff. ADC across load resistor

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &initSingle);
}

void initADC1(void) {
	// Enable ADC1 clock
	CMU_ClockEnable(cmuClock_ADC1, true);

	// Declare init structs
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	// Modify init structs and initialize
	init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

//  initSingle.diff       = false;        // single ended
	initSingle.diff = true;        // differential
	initSingle.reference = adcRef5V;    // internal 5V reference
	initSingle.resolution = adcRes12Bit;  // 12-bit resolution
	initSingle.acqTime = adcAcqTime4; // set acquisition time to meet minimum requirement

	// Select ADC input.
	initSingle.posSel = adcPosSelAPORT4XCH13; //PE13
	initSingle.negSel = adcNegSelAPORT4YCH12; //PE12
	init.timebase = ADC_TimebaseCalc(0);

	ADC_Init(ADC1, &init);
	ADC_InitSingle(ADC1, &initSingle);
}

void logging_task_create(main_logging_struct *logging_struct_ptr) {
	RTOS_ERR err;

	OSTaskCreate(&Logging_TaskTCB, /* Pointer to the task's TCB.  */
	"Logging Task.", /* Name to help debugging.     */
	&logging_task, /* Pointer to the task's code. */
	logging_struct_ptr, /* Pointer to task's argument. */
	LOGGING_TASK_PRIO, /* Task's priority.            */
	&Logging_TaskStk[0], /* Pointer to base of stack.   */
	(LOGGING_TASK_STK_SIZE / 10u), /* Stack limit, from base.     */
	LOGGING_TASK_STK_SIZE, /* Stack size, in CPU_STK.     */
	0u, /* Messages in task queue.     */
	0u, /* Round-Robin time quanta.    */
	DEF_NULL, /* External TCB data.          */
	OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, /* Task options.               */
	&err);
	if (err.Code != RTOS_ERR_NONE) {
		/* Handle error on task creation. */
	}

}

void logging_task(void *p_arg) {
	OS_RATE_HZ os_tick_rate;
	RTOS_ERR p_err;

	int streaming_startup_delay_count = 0;
	int retval = 0;

	uint64_t write_check = 0;
	uint64_t prev_write_cnt = 0;
	int log_state = 0;
	os_tick_rate = OSTimeTickRateHzGet(&p_err);
	RTOS_ERR err;

	int timestamp = 0;
//  int current_timestamp;
//  int last_timestamp;
//  float hall_effects = 0;
//  int dec_temp = 0;
//  int dec_con = 0;
	int dec_press = 0;
//  int dec_tpc = 0;
	int adc_data;
	char raw_ctd_data[24] = "\0";
	int raw_ctd_data_indx;
//  int displacement;
//  int velocity;
//  int odrive_current;

	int total_bytes_written = 0; //initial value here from file_setup() header
	unsigned int write_bytes_written = 0;
	float pwr_gen = 0.0;

	int ticks_since_last_data_out = 0;

//  int int_dec_press;
//  int dec_dec_press;
//  int int_adc_data;
//  int dec_adc_data;
//  int int_pwr_gen;
//  int dec_pwr_gen;
//  int int_battery_voltage;
//  int dec_battery_voltage;
//
//  int int_odrive_voltage;
//  int dec_odrive_voltage;
//  int int_odrive_current;
//  int dec_odrive_current;
//
//  char data_streaming_string[DATA_OUT_SIZE+1]="\0";
	char data_logging_string[LOG_BUFF_SIZE] = "\0";

//
	main_logging_struct *logging_ptr;

	/* Use argument. */
//
	logging_ptr = p_arg;
	logging_ptr->logging_curr_state = 1;
	log_state = 1;

	//LOGGING TASK
	while (DEF_TRUE)
//   while(!storing_ptr->storing_task_done)
	{
		/* switch statement to run through the 3 steps to log and stream lw data
		 * case 1: check lw data struct write count -> if it has incremented, there's new data
		 *         retrieve encoded and decoded lw data elements from their respective
		 *         buffers within the main lw data struct
		 * case 2: write encoded lw data to the sd card
		 * case 3: stream decoded data out
		 */

		//2023 Jan 23 eb *need to pad with zeros to keep the $LWG length consistent
		//^ padding in the snprintf lines using "%0(length)d"
//       switch(logging_ptr->logging_curr_state)
		switch (log_state) {
//       case init:
		//2022 11 16 eb wrote & tested case 1 & 2 to check+grab new lw data and stream decoded data
		case 1:
//           printf("logging_task is in \"init\" state, count %d\r\n",logging_run_count);
//           OSTimeDly (os_tick_rate/INIT_RATE, OS_OPT_TIME_PERIODIC,&p_err);
//           OSTimeDly (os_tick_rate/INIT_RATE, OS_OPT_TIME_PERIODIC,&p_err);
			write_check = lw_data.lw_cir_buf_write_cnt;
			read_indx = lw_data.lw_cir_buf_log_read_cnt % LW_CIR_BUFF_SIZE; // get the current indx to read

			if (write_check > lw_data.lw_cir_buf_log_read_cnt) {
				if (streaming_toggle_state || (sd_card_state == writing)) {
					//               printf("stream & log test indx: %d\r\n",indx); //this print works
					//gathering all data for streaming out from circular buffer
					timestamp =
							lw_data.livewire_decode_cir_buff[read_indx].timestamp; //timestamp in milliseconds
					dec_press =
							lw_data.livewire_decode_cir_buff[read_indx].press; //decibars

					adc_data =
							lw_data.livewire_decode_cir_buff[read_indx].adc_data; //adc_data in millivolts
					pwr_gen = (adc_data * adc_data) / (4.3 * 1000.0); //power in mW generated over 4.3 Ohm load

					//                displacement = lw_data.odrive_cir_buff[read_indx].displacement;
//					consumption_odrive_data.velocity =
//							lw_data.odrive_cir_buff[read_indx].velocity;
//					consumption_odrive_data.voltage =
//							lw_data.odrive_cir_buff[read_indx].voltage;
//					consumption_odrive_data.current =
//							lw_data.odrive_cir_buff[read_indx].current;
//					consumption_odrive_data.current_errors =
//							lw_data.odrive_cir_buff[read_indx].current_errors;
//					consumption_odrive_data.current_state =
//							lw_data.odrive_cir_buff[read_indx].current_state;
//					consumption_odrive_data.control_mode =
//							lw_data.odrive_cir_buff[read_indx].control_mode;
//					consumption_odrive_data.vel_set_point =
//							lw_data.odrive_cir_buff[read_indx].vel_set_point;
//					consumption_odrive_data.torque_set_point =
//							lw_data.odrive_cir_buff[read_indx].torque_set_point;
//					consumption_odrive_data.vel_gain =
//							lw_data.odrive_cir_buff[read_indx].vel_gain;
//					consumption_odrive_data.vel_integrator_gain =
//							lw_data.odrive_cir_buff[read_indx].vel_integrator_gain;

					//mbarrera - 26 July 2023: copying ctd data from circular buffer
					raw_ctd_data_indx = 0;
					while (raw_ctd_data_indx < SBE_DATA_LENGTH) {
						raw_ctd_data[raw_ctd_data_indx] =
								lw_data.livewire_cir_buff[read_indx].ctd_data[raw_ctd_data_indx];
						raw_ctd_data_indx++;
					}
					int pres = (int) (dec_press * 100);
					assemble_output_f(data_logging_string, timestamp+time_offset,
							raw_ctd_data, pres,
							lw_data.odrive_cir_buff[read_indx]);
					//data_logging_string = "$LWG0x0000000000000000 0x05D9290A889107FDAC416C 0x0000000000 0x0000000000 0x0000000000 0x0000000000 0x0000000000 0x0000000000 0x0000000000 0x00000000 0x0 0x0 0x0000000000 0x0000000000 0x0000000000 0x0000000000\n";
//					snprintf(data_logging_string,LOG_BUFF_SIZE+1,
//							"$LWG0x%016llX 0x%s 0x%010X 0x%010X 0x%010X 0x%010X 0x%010X 0x%010X 0x%010X 0x%08X 0x%01X 0x%01X 0x%010X 0x%010X 0x%010X 0x%010X\r\n", 0,"05D9290A889107FDAC416C",0,0,0,0,0,0,0,0,0,0,0,0,0,0);
//							timestamp,         //ms
//							raw_ctd_data, displacement_rel * 100,       //cm*100
//							pres,              //mbar *100
//							(int) (pwr_gen),    //mW
//							adc_data,  //mV
//							consumption_odrive_data.velocity, //turns per second *1000
//							consumption_odrive_data.voltage,    //volts*1000
//							consumption_odrive_data.current, //amps*1000
//							consumption_odrive_data.current_errors,
//							consumption_odrive_data.current_state,
//							consumption_odrive_data.control_mode,
//							consumption_odrive_data.vel_set_point, //rps *1000
//							consumption_odrive_data.torque_set_point, //nm *1000
//							consumption_odrive_data.vel_gain, //vel_gain *1000
//							consumption_odrive_data.vel_integrator_gain);
				}
				lw_data.lw_cir_buf_log_read_cnt++;
				log_state = 2;
				break;
			} else {
				log_state = 1;
				break; //remember it breaks out!!
			}
			// gschellenberg 6/5/2024 - modify logging task to get all data to sd card rather then 1/8
		case 2:
			if (streaming_toggle_state) {
				if (ticks_since_last_data_out >= streaming_rate_state) {
					printf(data_logging_string);
					ticks_since_last_data_out = 0;
				} else {
					ticks_since_last_data_out++;
				}
			}
			//reduces the amount of times data streams out(1 stream per x iterations)

			log_state = 3;
			break;
		case 3:
			//data file should still be open, so want to check lw_data for new entries to log
			//^ turns out this was untrue, before adding f_open in case 3, got error 9: FR_INVALID_OBJECT
			//error 9 most likely returned because the file was closed between file_setup() in app.c and this case
			//1 August 2023 - mbarrera: ^turns out this was untrue lol we removed f_open and it works fine?
			//what if the logging index is the same as the seabird write index
			//log_indx = (indx-1)%LW_CIR_BUFF_SIZE?

			//mbarrera - 24 August 2023: only write to sd card if timestamp has changed
//           current_timestamp = timestamp;
			//if(current_timestamp > last_timestamp){

			// mbarrera - 31 July 2023: added command to toggle on and off writing to sd card
			if (sd_card_state == writing) {
				//mbarrera - 2 August 2023: modified sd card data to be in hexadeciaml format
				//fixed point int representation, * 100 for two decimal point precision

				//2023 01 12 eb finished integrating working code to write data_entry to sd card
				retval = f_lseek(&superfile, total_bytes_written + 1); //set the pointer to just after the last byte written on the file
				if (retval != FR_OK) {
					printf("Failed to lseek %s, error %u\n", file_name, retval);
				}

				retval = f_write(&superfile, data_logging_string, LOG_BUFF_SIZE,
						&write_bytes_written);
				if (retval != FR_OK) {
					printf("Failed to write %s, error %u\n", file_name, retval);
				}

				total_bytes_written += write_bytes_written + 1; //want to track byte position in file to set lseek later
				//adding 1 to account for the /0 character
				retval = f_sync(&superfile); //TODO: do not sync so often, best to
				//sync when the buffers are closer to full rather than syncing a few bytes
				if (retval != FR_OK) {
					printf("Failed to sync %s, error %u\n", file_name, retval);
				}
			}
			logging_ptr->logging_curr_state = init;

			//1 August 2023 - mbarrera: option to write to new file with write_sd command
			if (new_file_state == new_file) {
				f_close(&superfile);

				//update file name
				//using while loop and f_stat to detect file name here takes too much resources
				//instead just iterate num_file up by one
				num_file++;
				sprintf(file_name, "LW_data_%d.txt", num_file);
				printf("\nwriting to file name: %s \n", file_name);

				//create, open, and append new file
				retval = f_open(&superfile, file_name,
				FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

				retval = f_open(&superfile, file_name,
				FA_OPEN_APPEND | FA_WRITE | FA_READ);

				if (retval != FR_OK) {
					printf("Failed to open %s, error %u\n", file_name, retval);
				}

				total_bytes_written = 0;
				new_file_state = same_file;
			}

			//}
			log_state = 1;
			break;
		}

		OSTimeDly(5,    //   consumer delay is #define at the beginning OS Ticks
				OS_OPT_TIME_DLY,          //   from now.
				&err);
		//   Check error code.
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

	}
}

//maibui - 4 Oct 2022

void sbe_acq_task_create(main_sbe_struct *sbe_struct_ptr) {
	RTOS_ERR err;

	OSTaskCreate(&Sbe_TaskTCB,                // Pointer to the task's TCB.
			"SBE Task.",                    // Name to help debugging.
			sbe_acq_f,                   // Pointer to the task's code.
			sbe_struct_ptr,                       // Pointer to task's argument.
			SBE_TASK_PRIO,             // Task's priority.
			&Sbe_TaskStk[0],             // Pointer to base of stack.
			(SBE_TASK_STK_SIZE / 10u),  // Stack limit, from base.
			SBE_TASK_STK_SIZE,         // Stack size, in CPU_STK.
			0u,                               // Messages in task queue.
			0u,                                // Round-Robin time quanta.
			DEF_NULL,                          //External TCB data.
			OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,          // Task options.
			&err);
	if (err.Code != RTOS_ERR_NONE) {
		// Handle error on task creation.
	}
}

//DMADRV variables
unsigned int chanid;
volatile int flag_dma_1, flag_dma_2 = 0;
/* Acquire SBE task
 * Description: Sync to CTD data stream, decode data, store data in circular buffer
 */
void sbe_acq_f(void *p_arg) {
//    int ret_val = 0;
	main_sbe_struct *sbe_ptr;
	//add var descriptions
	sbe_ptr = p_arg;
	sl_status_t status;

	int timestamp = 0;
//    uint64_t timestamp = 0; // when change timestamp to uint64_t -> hang
	uint64_t tick_cnt = 0;
	uint16_t averaging_width = 10;
	uint16_t averaging_count = 1;

	sbe_ptr->sbe_acq_data.acq_state = 0;
	float hall_data;
	int sbe_data_index = 0;

	RTOS_ERR err;

	initADC0();
	initADC1();

	//DMA variables
	DMADRV_Init(); //initialize DMADRV
	char sbe_data[SBE_BUFFER_SIZE + 1];
	sync_state = search_for_sync;

	//ACQUIRE TASK
	while (DEF_TRUE) {
		read_sbe_f(sbe_data); //read single data string from sbe

//        if(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

		// Get ADC result
		adc_sample = ADC_DataSingleGet(ADC0);
		// Start ADC conversion
		ADC_Start(ADC0, adcStartSingle);

		//initialize and start ADC battery monitoring
		adc1_sample = ADC_DataSingleGet(ADC1);
		ADC_Start(ADC1, adcStartSingle);

		//write index for decode and for the encoded data will be the same
		indx = lw_data.lw_cir_buf_write_cnt % LW_CIR_BUFF_SIZE; // get the current indx to write

		tick_cnt = sl_sleeptimer_get_tick_count64();  // get tick count
		status = sl_sleeptimer_tick64_to_ms(tick_cnt, &timestamp); // get timestamp

		lw_data.livewire_cir_buff[indx].timestamp = timestamp;
		lw_data.livewire_decode_cir_buff[indx].timestamp = timestamp;

		lw_data.odrive_cir_buff[indx].displacement = production_odrive_data.displacement;
		lw_data.odrive_cir_buff[indx].velocity = production_odrive_data.velocity;
		lw_data.odrive_cir_buff[indx].direction = production_odrive_data.direction;
		lw_data.odrive_cir_buff[indx].voltage = production_odrive_data.voltage;
		lw_data.odrive_cir_buff[indx].current = production_odrive_data.current;
		lw_data.odrive_cir_buff[indx].current_errors = production_odrive_data.current_errors;
		lw_data.odrive_cir_buff[indx].current_state = production_odrive_data.current_state;
		lw_data.odrive_cir_buff[indx].control_mode = production_odrive_data.control_mode;
		lw_data.odrive_cir_buff[indx].vel_set_point = production_odrive_data.vel_set_point;
		lw_data.odrive_cir_buff[indx].torque_set_point =
				production_odrive_data.torque_set_point;
		lw_data.odrive_cir_buff[indx].vel_gain = production_odrive_data.vel_gain;
		lw_data.odrive_cir_buff[indx].vel_integrator_gain =
				production_odrive_data.vel_integrator_gain;

		//v write sbe data to the lw_data circular buffer
		sbe_data_index = 0;
		while (sbe_data_index < SBE_DATA_LENGTH) {
			lw_data.livewire_cir_buff[indx].ctd_data[sbe_data_index] =
					sbe_data[sbe_data_index];
			sbe_data_index++;
		}       //write hall effect and adc data to the lw data circular buffer
		//lw_data.livewire_cir_buff[indx].rpm = hall_data;

		millivolts = (5000.0 / 4096) * 10 * (int) adc_sample; //x10 for voltage divider scaling
//        printf("millivolts set = %u\r\n",millivolts);

		if (averaging_count <= averaging_width) {
			battery_voltage_sample += (5000.0 / 4096) * 10.1
					* (int) adc1_sample; //battery voltage in millivolts
			averaging_count++;

		} else {
			battery_voltage = battery_voltage_sample / averaging_width;
			averaging_count = 1;
			battery_voltage_sample = 0;
		}

//        battery_voltage = (5000.0/4096)*10.1*(int)adc1_sample; //battery voltage in millivolts

		lw_data.livewire_cir_buff[indx].adc_data = millivolts;
		lw_data.livewire_decode_cir_buff[indx].adc_data = millivolts;

		//decode data
		sbe_hex_to_binary_f(sbe_data, indx, &lw_data);
		lw_data.lw_cir_buf_write_cnt++;

		// Delay Start Task execution for
		OSTimeDly(2,    //   consumer delay is #define at the beginning OS Ticks
				OS_OPT_TIME_DLY,          //   from now.
				&err);
		//   Check error code.
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
//    return ret_val;
	} // end of while loop

}

/*
 * Description: control task
 */ //2023 Jan 18 eb setting up control task

void control_task_create(void) {
	RTOS_ERR err;

	OSTaskCreate(&Control_TaskTCB,             // Pointer to the task's TCB.  */
			"Control Task.", /* Name to help debugging.     */
			&control_task, /* Pointer to the task's code. */
			DEF_NULL, /* Pointer to task's argument. */
			CONTROL_TASK_PRIO, /* Task's priority.            */
			&Control_TaskStk[0], /* Pointer to base of stack.   */
			(CONTROL_TASK_STK_SIZE / 10u), /* Stack limit, from base.     */
			CONTROL_TASK_STK_SIZE, /* Stack size, in CPU_STK.     */
			10u, /* Messages in task queue.     */
			0u, /* Round-Robin time quanta.    */
			DEF_NULL, /* External TCB data.          */
			OS_OPT_TASK_STK_CHK, /* Task options.               */
			&err);
	if (err.Code != RTOS_ERR_NONE) {
		printf("Failed to create acq_task\r\n");
		/* Handle error on task creation. */
	}
}
/*
 * Description: control task function
 * This task should check for new data and keep a local history
 * Then, it should make an informed control decision
 *
 */ //2023 Jan 18 eb setting up control task
void control_task(void) {
// static int i=0;
//  OS_RATE_HZ os_tick_rate;
	RTOS_ERR p_err;

	uint64_t write_check = 0;
	uint64_t prev_write_cnt = 0;

	prof_state = profiling_init;

	fsm_type = 0;
	uint32_t dec_press = 0;


//

	/* mbarrera 7/11/24: reshaping state machine structure to
	 * incorporate Odrive-based closed-loop velocity control
	 * and regenerative braking
	 *
	 * using profile command:
	 * fsm_type = 0 --> pressure-based profiling
	 * fsm_type = 1 --> encoder-based profiling
	 * fsm_type = 2 --> manual control
	 *
	 * prof_upper_limit and prof_lower_limit set profiling boundaries
	 *
	 * reference_v = commanded velocity of odrive in m/s
	 *
	 *
	 * if(profiling_down_state){
	 *  reference_v is positive until lower limit
	 *  once we exceed lower limit, change sign of reference_v
	 *  switch to profiling_up_state
	 *  }
	 *
	 *
	 * if(profiling_up_state){
	 *  reference_v is negative until upper limit
	 *  once we exceed upper limit, change sign of reference_v
	 *  switch to profiling_down_state
	 * }
	 */
	while (DEF_TRUE) {
		//the following few lines check for a new LW data string
		//this is to enact control at the sbe49 rate
		write_check = lw_data.lw_cir_buf_write_cnt;
		if (write_check > prev_write_cnt) {

			//if new data, execute loop
			fsm_type = state_machine_select;

			switch(fsm_type){

			  case sbe49_profiling:
          //go use the pressure-based state machine
          //first grab the pressure data, then fsm loop
          dec_press = lw_data.livewire_decode_cir_buff[indx].press;
          //2023 Feb 7 eb sbe49 pressure-based state machine
          /* states are as follows:
           * 0: init state, currently just passes to idle
           * 1: idle state, wait for input
           * 2: profiling down state, go to lower limit, then switch to going up
           * 3: profiling up state, go to upper limit, then switch to going down
           * 4: hover state: hang out within bounding box
           */
          switch (prof_state) {
            case profiling_init:
              prof_state = profiling_idle;
              break;

            case profiling_idle:
              prof_state = profiling_down;
              break;

            case profiling_down:
            if ((int) dec_press > prof_lower_limit) {
              prof_state = profiling_up;
              // send command to flip velocity sign
            }
            break;

          case profiling_up:
            //when the LW has reached the upper limit, switch to profiling_down
            if ((int) dec_press < prof_upper_limit) {
              prof_state = profiling_down;
              // send command to flip velocity sign
            }
            break;

          case profiling_hover:
            //go somewhere and idle
            break;
          }

          break;

				case encoder_profiling:
          //go use the rpm-based state machine
          //fsm loop using global_hall_displacement
          //2023 Feb 1 eb basic state machine
          /* states are as follows:
           * 0: init state, currently just passes to idle
           * 1: idle state, wait for input
           * 2: profiling down state, go to lower limit, then switch to going up
           * 3: profiling up state, go to upper limit, then switch to going down
           * 4: hover state: hang out within bounding box
           */
		         profile_state_machine(displacement_rel);


          break;

				case manual_control:

				  break;
			}
		}
		prev_write_cnt = write_check;
		//just moved time delay into the loop, let's see what it does
		// Delay Start Task execution for
		OSTimeDly(2,    //   consumer delay is #define at the beginning OS Ticks
				OS_OPT_TIME_DLY,          //   from now.
				&p_err);
		//   Check error code.
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(p_err) == RTOS_ERR_NONE), ;);

	}
}

void profile_state_machine(int depth){


  int DEFAULT_VEL_GAIN = 300;
  int DEFAULT_VEL_INTEGRATOR_GAIN = 200;


  switch (prof_state) {
    case profiling_init:


      set_control_mode_f(closed_loop_control);
      set_vel_gain_f(DEFAULT_VEL_GAIN);
      set_vel_integrator_gain_f(DEFAULT_VEL_INTEGRATOR_GAIN);
      prof_state = profiling_down;
      break;

    case profiling_down:
      // when the LW has reached the lower limit, switch to profiling up
      if ( depth > prof_lower_limit) {
        prof_state = profiling_up;
        // send command to flip velocity negative
        set_vel_f(-fabs(reference_vel));
      }
      else{
          set_vel_f(fabs(reference_vel));
      }
      break;

    case profiling_up:
      // go up (negative velocity)
      //when the LW has reached the upper limit, switch to profiling_down
      if ( depth < prof_upper_limit) {
        prof_state = profiling_down;
        // send command to flip velocity positive
        set_vel_f(fabs(reference_vel));
      }
      else{
          set_vel_f(-fabs(reference_vel));
      }
      break;

    case profiling_hover:
      //go somewhere and idle
      //cli command sets upper/lower limits to be equal and runs with the above cases
      break;

    case profiling_idle:
      set_control_mode_f(odrv_idle);
      break;
  }

}

void odrive_daqa_task_create(void) {
	RTOS_ERR err;

	OSTaskCreate(&Odrive_Daqa_TaskTCB,         // Pointer to the task's TCB.  */
			"Odrive Daqa Task.", /* Name to help debugging.     */
			&odrive_daqa_task, /* Pointer to the task's code. */
			DEF_NULL, /* Pointer to task's argument. */
			ODRIVE_DAQA_TASK_PRIO, /* Task's priority.            */
			&Odrive_Daqa_TaskStk[0], /* Pointer to base of stack.   */
			(ODRIVE_DAQA_TASK_STK_SIZE / 10u), /* Stack limit, from base.     */
			ODRIVE_DAQA_TASK_STK_SIZE, /* Stack size, in CPU_STK.     */
			10u, /* Messages in task queue.     */
			0u, /* Round-Robin time quanta.    */
			DEF_NULL, /* External TCB data.          */
			OS_OPT_TASK_STK_CHK, /* Task options.               */
			&err);
	if (err.Code != RTOS_ERR_NONE) {
		printf("Failed to create hall_displacement_task\r\n");
		/* Handle error on task creation. */
	}
}

/*
 * Description: Odrive data aquisition task
 * Using functions in odrive_comms.c, read encoder data for displacement and velocity
 * and other data
 */
void odrive_daqa_task(void) {
//  OS_RATE_HZ os_tick_rate;
	RTOS_ERR p_err;


	char *displacement_buffer = (char*) malloc(BUFSIZE);
	char *velocity_buffer = (char*) malloc(BUFSIZE);
	char *current_buffer = (char*) malloc(BUFSIZE);
	char *voltage_buffer = (char*) malloc(BUFSIZE);
	char *current_errors_buffer = (char*) malloc(BUFSIZE);
	char *current_state_buffer = (char*) malloc(3);
	char *control_mode_buffer = (char*) malloc(3);
	char *vel_set_point_buffer = (char*) malloc(4);
	char *torque_set_point_buffer = (char*) malloc(4);
	char *vel_gain_buffer = (char*) malloc(4);
	char *vel_integrator_gain_buffer = (char*) malloc(4);

	char *custom_cmd_buffer = (char*) malloc(48);

	//reading displacement, velocity, and direction from Odrive using odrive_comms.c functions
	while (1) {

		if (run_custom_odrive_command) {
			sl_iostream_write(sl_iostream_odrive_handle, custom_odrive_command,
					strlen(custom_odrive_command));

			read_odrive_f(custom_cmd_buffer);
			printf("<<read: %s>>\r\n", custom_cmd_buffer);
			run_custom_odrive_command = false;
		}
		//reading displacement data, calculating absolute and relative displacements
		displacement_abs = get_disp_f(displacement_buffer);
		displacement_rel = displacement_abs - displacement_origin;
		production_odrive_data.displacement = displacement_rel;

		//reading velocity
		production_odrive_data.velocity = get_velocity_f(velocity_buffer);
		if (odrive_velocity < 0) {
			production_odrive_data.direction = -1;
		} else {
			production_odrive_data.direction = 1;
		}

		//calibrate encoder with CTD pressure
//      if(calibrate_state == calibrate_on){
//          calibration_f();
//      }

		production_odrive_data.current = get_current_f(current_buffer);
		production_odrive_data.voltage = get_voltage_f(voltage_buffer);
		production_odrive_data.current_errors = get_current_errors_f(
				current_errors_buffer);
		production_odrive_data.current_state = get_current_state_f(
				current_state_buffer);
		production_odrive_data.control_mode = get_control_mode_f(
				control_mode_buffer);
		production_odrive_data.vel_set_point = get_vel_set_point_f(
				vel_set_point_buffer);
		production_odrive_data.torque_set_point = get_torque_set_point_f(
				torque_set_point_buffer);
		production_odrive_data.vel_gain = get_vel_gain_f(vel_gain_buffer);
		production_odrive_data.vel_integrator_gain = get_vel_integrator_gain_f(
				vel_integrator_gain_buffer);





		OSTimeDly(5,    //   consumer delay is #define at the beginning OS Ticks
				OS_OPT_TIME_DLY,          //   from now.
				&p_err);
		//   Check error code.
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(p_err) == RTOS_ERR_NONE), ;);
	}
}

/*
 * Description: create and open file for data storage
 * will be run before initializing tasks to avoid issues with hang
 *
 */ //2022 Dec 5 eb testing file interaction w/ stream
void file_setup(void) {

	int retval = 0;

	FRESULT fr; //result of f_stat
	FILINFO fno; //file info struct from f_stat

	// 2023 June 27 MB adding to file_setup to check if file exists, make new file in SD card if it does
	// checking if file name exists yet
	// while file already exists, change file name
	// if file name does not exist, proceed as normal
	fr = f_stat(file_name, &fno);
	while (fr == FR_OK) {
		num_file++;
		sprintf(file_name, "LW_data_%d.txt", num_file);
		fr = f_stat(file_name, &fno);
	}
	printf("\nwriting to file name: %s \n", file_name);

	//create testfile on sd card and write a test string
	retval = f_open(&superfile, file_name,
	FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	if (retval != FR_OK) {
		printf("Failed to open %s, error %u\n", file_name, retval);
	}

	//closing and reopening is taxing- but we'd like to write to the file without overwriting
	//each data entry. opening in FA_OPEN_APPEND lets us write multiple lines to the file
	retval = f_open(&superfile, file_name, FA_OPEN_APPEND | FA_WRITE | FA_READ);
	if (retval != FR_OK) {
		printf("Failed to open %s, error %u\n", file_name, retval);
	}

	return;

}

//2023 June 12 eb adding sbe49 decoding functions/variables
/*
 * Description: convert sbe49 decimal temperature to engineering units
 *
 */

float sbe_get_temp(int dec_temp) {
	float TA0 = 7.868136e-04;
	float TA1 = 2.913209e-04;
	float TA2 = -3.065917e-06;
	float TA3 = 2.232588e-07;

	float temp = 0.0f;
	float MV = 0.0f;
	float R = 0.0f;

	MV = (dec_temp - 524288) / (1.6e+007);
	R = (MV * 2.295e+10 + 9.216e+8) / (6.144e+4 - MV * 5.3e+5);

	temp = (1
			/ (TA0 + TA1 * log(R) + TA2 * log(R) * log(R)
					+ TA3 * log(R) * log(R) * log(R))) - 273.15;

	return temp;
}

float sbe_get_press(int dec_press, int dec_tpc) {
	float PA0 = -2.539195e-01;
	float PA1 = 4.417403e-03;
	float PA2 = 3.207088e-12;
	float PTCA0 = 5.204336e+05;
	float PTCA1 = -1.754152e+00;
	float PTCA2 = -2.458463e-02;
	float PTCB0 = 2.490738e+01;
	float PTCB1 = -7.250000e-04;
	float PTCB2 = 0.000000e+00;
	float PTEMPA0 = -5.265427e+01;
	float PTEMPA1 = 6.110659e+01;
	float PTEMPA2 = -3.026804e+00;

	double y = 0.0f, t = 0.0f, x = 0.0f, n = 0.0f;
	double press = 0.0f;

	y = ((double) dec_tpc) * 0.0000762951;
	t = PTEMPA0 + PTEMPA1 * y + PTEMPA2 * y * y;
	x = ((double) dec_press) - PTCA0 - PTCA1 * t - PTCA2 * t * t;
	n = (x * PTCB0) / (PTCB0 + PTCB1 * t + PTCB2 * t * t);
	press = PA0 + PA1 * n + PA2 * n * n;
	press = (press - 14.6959488) * 0.689476;

//  printf("%f,%f,%f,%f\r\n",y,t,x,n);

	return (float) press;
}

void sbe_hex_to_binary_f(char *raw_sbe_data, int indx,
		main_lw_struct *sbe_data_frame) { //ebohannon - 14 Oct 2022
//  raw_sbe_data
	int hex_read_idx = 0;
	int hex_table_idx = 0;
	int hex_countdown = 0;
	char *hex_table_value;
//  char hex_table[] = {"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F"};
	char *hex_table_string = "0123456789ABCDEF";

	char raw_temp[6] = { '\0' };
	char raw_con[6] = { '\0' };
	char raw_press[6] = { '\0' };
	char raw_tpc[4] = { '\0' };

	int dec_temp = 0;
	int dec_con = 0;
	int dec_press = 0;
	int dec_tpc = 0;

	int decoded_temp = 0;
	int decoded_press = 0;

	hex_countdown = 5;
	while (hex_read_idx < 6) {

		raw_temp[hex_read_idx] = raw_sbe_data[hex_read_idx]; //grab first char of data
		hex_table_value = strchr(hex_table_string, raw_temp[hex_read_idx % 6]); //find first occurrance of hex char in hex table
		hex_table_idx = (int) (hex_table_value - hex_table_string); //grab index of hex table value
		dec_temp = dec_temp + hex_table_idx * (pow(16, hex_countdown)); //use hex index and read index to sum up to get decimal value
		hex_countdown--;
		hex_read_idx++;
	}
	hex_countdown = 5;
	while (hex_read_idx < 12) {

		raw_con[hex_read_idx % 6] = raw_sbe_data[hex_read_idx];
		hex_table_value = strchr(hex_table_string, raw_con[hex_read_idx % 6]);
		hex_table_idx = (int) (hex_table_value - hex_table_string);
		dec_con = dec_con + hex_table_idx * (pow(16, hex_countdown));
		hex_countdown--;
		hex_read_idx++;
	}
	hex_countdown = 5;
	while (hex_read_idx < 18) {

		raw_press[hex_read_idx % 6] = raw_sbe_data[hex_read_idx];
		hex_table_value = strchr(hex_table_string, raw_press[hex_read_idx % 6]);
		hex_table_idx = (int) (hex_table_value - hex_table_string);
		dec_press = dec_press + hex_table_idx * (pow(16, hex_countdown));
		hex_countdown--;
		hex_read_idx++;
	}
	hex_countdown = 3;
	while (hex_read_idx < 22) {

		raw_tpc[hex_read_idx % 6] = raw_sbe_data[hex_read_idx];
		hex_table_value = strchr(hex_table_string, raw_tpc[hex_read_idx % 6]);
		hex_table_idx = (int) (hex_table_value - hex_table_string);
		dec_tpc = dec_tpc + hex_table_idx * (pow(16, hex_countdown));
		hex_countdown--;
		hex_read_idx++;
	}

	//convert decimal values to engineering units
	decoded_temp = (int) sbe_get_temp(dec_temp);
	decoded_press = (int) 10.0f * sbe_get_press(dec_press, dec_tpc); //pressure in centibar

	current_press = decoded_press;

	//store decoded data to lw decode circular buffer
	sbe_data_frame->livewire_decode_cir_buff[indx].temp = decoded_temp;
	sbe_data_frame->livewire_decode_cir_buff[indx].con = dec_con;
	sbe_data_frame->livewire_decode_cir_buff[indx].press = decoded_press;
	sbe_data_frame->livewire_decode_cir_buff[indx].tpc = dec_tpc;

//  printf("%s, %d,%d %d\r\n",raw_press,dec_press,dec_tpc,decoded_press);

	return;
}

// Callback function used in read_sbe_f()
void dma_checkpoint_f() {
	flag_dma_1 = 1;
}

// Callback function used in read_sbe_f()
// loops DMADRV reading 24 bytes at a time
void read_full_ctd_str_f(char *ctd_buffer) {

	flag_dma_2 = 1;

	DMADRV_AllocateChannel(&chanid, NULL);

	DMADRV_PeripheralMemory(chanid, dmadrvPeripheralSignal_USART0_RXDATAV,
			ctd_buffer, (void*) &(USART0->RXDATA),
			true,
			SBE_BUFFER_SIZE, dmadrvDataSize1, read_full_ctd_str_f,
			NULL);
}

/* 2023 November 15 - mbarrera adding a read_sbe_f to read CTD data using DMA
 * Objective: read CTD data stream using DMA instead of sl_iostream
 * Input: ctd_buffer; pointer to char array
 * Output: ctd_buffer; ctd_buffer should store 24 byte CTD string
 *
 * Description:
 * Use silab's DMADRV API to read the SBE serial port without needing the CPU
 * First, read CTD's USART stream byte by byte until you reach \n delimiter
 * Once you detect the \n delimiter, start reading 24 bytes at a time
 */
void read_sbe_f(char *ctd_buffer) {
	char ctd_sync_char;

	switch (sync_state) {
	case (search_for_sync):
		// read one byte at a time until you reach \n delimiter
		while (ctd_sync_char != '\n') {
			DMADRV_AllocateChannel(&chanid, NULL);
			DMADRV_PeripheralMemory(chanid,
					dmadrvPeripheralSignal_USART0_RXDATAV, &ctd_sync_char,
					(void*) &(USART0->RXDATA),
					true, 1, dmadrvDataSize1, dma_checkpoint_f,
					NULL); //Get 1 char constantly to reach EOL and sync.

			/* function idles in while loop until flag_dma_1 == 1 to
			 * make sure DMADRV_PeripheralMemory has finished
			 */
			while (true) {
				if (flag_dma_1 == 1) {
					break;
				}
				osDelay(1);
			}
			//reset flag_dma_1
			flag_dma_1 = 0;
		}

		//we have now synced with CTD stream, move to next case
		sync_state = synced;
		break;

	case (synced):
		//now that we are synced, transfer 24 bytes at a time
		DMADRV_PeripheralMemory(chanid, dmadrvPeripheralSignal_USART0_RXDATAV,
				ctd_buffer, (void*) &(USART0->RXDATA),
				true,
				SBE_BUFFER_SIZE, dmadrvDataSize1, read_full_ctd_str_f,
				ctd_buffer);

		/* function idles in while loop until flag_dma_1 == 1 to
		 * make sure DMADRV_PeripheralMemory has finished
		 */
		while (true) {
			if (flag_dma_2 == 1) {
				break;
			}
			osDelay(1);
		}

		// reset flag_dma_2
		flag_dma_2 = 0;

		/*  check if the last byte in buffer is delimiter
		 *  if last byte is \n, proceed with syncing
		 *  if last byte is not \n, go back to search_for_sync state
		 */
		if (ctd_buffer[23] == '\n') {
			sync_state = synced;
		} else {
			sync_state = search_for_sync;
		}
		break;
	}
}

//6/7/24 - gschellenberg add manual string output construction
void assemble_output_f(char output[], int64_t timestamp, char raw_ctd_data[24],
	int pres, odrive_data_t odrv) {
	int pos = 0;
	int len_32_bit_out = 8;
	int single_dig_out = 1;

	// Convert integers to padded hexadecimal strings
	int_to_hex_f(timestamp, 16, logging_hex1);
	int_to_hex_f(pres, 8, logging_hex2);
	int_to_hex_f(odrv.displacement, 8, logging_hex3);
	int_to_hex_f(odrv.velocity, 8, logging_hex4);
	int_to_hex_f(odrv.current, 8, logging_hex5);
	int_to_hex_f(odrv.voltage, 8, logging_hex6);
	int_to_hex_f(odrv.current_errors, 8, logging_hex7);
	int_to_hex_f(odrv.current_state, 1, logging_hex8);
	int_to_hex_f(odrv.control_mode, 1, logging_hex9);
	int_to_hex_f(odrv.vel_set_point, 8, logging_hex10);
	int_to_hex_f(odrv.torque_set_point, 8, logging_hex11);
	int_to_hex_f(odrv.vel_gain, 8, logging_hex12);
	int_to_hex_f(odrv.vel_integrator_gain, 8, logging_hex13);
	int_to_hex_f(prof_upper_limit, 8, logging_hex14);
	int_to_hex_f(prof_lower_limit, 8, logging_hex15);
	int_to_hex_f(prof_state,1, logging_hex16);

	// Copy hexadecimal strings to output string
	memcpy(output + pos, logging_hex1, 16);
	pos += 16;
	memcpy(output + pos, logging_hex2, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex3, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex4, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex5, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex6, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex7, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex8, single_dig_out);
	pos += single_dig_out;
	memcpy(output + pos, logging_hex9, single_dig_out);
	pos += single_dig_out;
	memcpy(output + pos, logging_hex10, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex11, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex12, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output + pos, logging_hex13, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output+pos, logging_hex14, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output+pos, logging_hex15, len_32_bit_out);
	pos += len_32_bit_out;
	memcpy(output+pos, logging_hex16, single_dig_out);
	pos += single_dig_out;

	output[pos] = '\n'; // Null terminator
	output[pos + 1] = '\0'; // Null terminator
	return;
}

//6/7/24 - gschellenberg int to hex string
//6/10/24 - gschellenberg switched to hex lookup table
const char hexLookupTable[] = "0123456789ABCDEF";
void int_to_hex_f(int64_t number, int length, char output[]) {
    int index = length-1;
    output[length] = '\0';

    for (int i = 0; i < length; i++) {
    	output[index] = hexLookupTable[number & 0xF];
        number >>= 4;
        index--;
    }
}

//Below does not work with negative numbers
//void int_to_hex_f(int number, int length, char output[]) {
//	// Convert number to hexadecimal
//	int i = length - 1;
//	while (number != 0 && i >= 0) {
//		int remainder = number % 16;
//		if (remainder < 10) {
//			output[i] = remainder + 48;
//		} else {
//			output[i] = remainder + 87;
//		}
//		number = number / 16;
//		i--;
//	}
//	// Pad with zeros
//	while (i >= 0) {
//		output[i] = '0';
//		i--;
//	}
//}

//TODO:

//check tasks running by toggling GPIO pins- faster than printing
//check San's seabird sim project for ^ example of this
//changelog format: 2022 11 16 (yr mo day) eb (initial) changes (short summary of changes - location helps)
