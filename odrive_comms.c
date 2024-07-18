/* odrive_comms.c
 *
 * Created: 17 August 2023
 * Author: Marfred Barrera
 *
 * Contains all functions and variables related to Odrive communications
 *
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "em_chip.h"
#include "sl_iostream.h"
#include "sl_iostream_init_instances.h"
#include "sl_iostream_handles.h"
#include "os.h"
#include <math.h>
#include "odrive_comms.h"
#include "task.h"

//variable declaration for zeroing and finding relative displacement
int displacement_origin = 0;
int displacement_abs;
int displacement_rel;

//variable declaration for calculating velocity and direction
int odrive_velocity;
int odrive_direction;
float displacement_current;
float displacement_last = 0.0;

//variable declaration for tracking total distance
int calibration_distance = 1000;
int linear_displacement_current;
int linear_displacement_last;
int total_distance;

//variable declaration for calibration
int distance_since_calibration;
int press_at_zero;
int gauge_press;

//variable declaration for odrive data - gschellenberg 5/29/24
int odrive_voltage;
int odrive_current;
int odrive_current_errors;
int odrive_current_state;
int odrive_control_mode;
int odrive_vel_set_point;
int odrive_torque_set_point;
int odrive_vel_gain;
int odrive_vel_integrator_gain;



void flushBuffer(char*buffer, size_t size){
  memset(buffer,0,size);
}

/*
 * Function: read_odrive_f()
 * Objective: return a string from Odrive's TX stream
 * inputs: pointer to buffer
 * output: char array ending with terminating \0 from odrive handle
 */
void read_odrive_f(char* odrive_buffer){
    uint8_t index = 0;
//    char odrive_buffer[BUFSIZE] = "\0";
    int8_t c = 0;
    sl_status_t odrive_read_flag;

//    flushBuffer(odrive_buffer,BUFSIZE);




    //gathering all characters up until and including /n, storing in odrive_buffer
        while(!(c == '\n')){

             odrive_read_flag = sl_iostream_read(sl_iostream_odrive_handle, &c, 1, NULL);
             if(index < BUFSIZE - 1){
                 odrive_buffer[index] = c;
                 index++;
             }
        }

        //add terminating /0
        odrive_buffer[index+1] = '\0';

        // wait for read process to finish
        while(odrive_read_flag != SL_STATUS_OK){;
          osDelay(1);
        }

 //       printf("%s\r\n", odrive_buffer);

        return;
}

void write_odrive_f(char* odrive_w_buffer, size_t buffer_length){
  sl_status_t odrive_write_flag;
  odrive_write_flag = sl_iostream_write(sl_iostream_odrive_handle, odrive_w_buffer,buffer_length);

  // wait until write process is finished
  while(odrive_write_flag != SL_STATUS_OK){;
    osDelay(1);
  }


}


/*
 * Function: get_disp_f()
 * Objective: return current encoder displacement
 * input: none
 * output: encoder displacement in mm
 *
 */
int get_disp_f(char* odrive_buffer){
  char* str;
  float disp;
  int disp_int;

//  sl_iostream_write(sl_iostream_odrive_handle, "f 0!", 4);

  write_odrive_f("f 0!",4);

  read_odrive_f(odrive_buffer);

  //printf("odrive_buffer: %s", odrive_buffer);

  disp = atof(odrive_buffer)*2*M_PI*3; //units: in
  disp = disp * 2.54; //units: cm

  disp_int = (int)(disp); //units: cm
  return disp_int;
}

//char_to_digit_f: take a char, compare it to '0', '1' ... '9'
// return int 0,1,...,9
//int char_to_digit_f(char* c){
//  switch(c){
//    case '1':
//      return 1;
//      break;
//    case '2':
//      return 2;
//      break;
//    case '3':
//      return 3;
//      break;
//    case '4':
//      return 4;
//      break;
//    case '5':
//      return 5;
//      break;
//    case '6':
//      return 6;
//      break;
//    case '7':
//      return 7;
//      break;
//    case '8':
//      return 8;
//      break;
//    case '9':
//      return 9;
//      break;
//    case '.':
//      return 0;
//      break;
//  }
//
//  return 0;
//}
/*
 * Function: get_vel_f()
 * Objective: return instantaneous encoder velocity
 * inputs: none
 * output: encoder velocity in rpm
 */
int get_velocity_f(char* odrive_buffer){

  float velocity;
  int velocity_int;

//  sl_iostream_write(sl_iostream_odrive_handle, "r axis0.pos_vel_mapper.vel!", 27);
  write_odrive_f("r axis0.pos_vel_mapper.vel!", 27);
  read_odrive_f(odrive_buffer);

//  printf("odrive_buffer: %s", odrive_buffer);

//  vel_ptr = strtok(odrive_buffer, " ");
//  vel_ptr = strtok(NULL, " ");

  //printf("%s\r\n", vel_ptr);

  velocity = atof(odrive_buffer); //units: turns per second
  //if - start from index 1 instead of 0, at the end
//  if(*odrive_buffer != '-'){
//      vel += char_to_digit_f(*(odrive_buffer));
//      for(int i=2; i<8; i++){
//          vel += char_to_digit_f(*(odrive_buffer+i))/(10**(i-1));
//      }
//  }

  velocity_int = (int) (velocity*1000);
//  printf("velocity: %d\r\n", velocity_int);
  return velocity_int;
}

int get_direction_f(char* odrive_buffer){

  int odrive_direction;

  write_odrive_f("r axis0.pos_vel_mapper.vel!", 27);
  read_odrive_f(odrive_buffer);

 // printf("odrive_buffer: %s", odrive_buffer);

  if(*odrive_buffer == '-'){
      odrive_direction = ODRIVE_UP;
  }
  else{
    odrive_direction = ODRIVE_DOWN;
  }

  return odrive_direction;
}

//odrive current and voltage sensing - gschellenberg 5/29/24
int get_voltage_f(char* odrive_buffer){
  float voltage;
  int voltage_int;

  write_odrive_f("r vbus_voltage!", strlen("r vbus_voltage!"));

  read_odrive_f(odrive_buffer);

  //printf("odrive_buffer: %s", odrive_buffer);

  voltage = atof(odrive_buffer); //units: volts

  voltage_int = (int)(voltage*1000); //units: 1000volts
  return voltage_int;
}

int get_current_f(char* odrive_buffer){
  float current;
  int current_int;

  write_odrive_f("r ibus!", strlen("r ibus!"));

  read_odrive_f(odrive_buffer);

  //printf("odrive_buffer: %s", odrive_buffer);

  current = atof(odrive_buffer); //units: volts

  current_int = (int)(current*1000); //units: amps
  return current_int;
}

int get_current_errors_f(char* odrive_buffer){
  write_odrive_f("r axis0.active_errors!", strlen("r axis0.active_errors!"));
  read_odrive_f(odrive_buffer);

  return atoi(odrive_buffer);
}

int get_current_state_f(char* odrive_buffer){
  write_odrive_f("r axis0.current_state!", strlen("r axis0.current_state!"));

  read_odrive_f(odrive_buffer);

  return atoi(odrive_buffer);
}

int get_control_mode_f(char* odrive_buffer){
  write_odrive_f("r axis0.controller.config.control_mode!", strlen("r axis0.controller.config.control_mode!"));

  read_odrive_f(odrive_buffer);

  return atoi(odrive_buffer);
}

int get_vel_set_point_f(char* odrive_buffer){
  write_odrive_f("r axis0.controller.vel_setpoint!", strlen("r axis0.controller.vel_setpoint!"));

  read_odrive_f(odrive_buffer);

  return (int)(atof(odrive_buffer)*1000);
}

int get_torque_set_point_f(char* odrive_buffer){
  write_odrive_f("r axis0.controller.torque_setpoint!", strlen("r axis0.controller.torque_setpoint!"));

  read_odrive_f(odrive_buffer);

  return (int)(atof(odrive_buffer)*1000);
}

int get_vel_gain_f(char* odrive_buffer){
  write_odrive_f("r axis0.controller.config.vel_gain!", strlen("r axis0.controller.config.vel_gain!"));

  read_odrive_f(odrive_buffer);

  return (int)(atof(odrive_buffer)*1000);
}

int get_vel_integrator_gain_f(char* odrive_buffer){
  write_odrive_f("r axis0.controller.config.vel_integrator_gain!", strlen("r axis0.controller.config.vel_integrator_gain!"));

  read_odrive_f(odrive_buffer);

  return (int)(atof(odrive_buffer)*1000);
}

void set_vel_f(float reference_vel){
  int len = snprintf(NULL, 0, "v 0 %f!", reference_vel);
  ++len;
  char *output[len];
  snprintf(output,len, "v 0 %f!", reference_vel);
  write_odrive_f(output, len);
}

void set_vel_gain_f(int vel_gain){
  char output[80];
  sprintf(output,"w axis0.controller.config.vel_gain 0.%d!",vel_gain);
  write_odrive_f(output,strlen(output));
}

void set_vel_integrator_gain_f(int vel_integrator_gain){
  char output[80];
  sprintf(output,"w axis0.controller.config.vel_integrator_gain 0.%d!",vel_integrator_gain);
  write_odrive_f(output,strlen(output));
}

void set_torque_f(float reference_torque){
  int len = snprintf(NULL, 0, "c 0 %f!", reference_torque);
  ++len;
  char *output[len];
  snprintf(output,len, "c 0 %f!", reference_torque);
  write_odrive_f(output, len);
}

void set_control_mode_f(int control_mode){
  int len = snprintf(NULL, 0, "w axis0.requested_state %d!", control_mode);
  ++len;
  char *output[len];
  snprintf(output,len, "w axis0.requested_state %d!", control_mode);
  write_odrive_f(output,len);
}

/*
 * Function: calibration_f()
 * Objective: calibrate encoder displacement data with SBE pressure once a certain distance is reached
 * input: none
 * output: none
 *
 */
//void calibration_f()
//{
////  printf("distance: %+05d, displacement_rel: %+05d, displacement_abs: %+05d, gauge_press: %+05d, press_at_zero: %+05d, displacement_origin: %+05d \r\n",
////         distance_since_calibration,
////         displacement_rel,
////         displacement_abs,
////         gauge_press,
////         press_at_zero,
////         displacement_origin);
//
//  distance_since_calibration += abs(linear_displacement_current - linear_displacement_last);
//
//  gauge_press = current_press - press_at_zero;
//
//  if(distance_since_calibration > calibration_distance){
//      displacement_origin = displacement_abs - (gauge_press*10);
//
//
//      distance_since_calibration = 0;
//  }
//}
