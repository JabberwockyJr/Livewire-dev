/*
 * odrive_comms.h
 *
 * Created: 17 August 2023
 * Author: Marfred Barrera
 *
 */


//defines
#ifndef BUFSIZE
#define BUFSIZE    24
#endif

#define M_PI     3.14159265358979323846
#define ODRIVE_DOWN 1
#define ODRIVE_UP -1


//global variables for use in task.c
extern int displacement_origin;
extern int displacement_abs;
extern int displacement_rel;
extern int odrive_velocity;
extern int odrive_direction;
extern int calibration_distance;
extern int linear_displacement_current;
extern int linear_displacement_last;
extern int total_distance;
extern int press_at_zero;
extern int distance_since_calibration;

extern int odrive_voltage;
extern int odrive_current;
extern int odrive_current_errors;
extern int odrive_current_state;
extern int odrive_control_mode;
extern int odrive_vel_set_point;
extern int odrive_torque_set_point;
extern int odrive_vel_gain;
extern int odrive_vel_integrator_gain;

//function prototypes
void flushBuffer(char* buffer, size_t size);
void read_odrive_f(char* odrive_buffer);
void write_odrive_f(char* odrive_w_buffer, size_t buffer_length);
int get_disp_f(char* odrive_buffer);
int get_velocity_f(char* odrive_buffer);
int get_direction_f(char* odrive_buffer);
int get_voltage_f(char* odrive_buffer);
int get_current_f(char* odrive_buffer);
int get_current_errors_f(char* odrive_buffer);
int get_current_state_f(char* odrive_buffer);
int get_control_mode_f(char* odrive_buffer);
int get_vel_set_point_f(char* odrive_buffer);
int get_torque_set_point_f(char* odrive_buffer);
int get_vel_gain_f(char* odrive_buffer);
int get_vel_integrator_gain_f(char* odrive_buffer);
void set_vel_f(float reference_v);
void set_vel_gain_f(int vel_gain);
void set_vel_integrator_gain_f(int vel_gain);
void set_control_mode_f(int control_mode);

void track_total_distance_f();
void calibration_f();
