/*
 * position_motion_controller.h
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */

#ifndef MOTORS_CONTROLLER_H_
#define MOTORS_CONTROLLER_H_

#define PI 3.14159265
#define RAD_TO_DEG(radians) (radians*180/PI) //converts radians to degree
#define DEG_TO_RAD(deg) (deg*PI/180)  //converts degree to radians

#define FULL_CIRCLE 360 //in degrees
#define HALF_CIRCLE 180 //in degrees
#define PERP 90 //perpendidular

void motors_controller_start(void); //starts the tread that calculates the position and orientation, and controls the motors

int16_t* get_self_position(void); //returns an array[2] with (x,y) coordinates (in mm) of the epuck
int16_t get_self_angle(void); //returns the angle of the epuck
//once the angle or the position has been achieved, we set a flag to signal it
bool get_angle_achieved(void);
bool get_position_achieved(void);

//list of possible commands. "obj" stands for objective
void set_angle_obj(uint16_t angle);
void set_position_obj(int16_t* position);
void set_distance_forward(float distance);
void set_rotation_speed(uint16_t speed, bool clockwise);

//to avoid a collision (in case there is an object in front, managed by "collision_manager") we set an intermediate point
uint8_t closest_point_to_obj(int16_t* point1, int16_t* point2); //return 1 if objective point is closer to point1 than point2, else returns 2 (for better choice of the intermediate point)
void set_intermediate_point(int16_t* position);
bool get_intermediate_achieved(void);
void continue_main_obj(void); //once the intermediate point has been achieved, return to original objective

float get_angle_difference(float angle1,float angle2);

void motor_shoot(void); //sets the motors to full speed
void reset_motor_shoot(void); //stops the motor

//once calibration is done, we need to reset current values to values read by the TOF sensor
void set_angle_calib(uint16_t angle_calib);
void set_position_calib(int16_t position_x, int16_t position_y);

void control_motors_BT(uint16_t* joystick_polar_coord); //controls the motors based on the joystick command (in bluetooth)

#endif /* MOTORS_CONTROLLER_H_ */
