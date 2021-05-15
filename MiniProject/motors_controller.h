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

void motors_controller_start(void);

bool manage_angle(float angle_objective);
bool manage_position(float* position_objective);

int16_t* get_self_position(void);
int16_t get_self_angle(void);

void set_angle_obj(uint16_t angle);
void set_position_obj(int16_t* position);
void set_distance_forward(float distance);
void set_rotation_speed(uint16_t speed, bool clockwise);
void set_intermediate_point(int16_t* position);

float get_angle_difference(float angle1,float angle2);
float get_position_difference(float* position1,float* position2);

bool get_angle_achieved(void);
bool get_position_achieved(void);
bool get_intermediate_achieved(void);
void continue_main_obj(void);

uint8_t closest_point_to_obj(int16_t* point1, int16_t* point2); //return 1 if objective point is closer to point1 than point2, else returns 2

void motor_shoot(void);
void reset_motor_shoot(void);

void set_angle_calib(uint16_t angle_calib);
void set_position_calib(int16_t position_x, int16_t position_y);

void control_motors_BT(uint16_t* joystick_polar_coord);

#endif /* MOTORS_CONTROLLER_H_ */
