/*
 * position_motion_controller.h
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */

#ifndef MOTORS_CONTROLLER_H_
#define MOTORS_CONTROLLER_H_

#define PI 3.14159265
#define RAD_TO_DEG(radians) (radians*180/PI) //converts radians to centi-degree
#define DEG_TO_RAD(deg) (deg*PI/180)  //converts centi-degree to radians

void motors_controller_start(void);

int16_t* get_self_position(void);
int16_t get_self_angle(void);

void set_angle_obj(uint16_t angle);
void set_position_obj(int16_t* position);

float get_angle_difference(float angle1,float angle2);
float get_position_difference(float* position1,float* position2);

bool get_direction_achieved(void);
bool get_position_achieved(void);

void motor_shoot(void);
void reset_motor_shoot(void);

void control_motors_BT(uint16_t* joystick_polar_coord);

#endif /* MOTORS_CONTROLLER_H_ */
