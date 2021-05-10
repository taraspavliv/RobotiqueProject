/*
 * position_motion_controller.h
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */

#ifndef MOTORS_CONTROLLER_H_
#define MOTORS_CONTROLLER_H_

#define PI 3.14159265

void motors_controller_start(void);

float* get_self_position(void);
float get_self_angle(void);

void set_angle(float angle);

float get_angle_difference(float angle1,float angle2);
float get_position_difference(float* position1,float* position2);

bool get_direction_achieved(void);
bool get_position_achieved(void);

#endif /* MOTORS_CONTROLLER_H_ */
