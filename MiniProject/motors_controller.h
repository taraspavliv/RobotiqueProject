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

uint32_t* get_self_position(void);
uint16_t get_self_angle(void);
#endif /* MOTORS_CONTROLLER_H_ */
