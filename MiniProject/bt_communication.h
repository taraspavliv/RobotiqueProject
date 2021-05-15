/*
 * bt_communication.h
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */

#ifndef BT_COMMUNICATION_H_
#define BT_COMMUNICATION_H_

void bt_communication_start(void); //starts the thread for the bluetooth communication

//getters, BT is for BlueTooth
int16_t* get_BT_enemy_position(void);
uint16_t* get_BT_controller_joystick_polar(void);
bool get_BT_controller_shoot(void);
bool get_BT_calibrate(void);

#endif /* BT_COMMUNICATION_H_ */
