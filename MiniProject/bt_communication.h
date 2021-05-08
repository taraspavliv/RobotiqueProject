/*
 * bt_communication.h
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */

#ifndef BT_COMMUNICATION_H_
#define BT_COMMUNICATION_H_

#define PI 3.14159265

#define BFR_IN_SIZE 48

#define ANGLE_CMD(charachter) charachter == 'a'
#define DISTANCE_CMD(charachter) charachter == 'd'
#define ENEMY_X_CMD(charachter) charachter == 'u'
#define ENEMY_Y_CMD(charachter) charachter == 'v'
#define SHOOT_CMD(charachter) charachter == 'h'
#define CALIBRATE_CMD(charachter) charachter == 'c'

#define MAX_DATA_IN_LENGTH 5

#define ASCII_OF_ZERO 48

void bt_communication_start(void);

void ReceiveCommand(BaseSequentialStream* in);

bool valid_command(char incomming_cmd);
bool is_number(char chara);
uint16_t* get_enemy_position(void);

#endif /* BT_COMMUNICATION_H_ */
