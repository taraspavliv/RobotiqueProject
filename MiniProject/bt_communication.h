/*
 * bt_communication.h
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */

#ifndef BT_COMMUNICATION_H_
#define BT_COMMUNICATION_H_

#define BFR_IN_SIZE 48

#define ANGLE_CMD 'a'
#define DISTANCE_CMD 'd'
#define ENEMY_X_CMD 'u'
#define ENEMY_Y_CMD 'v'
#define SHOOT_CMD 'h'
#define CALIBRATE_CMD 'c'

#define MAX_INTEGER_LENGTH 5

#define ASCII_OF_ZERO 48

void bt_communication_start(void);

//main functions
void get_input_buffer(BaseSequentialStream* in, char* input_bfr);
void process_input_bfr(char* input_bfr);
void send_position(BaseSequentialStream* in);

//process functions
bool valid_command(char incomming_cmd);
bool is_number(char chara);
int16_t get_recieved_integer(char* incoming_message_bfr);

//getters
uint16_t* get_BT_enemy_position(void);
uint16_t* get_BT_controller_joystick_polar(void);
bool get_BT_controller_shoot(void);
bool get_BT_calibrate(void);

//setters
void reset_shoot(void);

#endif /* BT_COMMUNICATION_H_ */
