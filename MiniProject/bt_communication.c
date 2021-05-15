/*
 * bt_communication.c
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>
#include <math.h>

#include <usbcfg.h>
#include <leds.h>

#include "bt_communication.h"
#include "role_selector.h"
#include "motors_controller.h"

//list of possible commands. a,d,u,v will be followed by ":" and a number with a maximum of MAX_INTEGER_LENGTH digits
#define ANGLE_CMD 'a' //gives the angle of the joystick on the phone app (only used if the epuck is in "controlled" role)
#define DISTANCE_CMD 'd' //gives the amplitude of the joystick (only used if the epuck is in "controlled" role)
#define SHOOT_CMD 'h' //gives a command to shoot if the button "shoot" on the phone app is pressed (only used if the epuck is in "controlled" role)
#define ENEMY_X_CMD 'u' //gives the x position of the enemy. /!\ it gives it in the coordinates of the enemy
#define ENEMY_Y_CMD 'v' //gives the y position of the enemy. /!\ it gives it in the coordinates of the enemy
#define CALIBRATE_CMD 'c' //gives a command to calibrate the epuck
#define END_OF_MESSAGE_CMD '-' //signals the end of the input message
#define SEP ' '//all commands are separated by a space ' '

#define MAX_INTEGER_LENGTH 5 //maximum length (in digit) of commands a,d,u,v

#define BFR_IN_SIZE 48 //size of the input buffer, input is expected to have a maximum size of 38

#define ASCII_OF_ZERO 48 //equivalent to (int)('0')

void get_input_buffer(char* input_bfr); //stores the input message from the bluetooth in input_bfr
void process_input_bfr(char* input_bfr); //updates the static variables based on the message commands
void send_position(void); //sends his position to bluetooth (the other epuck will recieve it as an enemy position command)

bool valid_command(char incomming_cmd); //returns true if the command exists
int16_t get_recieved_integer(char* incoming_message_bfr); //returns the number following the commands a,d,u,v
bool is_number(char chara); //returns true if the input character is between '0' and '9'

//expected recieved data
static int16_t enemy_position[2] = {0,0}; // (x,y) in mm
static uint16_t ctrl_joystick_polar[2] = {0,0}; //(distance, angle)
static bool ctrl_shoot = false;
static bool calibrate = false;

static THD_WORKING_AREA(waBluetoothComm, 512);
static THD_FUNCTION(BluetoothComm, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  static char incoming_message_bfr[BFR_IN_SIZE] = {0};

  while(true) {
	  get_input_buffer(incoming_message_bfr);
	  process_input_bfr(incoming_message_bfr);
	  send_position();
	  chThdSleepMilliseconds(50); //20Hz, same as the phone upload frequency
  }
}

void get_input_buffer(char* input_bfr){
	char incoming_char = SEP;
	memset(input_bfr,SEP,BFR_IN_SIZE);

	uint8_t last_char_index = 0;
	do{
		//stores the input characters inside the input buffer until and end of message command or the buffer is overflown
		incoming_char = chSequentialStreamGet((BaseSequentialStream *) &SD3);
		input_bfr[last_char_index] = incoming_char;
		++last_char_index;
	}while(incoming_char != END_OF_MESSAGE_CMD && last_char_index < BFR_IN_SIZE);
}

void process_input_bfr(char* input_bfr){
	bool discard = false;
    for(int i=0; i < BFR_IN_SIZE && input_bfr[i] != END_OF_MESSAGE_CMD; ++i){
		if(valid_command(input_bfr[i])){
			discard = false;
			int16_t k = 0;//will store recieved integer

			if(input_bfr[i+1]==':'){ //only 'a','d','u','v' will be followed by ":"
				k = get_recieved_integer(&input_bfr[i+2]);
				if(k<0){
					discard = true;
				}
			}
			//update data accordingly
			if(discard == false){
				switch(input_bfr[i]){
				case ANGLE_CMD:{
					ctrl_joystick_polar[1] = k;
					break;
				}
				case DISTANCE_CMD:{
					ctrl_joystick_polar[0] = k;
					break;
				}
				case ENEMY_X_CMD:{
					enemy_position[0] = k;
					break;
				}
				case ENEMY_Y_CMD:{
					enemy_position[1] = k;
					break;
				}
				case SHOOT_CMD:{
					ctrl_shoot = true;
					break;
				}
				case CALIBRATE_CMD:{
					calibrate = true;
					break;
				}
				default: break;
				}
			}
		}
    }

}

void send_position(void){
	chprintf((BaseSequentialStream *) &SD3, "x:%d y:%d -", (uint16_t)get_self_position()[0], (uint16_t)get_self_position()[1]);
	return;
}


bool valid_command(char incomming_cmd){
	if(incomming_cmd == ANGLE_CMD || incomming_cmd == DISTANCE_CMD ||incomming_cmd == ENEMY_X_CMD ||
	   incomming_cmd == ENEMY_Y_CMD ||incomming_cmd == SHOOT_CMD ||incomming_cmd == CALIBRATE_CMD){
		return true;
	}else{
		return false;
	}
}

bool is_number(char chara){
    if((uint8_t)chara >= ASCII_OF_ZERO && (uint8_t)chara <= ASCII_OF_ZERO + 9){
        return true;
    }else{
        return false;
    }
}

int16_t get_recieved_integer(char* incoming_message_bfr){
	//translates incoming following ASCII characeters to integer
	uint16_t k = 0;
	for(int j = 0; j < MAX_INTEGER_LENGTH; ++j){
		if(is_number(incoming_message_bfr[j])){
			k+=(uint8_t)incoming_message_bfr[j]-ASCII_OF_ZERO;
		}else if(incoming_message_bfr[j] == SEP){ //every data followed by a sepatator is considered valid
			break;
		}else{
			return -1; //means the following is neither a separator neither a number -> data is corrupted
		}
		k *= 10;
	}
	k /= 10;
	return k;
}

//getters
int16_t* get_BT_enemy_position(void){ return enemy_position; }

uint16_t* get_BT_controller_joystick_polar(void){ return ctrl_joystick_polar; }

bool get_BT_controller_shoot(void){
	if(ctrl_shoot){
		ctrl_shoot = false;
		return true;
	}else{
		return false;
	}
}

bool get_BT_calibrate(void){
	if(calibrate){
		calibrate = false;
		return true;
	}else{
		return false;
	}
}

void bt_communication_start(void){
	chThdCreateStatic(waBluetoothComm, sizeof(waBluetoothComm), NORMALPRIO, BluetoothComm, NULL);
}
