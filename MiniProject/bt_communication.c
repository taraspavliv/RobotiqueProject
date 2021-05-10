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

#define BFR_IN_SIZE 48

#define ANGLE_CMD 'a'
#define DISTANCE_CMD 'd'
#define ENEMY_X_CMD 'u'
#define ENEMY_Y_CMD 'v'
#define SHOOT_CMD 'h'
#define CALIBRATE_CMD 'c'

#define MAX_INTEGER_LENGTH 5

#define ASCII_OF_ZERO 48

//expected recieved data
static uint16_t enemy_position[2] = {0,0}; // (x,y)
static uint16_t ctrl_joystick_polar[2] = {0,0}; //(distance, angle)
static bool ctrl_shoot = false;
static bool calibrate = false;

static THD_WORKING_AREA(waBluetoothComm, 512);
static THD_FUNCTION(BluetoothComm, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  static char incoming_message_bfr[BFR_IN_SIZE] = {0}; //with a margin

  while(true) {
	  get_input_buffer(incoming_message_bfr);
	  process_input_bfr(incoming_message_bfr);
	  send_position();
	  chThdSleepMilliseconds(50); //20Hz, like the phone upload frequency
  }
}

void get_input_buffer(char* input_bfr){
	char incoming_char = ' ';

	memset(input_bfr,' ',BFR_IN_SIZE);

	uint8_t last_char_index = 0;
	do{
		incoming_char = chSequentialStreamGet((BaseSequentialStream *) &SD3);
		input_bfr[last_char_index] = incoming_char;
		++last_char_index;
	}while(incoming_char != '-' && last_char_index < BFR_IN_SIZE); // in case the '-' is missed, to not overflow the array
}

void process_input_bfr(char* input_bfr){
	bool discard = false;
    for(int i=0; i < BFR_IN_SIZE && input_bfr[i] != '-'; ++i){
		if(valid_command(input_bfr[i])){

			discard = false;
			int16_t k = 0;//will store recieved integer

			if(input_bfr[i+1]==':'){
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
	chprintf((BaseSequentialStream *) &SD3, "x:%d y:%d -", get_self_position()[0], get_self_position()[1]);
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
		}else if(incoming_message_bfr[j] == ' '){ //every data followed by a space is considered valid
			break;
		}else{
			return -1; //corrupted data
		}
		k *= 10;
	}
	k /= 10;
	return k;
}

uint16_t* get_BT_enemy_position(void){ return enemy_position; }

uint16_t* get_BT_controller_joystick_polar(void){ return ctrl_joystick_polar; }

bool get_BT_controller_shoot(void){ return ctrl_shoot; }

bool get_BT_calibrate(void){ return calibrate; }

void reset_shoot(void){ ctrl_shoot = false; }

void bt_communication_start(void){
	chThdCreateStatic(waBluetoothComm, sizeof(waBluetoothComm), NORMALPRIO, BluetoothComm, NULL);
}
