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

#include <leds.h>

#include <bt_communication.h>
#include <role_selector.h>
#include <position_motion_controller.h>

static THD_WORKING_AREA(waBluetoothComm, 512);
static THD_FUNCTION(BluetoothComm, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;


  while(true) {

	  //doesn't need to be very reactive, so long sleep time
	  ReceiveCommand((BaseSequentialStream *) &SD3);
	  chThdSleepMilliseconds(50);
  }
}

void ReceiveCommand(BaseSequentialStream* in){
	static char incoming_message_bfr[BFR_IN_SIZE] = {0}; //with a margin
	volatile char incoming_char='-';

	memset(incoming_message_bfr,0,sizeof(incoming_message_bfr));
	//fill the buffer
	uint8_t last_char_index = 0;
	do{
		incoming_char = chSequentialStreamGet(in);
		incoming_message_bfr[last_char_index] = incoming_char;
		++last_char_index;
	}while(incoming_char != '-');

	//empty the recienving message
	/*for(uint8_t m=0; m<5; ++m){
		if(chSequentialStreamGet(in)){
			incoming_char = chSequentialStreamGet(in);
		}
	}*/

	//expected data
	uint16_t angle_deg=0;
	uint8_t distance=0;
	uint16_t enemy_position_x = 0;
	uint16_t enemy_position_y = 0;
	bool shoot = false;
	bool calibrate = false;

	bool discard = false;

    for(int i=0; i < BFR_IN_SIZE && incoming_message_bfr[i] != '-'; ++i){
		if(valid_command(incoming_message_bfr[i])){
			if(incoming_message_bfr[i+1]==':'){
				//translates incoming string to integer
				uint16_t k = 0;
				for(int j = 0; j < MAX_DATA_IN_LENGTH; ++j){
					if(is_number(incoming_message_bfr[i+2+j])){
						k+=(uint8_t)incoming_message_bfr[i+2+j]-ASCII_OF_ZERO;
					}else if(incoming_message_bfr[i+2+j] == ' '){
						break;
					}else{
						discard = true;
						break;
					}
					k *= 10;
				}
				k /= 10;
				if(discard == false){
					if(ANGLE_CMD(incoming_message_bfr[i])){
						angle_deg = k;
					}else if(DISTANCE_CMD(incoming_message_bfr[i])){
						distance = k;
					}else if(ENEMY_X_CMD(incoming_message_bfr[i])){
						enemy_position_x = k;
					}else if(ENEMY_Y_CMD(incoming_message_bfr[i])){
						enemy_position_y = k;
					}
				}

			}else if(SHOOT_CMD(incoming_message_bfr[i])){
				shoot = true;
			}else if(CALIBRATE_CMD(incoming_message_bfr[i])){
				calibrate = true;
			}
		}
    }

    //handle recieved data
    float angle_rad = 0;
    angle_rad = (angle_deg * PI) / 180; // Converting to radian

    set_led(0,cos(angle_rad) > 0);
    set_led(2,cos(angle_rad) < 0);

    set_led(1,sin(angle_rad) > 0);
    set_led(3,sin(angle_rad) < 0);
	/*left_motor_set_speed(8*distance*(cos(angle)+sin(angle)));
	right_motor_set_speed(8*distance*(cos(angle)-sin(angle)));

	//initialisation des variables


	float l1=0; //increment de la roue right
	float l2=0; //increment de la roue left
	l1= (right_motor_get_pos()-stepr_1)*CONVERTER; //de step a mm
	l2= (left_motor_get_pos()-stepl_1)*CONVERTER;

	position_x= position_x+(l1+l2)*cosf(my_angle)/2;
	position_y= position_y+(l1+l2)*sinf(my_angle)/2;
	my_angle=my_angle+tanf((l1-l2)/RAYON);
	my_angle = my_angle*180/PI;

	stepr_1=right_motor_get_pos();
	stepl_1=left_motor_get_pos();*/
	//chprintf((BaseSequentialStream *)&SD3, "x:%f y:%f theta:%f \n\r", position_x, position_y, my_angle);
}

bool valid_command(char incomming_cmd){
	return ANGLE_CMD(incomming_cmd) || DISTANCE_CMD(incomming_cmd) || ENEMY_X_CMD(incomming_cmd) ||
		   ENEMY_Y_CMD(incomming_cmd) || SHOOT_CMD(incomming_cmd) || CALIBRATE_CMD(incomming_cmd);
}

bool is_number(char chara){
    if((uint8_t)chara >= ASCII_OF_ZERO && (uint8_t)chara <= ASCII_OF_ZERO+9){
        return true;
    }else{
        return false;
    }
    return false;
}

void bt_communication_start(void){
	chThdCreateStatic(waBluetoothComm, sizeof(waBluetoothComm), NORMALPRIO, BluetoothComm, NULL);
}
