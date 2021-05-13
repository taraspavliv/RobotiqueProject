#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <leds.h>
#include <spi_comm.h> //for RGB leds
#include <selector.h>

#include "role_selector.h"
#include "process_image.h"
#include "bt_communication.h"
#include "position_calibrator.h"
#include "motors_controller.h"

#define GOAL_WIDTH 300 //in mm
#define FIELD_WIDTH 600 //in mm
#define FIELD_HEIGHT 1000 //in mm
#define EPUCK_DIAMETER 80 //in mm
#define KEEPER_Y_MARGIN 10 //in mm

#define KEEPER_LEFT 160 //camera angle of view is 45 deg, so by turning to 160 should be able to see up to 180
#define KEEPER_RIGHT 20
#define KEEPER_INTERCEPT_THRS 9 //in mm^2*s^-2, we already square 3mm/s for optimisation

enum KeeperState{RETREATING, K_SCANNING, K_FOCUSING, INTERCEPTING, K_REFOCUSING};
enum AttackerState{A_SCANNING, A_FOCUSING, A_POSITIONING, A_REFOCUSING,  SHOOTING};

void attack_FSM(bool role_changed);
void keeper_FSM(bool role_changed);
void bt_control(void);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    //messagebus_init(); TODO:remove?

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start(); //TODO:remove?
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//for rgb leds
	//spi_comm_start();

	//stars the threads for the processing of the image
	process_image_start();
	role_selector_start();
	bt_communication_start();
	motors_controller_start();
	//position_calibrator_start();

	//set_rgb_led(1,200,200,0);
    /* Infinite loop. */
	enum Role my_role = SMART;
	bool smart_attacking = true; //if true, the epuck plays as an attacker, if false he plays as goalkeeper
	bool role_changed = false;
	bool smart_role_changed = false;
	bool calib_called = false;

    while (true) {
    	role_changed = (my_role != get_role());
    	my_role = get_role();

    	switch(my_role){
    	case ATTACKER:{
    		attack_FSM(role_changed);
    		break;
    	}
    	case GOALKEEPER:{
    		keeper_FSM(role_changed);
    		break;
    	}
    	case CONTROLLED:{
    		bt_control();
    		break;
    	}
    	case SMART:{
    		//if sees ball, checks if should switch state, (re)sets smart_role_changed
    		//smart_attacking?attack_FSM(smart_role_changed):keeper_FSM(smart_role_changed);
    	}
    	}
    	/*if(calib_called == false){
    		calibrate();
    		calib_called = true;
    	}*/


        chThdSleepMilliseconds(30);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

void bt_control(void){
    if(get_BT_controller_shoot() == true){
    	motor_shoot();
    	chThdSleepMilliseconds(700);
    	reset_BT_shoot();
    	reset_motor_shoot();
    }else{
    	control_motors_BT(get_BT_controller_joystick_polar());
    }
}

void keeper_FSM(bool role_changed){
	static enum KeeperState goalkeeper_state = K_SCANNING;
	static bool scanning_left = true; //false-> scanning right
	static int16_t refocus_angle = 0;

	if(role_changed){
		if((uint16_t)get_self_position()[0] > FIELD_WIDTH/2 - GOAL_WIDTH/2 && (uint16_t)get_self_position()[0] < FIELD_WIDTH/2 + GOAL_WIDTH/2 &&
		   (uint16_t)get_self_position()[1] > EPUCK_DIAMETER/2 && (uint16_t)get_self_position()[1] < EPUCK_DIAMETER/2 + KEEPER_Y_MARGIN){
			set_angle_obj(KEEPER_LEFT);
			scanning_left = true;
			goalkeeper_state = K_SCANNING;
		}else{
			int16_t goal_center[2] = {FIELD_WIDTH/2, EPUCK_DIAMETER/2 + KEEPER_Y_MARGIN};
			set_position_obj(goal_center);
			goalkeeper_state = RETREATING;
		}
	}

	switch(goalkeeper_state){
	case RETREATING:{
		if(get_position_achieved()){
			set_angle_obj(KEEPER_LEFT);
			goalkeeper_state = K_SCANNING;
		}
		break;
	}
	case K_SCANNING:{
		if(get_ball_visibility() == FULL || get_ball_visibility() == PARTIAL){
			float angle_to_ball = 0.0;
			angle_to_ball = RAD_TO_DEG(atan2f(get_ball_position()[1] - get_self_position()[1], get_ball_position()[0] - get_self_position()[1]));
			if(angle_to_ball < 0){
				angle_to_ball += 360;
			}
			set_angle_obj(angle_to_ball);
			goalkeeper_state = K_FOCUSING;
		}else{
			if(get_direction_achieved()){
				//scans to the other side when turned to the right angle
				scanning_left == true ? set_angle_obj(KEEPER_RIGHT) : set_angle_obj(KEEPER_LEFT);
				scanning_left = !scanning_left;
			}
		}
		break;
	}
	case K_FOCUSING:{
		if(get_ball_visibility() == PARTIAL){
			float angle_to_ball = 0.0;
			angle_to_ball = RAD_TO_DEG(atan2f(get_ball_position()[1] - get_self_position()[1], get_ball_position()[0] - get_self_position()[0]));
			if(angle_to_ball < 0){
				angle_to_ball += 360;
			}
			set_angle_obj(angle_to_ball);
		}else if(get_ball_visibility() == FULL){
			//will the ball score if epuck doesn't move? reposition + set refocus angle: keep focusing;
			int16_t* ball_position = {0};
			ball_position = get_ball_position();
			if(get_ball_movement()[1] < 0 && get_ball_movement()[0]*get_ball_movement()[0] + get_ball_movement()[1]*get_ball_movement()[1] >= KEEPER_INTERCEPT_THRS){
				float time_to_score = 0; //the time for the ball to arrive at the epuck level (in seconds)
				time_to_score = -(get_ball_position()[1] - EPUCK_DIAMETER/2 - KEEPER_Y_MARGIN) / get_ball_movement()[1];
				int16_t intercept_point[2] = {FIELD_WIDTH/2, EPUCK_DIAMETER/2 + KEEPER_Y_MARGIN}; //goal center by default
				intercept_point[0] = get_ball_position()[0] + time_to_score*get_ball_movement()[0];
				if(intercept_point[0] > FIELD_WIDTH/2 - GOAL_WIDTH/2 && intercept_point[0] < FIELD_WIDTH/2 + GOAL_WIDTH/2){
					set_position_obj(intercept_point);
					refocus_angle = RAD_TO_DEG(atan2f(get_ball_position()[1] - intercept_point[1], get_ball_position()[0] - intercept_point[0]));
					if(refocus_angle < 0){
						refocus_angle += 360;
					}
					goalkeeper_state = INTERCEPTING;
				}

			}
		}else if(get_ball_visibility() == NONE){
			goalkeeper_state = K_SCANNING;
		}
		break;
	}
	case INTERCEPTING:{
		if(get_position_achieved()){
			set_angle_obj(refocus_angle);
			goalkeeper_state = K_REFOCUSING;
		}
		break;
	}
	case K_REFOCUSING:{
		if(get_direction_achieved()){
			if(get_ball_visibility() == FULL || get_ball_visibility() == PARTIAL){
				goalkeeper_state = K_FOCUSING;
			}else if(get_ball_visibility() == NONE){
				goalkeeper_state = K_SCANNING;
			}
		}
		break;
	}
	default: break;
	}

}

void attack_FSM(bool role_changed){
	static enum AttackerState attacker_state = A_SCANNING;

	if(role_changed){
		right_motor_set_speed(880);
		left_motor_set_speed(-880);
		attacker_state = A_SCANNING;
	}
	//static bool scanning_left = true; //false-> scanning right
	//static int16_t refocus_angle = 0;

	static int16_t attack_position[2]={0,0};
	static int16_t refocus_angle = 0;

	set_led(0, attacker_state == A_SCANNING);
	set_led(1, attacker_state == A_FOCUSING);
	set_led(2, attacker_state == A_POSITIONING);
	set_led(3, attacker_state == A_REFOCUSING);

	switch(attacker_state){
	case A_SCANNING:{
		right_motor_set_speed(880);
		left_motor_set_speed(-880);
		if(get_ball_visibility() == FULL || get_ball_visibility() == PARTIAL){
			float angle_to_ball = 0.0;
			angle_to_ball = RAD_TO_DEG(atan2f(get_ball_position()[1] - get_self_position()[1], get_ball_position()[0] - get_self_position()[0]));
			if(angle_to_ball < 0){
				angle_to_ball += 360;
			}
			set_angle_obj(angle_to_ball);
			attacker_state = A_FOCUSING;
		}
		break;
	}
	case A_FOCUSING:{
		if(get_ball_visibility() == FULL ){
			//refocus angle
			refocus_angle = RAD_TO_DEG(atan2f(FIELD_HEIGHT -get_ball_position()[1], FIELD_WIDTH/2 - get_ball_position()[0]));
			if(refocus_angle < 0){
				refocus_angle += 360;
			}

			attack_position[0]= get_ball_position()[0]-cos(refocus_angle)*80;
			attack_position[1]= get_ball_position()[1]-sin(refocus_angle)*80;

			if(attack_position[0]<=40){
				attack_position[0]=40;
				refocus_angle=90;
			}else if(attack_position[0]>=560){
				attack_position[0]=560;
				refocus_angle=90;
			}
			if(attack_position[1]<=40){
				attack_position[1]=40;
				refocus_angle=0;
			}else if(attack_position[1]>=960){
				attack_position[1]=960;
				refocus_angle=0;
			}
			set_position_obj(attack_position);
			attacker_state = A_POSITIONING;
		}else if(get_ball_visibility() == NONE ){
			right_motor_set_speed(880);
			left_motor_set_speed(-880);
			attacker_state = A_SCANNING;
		}else if(get_ball_visibility() == PARTIAL){
			float angle_to_ball = 0.0;
			angle_to_ball = RAD_TO_DEG(atan2f(get_ball_position()[1] - get_self_position()[1], get_ball_position()[0] - get_self_position()[0]));
			if(angle_to_ball < 0){
				angle_to_ball += 360;
			}
			set_angle_obj(angle_to_ball);

		}
		break;
	}
	case A_POSITIONING:{
		if(get_position_achieved()){
			set_angle_obj(refocus_angle);
			attacker_state = A_REFOCUSING;
		}
		break;
	}
	case A_REFOCUSING:{
		if(get_direction_achieved()){
			if(get_ball_visibility() == FULL){
				attacker_state = SHOOTING;
			}
			if(get_ball_visibility() == PARTIAL){
				float angle_to_ball = 0.0;
				angle_to_ball = RAD_TO_DEG(atan2f(get_ball_position()[1] - get_self_position()[1], get_ball_position()[0] - get_self_position()[0]));
				if(angle_to_ball < 0){
					angle_to_ball += 360;
				}
				set_angle_obj(angle_to_ball);
				attacker_state = A_FOCUSING;
			}
			if(get_ball_visibility() == NONE){
				right_motor_set_speed(880);
				left_motor_set_speed(-880);
				attacker_state = A_SCANNING;
			}
		}
		break;
	}
	case SHOOTING:{
		motor_shoot();
		chThdSleepMilliseconds(900);
		reset_motor_shoot();
		attacker_state= A_FOCUSING;
		break;
	}
	default :break;	
	}
}
