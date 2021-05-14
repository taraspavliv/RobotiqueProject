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
#include <sensors/proximity.h>

#include "role_selector.h"
#include "process_image.h"
#include "bt_communication.h"
#include "position_calibrator.h"
#include "motors_controller.h"
#include "collision_manager.h"

#define GOAL_WIDTH 300 //in mm
#define EPUCK_DIAMETER 80 //in mm
#define KEEPER_Y_MARGIN 10 //in mm

#define KEEPER_LEFT 160 //camera angle of view is 45 deg, so by turning to 160 should be able to see up to 180
#define KEEPER_RIGHT 20
#define KEEPER_INTERCEPT_THRS 9 //in mm^2*s^-2, we already square 3mm/s for optimisation
#define KEEPER_MIN_MOVE 10 //in mm, if less than this value, the epuck won't move

#define ATTACKER_DISTANCE_TO_CHARGE_BALL 120 //in mm

MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
	/** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);

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
	proximity_start();

	//stars the threads for the processing of the image
	process_image_start();
	role_selector_start();
	bt_communication_start();
	motors_controller_start();
	position_calibrator_start();
	collision_manager_start();


	//set_rgb_led(1,200,200,0);
    /* Infinite loop. */
	enum Role my_role = SMART;
	bool smart_attacking = true; //if true, the epuck plays as an attacker, if false he plays as goalkeeper
	bool role_changed = false;
	bool smart_role_changed = false;

	bool test_command_sent = false; //TODO:remove
	uint16_t posititon_test[2] = {0};
	posititon_test[0] = 400;
	posititon_test[1] = 200;
    while (true) {
    	if(test_command_sent == false){
    		test_command_sent = true;
    		set_position_obj(posititon_test);
    	}
    	/*role_changed = (my_role != get_role());
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
    		//if sees the ball, decides if it's better to attack or defend
    		if(get_ball_visibility() == FULL){
    			int16_t self_attacking_score = 0;
    			int16_t enemy_goal_center_point[2]= {0};
    			enemy_goal_center_point[0] = FIELD_WIDTH/2;
    			enemy_goal_center_point[1] = FIELD_HEIGHT;
    			self_attacking_score = attacking_score(get_self_position(), get_ball_position(), enemy_goal_center_point);

    			//from the enemy point of view, we should transform the coordinates of the ball
    			int16_t enemy_attacking_score = 0;
    			int16_t ball_in_enemy_coordinates[2] = {0};
    			ball_in_enemy_coordinates[0] = FIELD_WIDTH - get_ball_position()[0];
    			ball_in_enemy_coordinates[1] = FIELD_HEIGHT - get_ball_position()[1];
    			enemy_attacking_score = attacking_score(get_BT_enemy_position(), ball_in_enemy_coordinates, enemy_goal_center_point);

    			smart_role_changed = (smart_attacking == (self_attacking_score > enemy_attacking_score)); //if it was smart attacking and still is, role unchanged
    			smart_attacking = self_attacking_score > enemy_attacking_score;
    		}
    		if(smart_attacking){
    			attack_FSM(smart_role_changed);
    			smart_role_changed = false;
    		}else{
    			keeper_FSM(smart_role_changed);
    			smart_role_changed = false;
    		}
    		break;
    	}
    	default: break;
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
			if(get_angle_achieved()){
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
			if(get_ball_movement()[1] < 0 && get_ball_movement()[0]*get_ball_movement()[0] + get_ball_movement()[1]*get_ball_movement()[1] >= KEEPER_INTERCEPT_THRS){
				float time_to_score = 0; //the time for the ball to arrive at the epuck level (units unknown, but doesn't matter, we just want to project the interception point)
				time_to_score = -(get_ball_position()[1] - EPUCK_DIAMETER/2 - KEEPER_Y_MARGIN) / get_ball_movement()[1];
				int16_t intercept_point[2] = {FIELD_WIDTH/2, EPUCK_DIAMETER/2 + KEEPER_Y_MARGIN}; //goal center by default
				intercept_point[0] = get_ball_position()[0] + time_to_score*get_ball_movement()[0];
				if(intercept_point[0] > FIELD_WIDTH/2 - GOAL_WIDTH/2 && intercept_point[0] < FIELD_WIDTH/2 + GOAL_WIDTH/2){
					if(intercept_point[0] > get_self_position()[0] + KEEPER_MIN_MOVE || intercept_point[0] < get_self_position()[0] - KEEPER_MIN_MOVE){
						set_position_obj(intercept_point);
						refocus_angle = RAD_TO_DEG(atan2f(get_ball_position()[1] - intercept_point[1], get_ball_position()[0] - intercept_point[0]));
						if(refocus_angle < 0){
							refocus_angle += 360;
						}
						goalkeeper_state = INTERCEPTING;
					}
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
		if(get_angle_achieved()){
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
		set_rotation_speed(880, false);
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
			refocus_angle = RAD_TO_DEG(atan2f(FIELD_HEIGHT - get_ball_position()[1], FIELD_WIDTH/2 - get_ball_position()[0]));
			if(refocus_angle < 0){
				refocus_angle += 360;
			}

			attack_position[0] = get_ball_position()[0] - cosf(DEG_TO_RAD(refocus_angle))*ATTACKER_DISTANCE_TO_CHARGE_BALL;
			attack_position[1] = get_ball_position()[1] - sinf(DEG_TO_RAD(refocus_angle))*ATTACKER_DISTANCE_TO_CHARGE_BALL;

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
			set_rotation_speed(880, false);
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
		if(get_angle_achieved()){
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
				set_rotation_speed(880, false);
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

#define K1 2000.
#define K2 1000.
#define K3 0.3333333
#define K4 100.

int16_t attacking_score(int16_t* player_pos, int16_t* ball_pos, int16_t* goal_pos){
	//these functions where found with a simulation to get an attacking score that "seems" right, don't overthink it
	uint16_t dist_to_ball_score = 0; //if player is close to ball -> high attacking score
	uint16_t ball_dist_to_goal_score = 0; //if ball is close to enemy goal -> high attacking score
	int16_t alignement_player_ball_goal_score = 0; //if player is well aligned with the ball and the enemy goal -> high score

	dist_to_ball_score = (uint16_t)(K1 / sqrtf(distance_between(ball_pos, player_pos)));
	ball_dist_to_goal_score = (uint16_t)(K2 / distance_between(ball_pos, player_pos));
	float alignement_cos = 0.; //cos of the angle player-ball-goal
	//cosinus theorem:
	alignement_cos = (distance_squared(ball_pos, player_pos) + distance_squared(ball_pos, goal_pos) - distance_squared(player_pos,goal_pos)) /
					 (2*distance_between(ball_pos, player_pos)*distance_between(ball_pos, goal_pos));
	float temp_var = 0;
	temp_var = 2*acosf(alignement_cos) - PI;
	alignement_player_ball_goal_score = (int16_t)((K3 + dist_to_ball_score/K4) * temp_var * temp_var * temp_var);

	return dist_to_ball_score + ball_dist_to_goal_score + alignement_player_ball_goal_score;
}

float distance_between(int16_t* point1, int16_t* point2){
	float distance = 0;
	distance = sqrtf( (point1[0]-point2[0])*(point1[0]-point2[0]) + (point1[1]-point2[1])*(point1[1]-point2[1]) );
	return distance;
}

uint32_t distance_squared(int16_t* point1, int16_t* point2){
	uint32_t distance_squared = 0;
	distance_squared = (point1[0]-point2[0])*(point1[0]-point2[0]) + (point1[1]-point2[1])*(point1[1]-point2[1]);
	return distance_squared;
}
