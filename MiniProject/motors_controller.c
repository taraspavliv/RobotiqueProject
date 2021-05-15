/*
 * position_motion_controller.c
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */
#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <motors.h>

#include "motors_controller.h"

#define WHEEL_DIAMETER 41 //unit: mm
#define WHEELS_DIST 52.0 //distance between wheel unit: mm
#define STEPS_TO_DIST(steps) ((steps)*PI*WHEEL_DIAMETER/1000)

#define DIRECTION_PRECISION 0.1 //in degrees
#define DIRECTION_SPEED_FINE_THRESHOL 5 //in degrees, if the error on angle is smaller than that the rotations slows down
#define DIRECTION_COARSE_TUNE_SPEED 400 //the speed if the error on the angle is still big (>DIRECTION_SPEED_FINE_THRESHOL) (coarse angle tuning)

#define POSITION_PRECISION 2 //unit: mm
#define POSITION_SPEED_FINE_THRESHOL 5 //unit: mm, if the position error is smaller than that the epucks slow down
#define POSITION_COARSE_TUNE_SPEED 880 //the speed if the position error is still big (>POSITION_SPEED_FINE_THRESHOL) (coarse position tuning)

#define BT_CONTROL_K 8 //coefficient found to convert from joystick command to motor movement

#define KP 10 //proportional regulator constant, for position control

bool manage_angle(float angle_objective); //controls the motors based on objective and the current position, returns true if the angle has been achieved (within DIRECTION_PRECISION of the objective)
bool manage_position(float* position_objective); //controls the motors based on objective and the current position, returns true if the position has been achieved (within POSITION_PRECISION of the objective)

float get_position_difference(float* position1,float* position2); //return the distance from one point to the other, the first position is expected to be the position of the epuck
//it gives a negative distance if the position is behind the position1 (based on the angle of the epuck)

static float my_position[2] = {200, 200}; //(x,y) unit: mm
static float my_angle = 0; //unit: degree

static bool angle_obj_ach = true; //flag,true if the objective(goal) direction has been achieved
static bool pos_obj_ach = true; //flag,true if the objective(goal) position has been achieved
static bool interm_ach = true; //flag,true if the intermediate position has been achieved

static float position_objective[2] = {0,0};
static float angle_objective = 0;
static float intermediate_position[2] = {0,0};

static THD_WORKING_AREA(waMotorsControl, 256);
static THD_FUNCTION(MotorsControl, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  left_motor_set_pos(0);
  right_motor_set_pos(0);

  int8_t lmotor_incr = 0; //left motor increment
  int8_t rmotor_incr = 0; //right motor increment
  int32_t lmotor_last_pos = 0; //for integration
  int32_t rmotor_last_pos = 0; //for integration

  float angle_variation_temp = 0;

  while(true) {
	 //estimates the epuck position by integrating the motor movement
	 lmotor_incr = left_motor_get_pos() - lmotor_last_pos;
	 rmotor_incr = right_motor_get_pos() - rmotor_last_pos;

	 my_position[0] += STEPS_TO_DIST(lmotor_incr+rmotor_incr) * cosf(DEG_TO_RAD(my_angle))/2;
	 my_position[1] += STEPS_TO_DIST(lmotor_incr+rmotor_incr) * sinf(DEG_TO_RAD(my_angle))/2;

	 angle_variation_temp = RAD_TO_DEG(atanf( STEPS_TO_DIST(rmotor_incr-lmotor_incr)/WHEELS_DIST ));
	 if(my_angle + angle_variation_temp < 0){
		 my_angle = my_angle + angle_variation_temp + FULL_CIRCLE;
	 }else if(my_angle + angle_variation_temp > FULL_CIRCLE){
		 my_angle = my_angle + angle_variation_temp - FULL_CIRCLE;
	 }else{
		 my_angle += angle_variation_temp;
	 }
	 lmotor_last_pos = left_motor_get_pos();
	 rmotor_last_pos = right_motor_get_pos();

	 //handles objectives achievement
	 //first we set the right angle, then we go to the objective position.
	 if(angle_obj_ach == false){
		 angle_obj_ach = manage_angle(angle_objective);
	 }else if(interm_ach == false){
		 //if there is an obstacle, we go to the intermediate point before going to the final objective
		 interm_ach = manage_position(intermediate_position);
	 }else if(pos_obj_ach == false){
		 pos_obj_ach = manage_position(position_objective);
	 }

	 chThdSleepMilliseconds(20); //needs to be quite responsive
  }
}

bool manage_angle(float angle_objective){
	 float angle_difference = 0;
	 angle_difference = get_angle_difference(my_angle, angle_objective);

	 if(abs(angle_difference) < DIRECTION_PRECISION){
		 //if the angle is close enough to the objective, we return true
		 left_motor_set_speed(0);
		 right_motor_set_speed(0);
		 return true;
	 }else{
		 //otherwise we choose the fastest way to turn to the objective
		 if(angle_difference > 0){
			 if(angle_difference > DIRECTION_SPEED_FINE_THRESHOL){
				 left_motor_set_speed(DIRECTION_COARSE_TUNE_SPEED);
				 right_motor_set_speed(-DIRECTION_COARSE_TUNE_SPEED);
			 }else{
				 //if it getting close, we slow down quadratically, otherwise it oscillates around the objective without ever getting close enough to see as "angle achieved"
				 left_motor_set_speed((int16_t)( DIRECTION_COARSE_TUNE_SPEED * (angle_difference/DIRECTION_SPEED_FINE_THRESHOL) * (angle_difference/DIRECTION_SPEED_FINE_THRESHOL) ));
				 right_motor_set_speed(-(int16_t)( DIRECTION_COARSE_TUNE_SPEED * (angle_difference/DIRECTION_SPEED_FINE_THRESHOL) * (angle_difference/DIRECTION_SPEED_FINE_THRESHOL) ));
			 }
		 }else{
			 if(angle_difference < -DIRECTION_SPEED_FINE_THRESHOL){
				 left_motor_set_speed(-DIRECTION_COARSE_TUNE_SPEED);
				 right_motor_set_speed(DIRECTION_COARSE_TUNE_SPEED);
			 }else{
				 //if it getting close, we slow down quadratically, otherwise it oscillates around the objective without ever getting close enough to see as "angle achieved"
				 left_motor_set_speed(-(int16_t)(DIRECTION_COARSE_TUNE_SPEED*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)));
				 right_motor_set_speed((int16_t)(DIRECTION_COARSE_TUNE_SPEED*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)));
			 }
		 }
		 return false;
	 }
}

bool manage_position(float* position_objective){
	float position_difference = 0;
	position_difference = get_position_difference(my_position, position_objective);
	if(abs(position_difference) < POSITION_PRECISION){
		//if the position is close enough to the objective, we return true
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		return true;
	}else{
		//otherwise we choose the direction to get closer to the objective
		float angle_regulator = 0;
		//we add a proportional regulator based on angle because a small error on angle might give a large error on position if the distance is large enough
		angle_regulator = get_angle_difference(my_angle, RAD_TO_DEG(atan2f(position_objective[1]-my_position[1], position_objective[0]-my_position[0])));
		if(position_difference > 0){
			if(position_difference > POSITION_SPEED_FINE_THRESHOL){
				left_motor_set_speed(POSITION_COARSE_TUNE_SPEED + KP*angle_regulator);
				right_motor_set_speed(POSITION_COARSE_TUNE_SPEED - KP*angle_regulator);
			}else{
				//if it is getting close, we slow down quadratically, otherwise it oscillates around the objective without ever getting close enough to see as "position achieved"
				left_motor_set_speed((int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
				right_motor_set_speed((int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
			}
		}else{
			if(position_difference < -POSITION_SPEED_FINE_THRESHOL){
				left_motor_set_speed(-POSITION_COARSE_TUNE_SPEED - KP*angle_regulator);
				right_motor_set_speed(-POSITION_COARSE_TUNE_SPEED + KP*angle_regulator);
			}else{
				//if it is getting close, we slow down quadratically, otherwise it oscillates around the objective without ever getting close enough to see as "position achieved"
				left_motor_set_speed(-(int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
				right_motor_set_speed(-(int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
			}
		}
		return false;
	}
}
//getters
int16_t* get_self_position(void){
    static int16_t position_in_int[2] = {0};
    position_in_int[0] = (int16_t)my_position[0];
    position_in_int[1] = (int16_t)my_position[1];
    return position_in_int;
}

int16_t get_self_angle(void){
	return (int16_t)my_angle;
}

bool get_angle_achieved(void){
	return angle_obj_ach;
}

bool get_position_achieved(void){
	return pos_obj_ach;
}

//setters for objectives
void set_angle_obj(uint16_t angle){
	angle_obj_ach = false;
	angle_objective = (float)angle;
}

void set_distance_forward(float distance){
	pos_obj_ach = false;
	position_objective[0] = my_position[0] + distance*cosf(DEG_TO_RAD(my_angle));
	position_objective[1] = my_position[1] + distance*sinf(DEG_TO_RAD(my_angle));
}

void set_position_obj(int16_t* position){
	/*we set the angle to the objective and the position:
	*the threads first sets the right angle to go to the objective position
	then uses the position objective*/
	angle_obj_ach = false;
	pos_obj_ach = false;
	angle_objective = RAD_TO_DEG(atan2f((float)(position[1])-my_position[1], (float)(position[0])-my_position[0]));
	position_objective[0] = position[0];
	position_objective[1] = position[1];
}

void set_rotation_speed(uint16_t speed, bool clockwise){
	angle_obj_ach = true;
	pos_obj_ach = true;
	if(clockwise){
		left_motor_set_speed(speed);
		right_motor_set_speed(-speed);
	}else{
		left_motor_set_speed(-speed);
		right_motor_set_speed(speed);
	}
}

//intermediate point to avoid collisions
uint8_t closest_point_to_obj(int16_t* point1, int16_t* point2){
	//if there is 2 candidates for the intermediate point, we choose the one closer to the main objective
	if( (position_objective[0] - point1[0])*(position_objective[0] - point1[0]) + (position_objective[1] - point1[1])*(position_objective[1] - point1[1]) <=
		(position_objective[0] - point2[0])*(position_objective[0] - point2[0]) + (position_objective[1] - point2[1])*(position_objective[1] - point2[1])	){
		return 1;
	}else{
		return 2;
	}
}

void set_intermediate_point(int16_t* position){
	interm_ach = false;
	angle_obj_ach = false;
	angle_objective = RAD_TO_DEG(atan2f((float)(position[1])-my_position[1], (float)(position[0])-my_position[0]));
	intermediate_position[0] = position[0];
	intermediate_position[1] = position[1];
}

void continue_main_obj(void){
	//after the intermediate point has been achieved, we go again to the main objective position
	int16_t position[2] = {0};
	position[0] = position_objective[0];
	position[1] = position_objective[1];
	set_position_obj(position);
}

bool get_intermediate_achieved(void){
	return interm_ach;
}

//for error estimation
float get_angle_difference(float angle1,float angle2){
	if(angle2 < 0){
		angle2 += FULL_CIRCLE;
	}
	if(angle1 < 0){
		angle1 += FULL_CIRCLE;
	}
	if(abs(angle1-angle2) < abs(fmod(angle1+HALF_CIRCLE,FULL_CIRCLE) - fmod(angle2+HALF_CIRCLE,FULL_CIRCLE)) ){
		return angle1 - angle2;
	}else{
		return fmod(angle1+HALF_CIRCLE,FULL_CIRCLE) - fmod(angle2+HALF_CIRCLE,FULL_CIRCLE);
	}
}

float get_position_difference(float* position1,float* position2){
	//fist we check if position2 is ahead of position1
	float angle_to_pos2 = 0; //angle from horizontal to line position1-position2
	angle_to_pos2 = RAD_TO_DEG(atan2f(position2[1]-position1[1], position2[0]-position1[0]));

	if(get_angle_difference(my_angle, angle_to_pos2) < PERP  && get_angle_difference(my_angle, angle_to_pos2) > -PERP){
		 //position 2 ahead
		return sqrt((position2[0]-position1[0])*(position2[0]-position1[0]) + (position2[1]-position1[1])*(position2[1]-position1[1]));
	}else{
		return -sqrt((position2[0]-position1[0])*(position2[0]-position1[0]) + (position2[1]-position1[1])*(position2[1]-position1[1]));
	}
}

void motor_shoot(void){
	left_motor_set_speed(MOTOR_SPEED_LIMIT);
	right_motor_set_speed(MOTOR_SPEED_LIMIT);
}

void reset_motor_shoot(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void control_motors_BT(uint16_t* joystick_polar_coord){
	//when epuck is controlled over bluetooth, controls the motors based on the joystick commands
	uint8_t joystick_distance = (uint8_t)joystick_polar_coord[0];
	float angle_rad = DEG_TO_RAD(joystick_polar_coord[1]);

	left_motor_set_speed(BT_CONTROL_K*joystick_distance*(cos(angle_rad)+sin(angle_rad)));
	right_motor_set_speed(BT_CONTROL_K*joystick_distance*(cos(angle_rad)-sin(angle_rad)));
}

//once calibration is done, we need to update the current values with the measured one
void set_angle_calib(uint16_t angle_calib){
	my_angle = (float)angle_calib;
}

void set_position_calib(int16_t position_x, int16_t position_y){
	my_position[0] = position_x;
	my_position[1] = position_y;
}

void motors_controller_start(void){
	chThdCreateStatic(waMotorsControl, sizeof(waMotorsControl), NORMALPRIO+1, MotorsControl, NULL);
}
