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
#include <leds.h> //TODO:remove
#include <math.h>
#include <motors.h>

#include "motors_controller.h"

#define WHEEL_DIAMETER 41 //unit: mm
#define WHEELS_DIST 53.0 //distance unit: mm
#define STEPS_TO_DIST(steps) ((steps)*PI*WHEEL_DIAMETER/1000)

#define FULL_CIRCLE 360 //in degrees

#define DIRECTION_PRECISION 0.1 //deg
#define DIRECTION_SPEED_FINE_THRESHOL 5 //deg
#define DIRECTION_COARSE_TUNE_SPEED 400
#define DIRECTION_FINE_TUNE_SPEED 50

#define POSITION_PRECISION 1 //unit: mm
#define POSITION_SPEED_FINE_THRESHOL 5 //unit: mm
#define POSITION_COARSE_TUNE_SPEED 880
#define POSITION_FINE_TUNE_SPEED 50

#define KP 10 //proportional regulator

static float my_position[2] = {500, 500}; //(x,y) unit: mm
static float my_angle = 0; //unit: deg

static bool dir_obj_ach = true; //true if the objective(goal) direction has been achieved
static bool pos_obj_ach = true; //true if the objective(goal) position has been achieved

static float position_objective[2] = {0,0};
static float angle_objective = 0;


static THD_WORKING_AREA(waMotorsControl, 256);
static THD_FUNCTION(MotorsControl, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  left_motor_set_pos(0);
  right_motor_set_pos(0);

  int8_t lmotor_incr = 0; //left motor increment
  int8_t rmotor_incr = 0; //right motor increment
  int32_t lmotor_last_pos = 0; //
  int32_t rmotor_last_pos = 0;

  float angle_variation_temp = 0;

  float angle_difference = 0;
  float position_difference = 0;

  float angle_regulator = 0;

  while(true) {
	 //estimates the epuck position
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
	 if(dir_obj_ach == false){
		 angle_difference = get_angle_difference(my_angle, angle_objective);
		 //chprintf((BaseSequentialStream *) &SDU1, "pos_dif:%f- \n\r", angle_difference);
		 if(abs(angle_difference) < DIRECTION_PRECISION){
			 dir_obj_ach = true;
			 left_motor_set_speed(0);
			 right_motor_set_speed(0);
		 }else{
			 if(angle_difference > 0){
				 if(angle_difference > DIRECTION_SPEED_FINE_THRESHOL){
					 left_motor_set_speed(DIRECTION_COARSE_TUNE_SPEED);
					 right_motor_set_speed(-DIRECTION_COARSE_TUNE_SPEED);
				 }else{
					 left_motor_set_speed((int16_t)(DIRECTION_COARSE_TUNE_SPEED*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)));
					 right_motor_set_speed(-(int16_t)(DIRECTION_COARSE_TUNE_SPEED*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)));
				 }
			 }else{
				 if(angle_difference < -DIRECTION_SPEED_FINE_THRESHOL){
					 left_motor_set_speed(-DIRECTION_COARSE_TUNE_SPEED);
					 right_motor_set_speed(DIRECTION_COARSE_TUNE_SPEED);
				 }else{
					 left_motor_set_speed(-(int16_t)(DIRECTION_COARSE_TUNE_SPEED*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)));
					 right_motor_set_speed((int16_t)(DIRECTION_COARSE_TUNE_SPEED*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)*(angle_difference/DIRECTION_SPEED_FINE_THRESHOL)));
				 }
			 }
		 }
	 }else if(pos_obj_ach == false){
		 position_difference = get_position_difference(my_position, position_objective);
		 //chprintf((BaseSequentialStream *) &SDU1, "pos_dif:%f- \n\r", position_difference);
		 if(abs(position_difference) < POSITION_PRECISION){
			 pos_obj_ach = true;
			 left_motor_set_speed(0);
			 right_motor_set_speed(0);

		 }else{
			 angle_regulator = get_angle_difference(my_angle, RAD_TO_DEG(atan2f(position_objective[1]-my_position[1], position_objective[0]-my_position[0])));
			 if(position_difference > 0){
				 if(position_difference > POSITION_SPEED_FINE_THRESHOL){
					 left_motor_set_speed(POSITION_COARSE_TUNE_SPEED + KP*angle_regulator);
					 right_motor_set_speed(POSITION_COARSE_TUNE_SPEED - KP*angle_regulator);
				 }else{
					 left_motor_set_speed((int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
					 right_motor_set_speed((int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
				 }
			 }else{
				 if(position_difference < -POSITION_SPEED_FINE_THRESHOL){
					 left_motor_set_speed(-POSITION_COARSE_TUNE_SPEED - KP*angle_regulator);
					 right_motor_set_speed(-POSITION_COARSE_TUNE_SPEED + KP*angle_regulator);
				 }else{
					 left_motor_set_speed(-(int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
					 right_motor_set_speed(-(int16_t)(POSITION_COARSE_TUNE_SPEED*(position_difference/POSITION_SPEED_FINE_THRESHOL)*(position_difference/POSITION_SPEED_FINE_THRESHOL)));
				 }
			 }
		 }
	 }

	 chThdSleepMilliseconds(20); //50Hz
  }
}

float* get_self_position(void){
	return my_position;
}

float get_self_angle(void){
	return my_angle;
}

void shoot(void){

}

void set_angle(float angle){
	dir_obj_ach = false;
	angle_objective = angle;
}

void set_distance_forward(float distance){
	pos_obj_ach = false;
	position_objective[0] = my_position[0] + distance*cosf(DEG_TO_RAD(my_angle));
	position_objective[1] = my_position[1] + distance*sinf(DEG_TO_RAD(my_angle));
}

void set_position(float* position){
	dir_obj_ach = false;
	pos_obj_ach = false;
	angle_objective = RAD_TO_DEG(atan2f(position[1]-my_position[1], position[0]-my_position[0]));
	position_objective[0] = position[0];
	position_objective[1] = position[1];
}

float get_angle_difference(float angle1,float angle2){
	if(abs(angle1-angle2) < abs(fmod(angle1+180,360) - fmod(angle2+180,360)) ){
		return angle1 - angle2;
	}else{
		return fmod(angle1+180,360) - fmod(angle2+180,360);
	}
}

float get_position_difference(float* position1,float* position2){
	//fist we check if position2 is ahead of position1
	float angle_to_pos2 = 0; //angle from horizontal to line position1-position2
	angle_to_pos2 = RAD_TO_DEG(atan2f(position2[1]-position1[1], position2[0]-position1[0]));

	if(get_angle_difference(my_angle, angle_to_pos2) < 90  && get_angle_difference(my_angle, angle_to_pos2) > -90){ //position 2 ahead
		return sqrt((position2[0]-position1[0])*(position2[0]-position1[0]) + (position2[1]-position1[1])*(position2[1]-position1[1]));
	}else{
		return -sqrt((position2[0]-position1[0])*(position2[0]-position1[0]) + (position2[1]-position1[1])*(position2[1]-position1[1]));
	}
}

bool get_direction_achieved(void){
	return dir_obj_ach;
}

bool get_position_achieved(void){
	return pos_obj_ach;
}

void motors_controller_start(void){
	chThdCreateStatic(waMotorsControl, sizeof(waMotorsControl), NORMALPRIO+1, MotorsControl, NULL);
}
