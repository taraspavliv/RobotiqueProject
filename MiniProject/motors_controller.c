/*
 * position_motion_controller.c
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <motors.h>

#include "motors_controller.h"

#define WHEEL_DIAMETER 410 //unit: 10^⁻4 m
#define WHEELS_DIST 530.0 //distance unit: 10^⁻4 m
#define STEPS_TO_DIST(steps) (int16_t)((steps)*PI*WHEEL_DIAMETER/1000)

#define FULL_CIRCLE 36000 //in centi-degrees
#define RAD_TO_cDEG(radians) (int32_t)(radians*18000/PI) //converts radians to centi-degree
#define cDEG_TO_RAD(centideg) (centideg)*PI/18000  //converts centi-degree to radians

static uint32_t my_position[2] = {5000, 5000}; //(x,y) unit: 10^-4 m
static uint16_t my_angle = 0; //unit: 10^-2 deg

static bool action_done = true;

static THD_WORKING_AREA(waMotorsControl, 256);
static THD_FUNCTION(MotorsControl, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  left_motor_set_pos(0);
  right_motor_set_pos(0);

  int16_t lmotor_incr = 0; //left motor increment
  int16_t rmotor_incr = 0; //right motor increment
  int16_t lmotor_last_pos = 0; //
  int16_t rmotor_last_pos = 0;

  int16_t angle_variation_temp = 0;

  while(true) {
	 //estimates the epuck position
	 lmotor_incr = left_motor_get_pos() - lmotor_last_pos;
	 rmotor_incr = right_motor_get_pos() - rmotor_last_pos;

	 my_position[0] += STEPS_TO_DIST(lmotor_incr+rmotor_incr) * cosf(cDEG_TO_RAD(my_angle))/2;
	 my_position[1] += STEPS_TO_DIST(lmotor_incr+rmotor_incr) * sinf(cDEG_TO_RAD(my_angle))/2;
	 angle_variation_temp = RAD_TO_cDEG(atanf( STEPS_TO_DIST(rmotor_incr-lmotor_incr)/WHEELS_DIST ));
	 if(my_angle + angle_variation_temp < 0){
		 my_angle = my_angle + angle_variation_temp + FULL_CIRCLE;
	 }else if(my_angle + angle_variation_temp > FULL_CIRCLE){
		 my_angle = my_angle + angle_variation_temp - FULL_CIRCLE;
	 }else{
		 my_angle += angle_variation_temp;
	 }
	 lmotor_last_pos = left_motor_get_pos();
	 rmotor_last_pos = right_motor_get_pos();



	 chThdSleepMilliseconds(20); //50Hz
  }
}

uint32_t* get_self_position(void){
	return my_position;
}

uint16_t get_self_angle(void){
	return my_angle/100;
}

void shoot(void){

}

void set_direction(void){

}

void motors_controller_start(void){
	chThdCreateStatic(waMotorsControl, sizeof(waMotorsControl), NORMALPRIO+1, MotorsControl, NULL);
}
