/*
 * ir_sensors.c
 *
 *  Created on: May 13, 2021
 *      Author: rosendo
 */

#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "ir_sensors.h"
#include "sensors/proximity.h"

#define CLOSE 20

enum Ir_mvt{LOOK_AROUND, ROTATE, MOVE_POINT};

static enum Ir_mvt ir_state = LOOK_AROUND;

static THD_WORKING_AREA(waSensor_IR, 128);
static THD_FUNCTION(Sensor_IR, arg) {

	proximity_start();
	calibrate_ir();

 /** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  uint8_t selector_position = 0;
  uint16_t intermediate_point[2]={0,0};
  uint8_t ir_vector[8]={0,0,0,0,0,0,0,0};
  while(true) {
	  messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	  //chprintf((BaseSequentialStream *) &SD3, "x:%d \r\n", get_calibrated_prox(1));
	  
	  if(get_calibrated_prox(0)<CLOSE){
		  ir_vector[0]=1;
	  }
	  if(get_calibrated_prox(1)<CLOSE){
		  ir_vector[1]=1;
	  }
	  if(get_calibrated_prox(2)<CLOSE){
		  ir_vector[2]=1;
	  }
	  if(get_calibrated_prox(3)<CLOSE){
		  ir_vector[3]=1;
	  }
	  if(get_calibrated_prox(4)<CLOSE){
		  ir_vector[4]=1;
	  }
	  if(get_calibrated_prox(5)<CLOSE){
		  ir_vector[5]=1;
	  }
	  if(get_calibrated_prox(6)<CLOSE){
		  ir_vector[6]=1;
	  }
	  if(get_calibrated_prox(7)<CLOSE){
		  ir_vector[7]=1;
	  }
	  
	  switch(ir_state) {
	  			case LOOK_AROUND:{
	  			
	  			break;
	  			}
	  			case ROTATE:{
	  				if(get_direction_achieved){
	  					ir_state= MOVE_POINT;
	  					intermediate_point[0]=1;
	  					intermediate_point[1]=2;
	  					set_position_obj(intermediate_point);
	  				}			
				break;
				}
	  			case MOVE_POINT:{
	  				if(get_position_achieved()){
	  					ir_state= LOOK_AROUND;
	  				}			
				break;
				}
	  			default: break;
	  }
	  //doesn't need to be very reactive, so long sleep time
	  chThdSleepMilliseconds(500);
  }
}


void role_selector_start(void){
	chThdCreateStatic(waSensor_IR, sizeof(waSensor_IR), NORMALPRIO-1, Sensor_IR, NULL);
}

