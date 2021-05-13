/*
 * ir_sensors.c
 *
 *  Created on: May 13, 2021
 *      Author: rosendo
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "ir_sensors.h"
#include "sensors/proximity.h"
#include "stdbool"
#include "motors_controller.h"

#define CLOSE 20
#define INTERMEDIATE_POINT 50;

enum Ir_mvt{LOOK_AROUND, ROTATE, MOVE_POINT};

static enum Ir_mvt ir_state = LOOK_AROUND;

static THD_WORKING_AREA(waSensor_IR, 128);
static THD_FUNCTION(Sensor_IR, arg) {

	proximity_start();
	calibrate_ir();
bool test=0;

 /** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  uint8_t selector_position = 0;
  uint8_t sum=0;
  uint16_t intermediate_point[2]={0,0};
  uint16_t ir_vector[8]={0,0,0,0,0,0,0,0};
  uint16_t ir_vector_copy[8]={0,0,0,0,0,0,0,0};
  while(true) {
	  messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	  //chprintf((BaseSequentialStream *) &SD3, "x:%d \r\n", get_calibrated_prox(1));

	  sum=0;
	  for(uint8_t i=0; i<8; ++i){
		  if(get_calibrated_prox(i)<CLOSE){
			  ir_vector[i]=1;
		  	  }
		  else{
			  ir_vector[i]=0;
		  }
		  ir_vector_copy=ir_vector;
		  ir_vector[0]=ir_vector_copy[3];
		  ir_vector[1]=ir_vector_copy[2];
		  ir_vector[2]=ir_vector_copy[1];
		  ir_vector[3]=ir_vector_copy[8];
		  ir_vector[4]=ir_vector_copy[7];
		  ir_vector[5]=ir_vector_copy[6];
		  ir_vector[6]= 0;
	  }
	  switch(ir_state) {
	  			case LOOK_AROUND:{
	  				for(uint8_t i=0; i<6; ++i){
	  					sum+=ir_vector[i];
	  				}
	  				if(sum==1){ //if sum==0 goes directly to 0
	  					ir_state=ROTATE;
	  					set_angle_obj(get_angle_obj()+90);
	  				}
	  				else{
	  					if(sum>1){
	  						int bools=0;
	  						int first_ir=0;
	  						int second_ir=0;
	  						float first_angle=0.0;
	  						float second_angle=0.0;
	  		  				for(uint8_t i=0; i<6; ++i){ //enables to find the two ir that are more fare of
	  		  					if(bools==0 & ir_vector[i]==1){
	  		  						first_ir=i;
	  		  						bools=1;
	  		  					}
								if(bools==1 & ir_vector[i+1]==1){
									second_ir=i;
									bools=2;
								}
	  		  				}
							first_angle=RAD_TO_DEG(atan2f(-cosf(36*first_ir), sinf(36*i))+get_angle_obj(); //a verifier la forme, mais ici les angles sont en absolu
							second_angle=RAD_TO_DEG(atan2f(-cosf(36*first_ir), sinf(36*i))+get_angle_obj();
							if(first_angle < 0){
								first_angle += 360;
							}
							if(second_angle < 0){
								second_angle += 360;
							}
							int first_distance=0;
							int second_distance=0;
							//calcule les distances des points intermédiaires au point final
							//first_distance=pow((y_goal()-sinf(first_angle)*INTERMEDIATE_POINT))+pow((x_goal()-cosf(first_angle)*INTERMEDIATE_POINT));
							//second_distance=pow((y_goal()-sinf(second_angle)*INTERMEDIATE_POINT))+pow((x_goal()-cosf(second_angle)*INTERMEDIATE_POINT));

							if(first_distance>second_distance){
								ir_state=ROTATE;
								set_angle_obj(first_angle);
							}
							else{
								ir_state=ROTATE;
								set_angle_obj(second_angle);
							}

	  					}
	  				}//if none is activated, no obstavle in front so no problems

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
