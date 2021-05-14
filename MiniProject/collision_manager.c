/*
 * collision_manager.c
 *
 *  Created on: May 13, 2021
 *      Author: rosendo
 */
#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include "main.h"

#include <math.h>
#include <motors_controller.h>
#include <collision_manager.h>
#include "role_selector.h"
//TODO
#include <chprintf.h>

#define CLOSE_THRS 50
#define INTERMEDIATE_POINT_DIST 50 // in mm
#define ANGLE_BETWEEN_SENSORS 36 //in deg
#define FRONT_SENSORS 6

enum Ir_mvt{LOOK_AROUND, GOING_TO_INTERMEDIATE};

messagebus_t bus;

static enum Ir_mvt ir_state = LOOK_AROUND;
static bool avoid_collision = true;

static THD_WORKING_AREA(waCollisionManager, 512);
static THD_FUNCTION(CollisionManager, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	calibrate_ir();

	bool IR_active[FRONT_SENSORS] = {0};
	uint8_t active_sensors = 0;
	uint8_t edge_active_sensors[2] = {0};

	int16_t angle_to_candidate = 0;
	int16_t candidate_inter_point_1[2] = {0};
	int16_t candidate_inter_point_2[2] = {0};

	while(true) {
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

		if(get_position_achieved() == false && get_angle_achieved() == true && avoid_collision == true){
			active_sensors = update_IR_sensors(IR_active, edge_active_sensors);
			//chprintf((BaseSequentialStream *) &SD3, "0:%d 1:%d 2:%d 3:%d 4:%d 5:%d \r\n", IR_active[0], IR_active[1], IR_active[2], IR_active[3], IR_active[4], IR_active[5]);

			switch(ir_state) {
			case LOOK_AROUND:{
				if(active_sensors == 1){ //look only for 1-4 active
					//set_angle_obj(get_angle_obj()+90);//or -90, depending on what is closest to objective
					if(edge_active_sensors[0] != 0 && edge_active_sensors[0] != FRONT_SENSORS-1){
						angle_to_candidate = get_self_angle() - 90 + edge_active_sensors[0]*ANGLE_BETWEEN_SENSORS + 90;
						candidate_inter_point_1[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_1[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);

						angle_to_candidate = get_self_angle() - 90 + edge_active_sensors[0]*ANGLE_BETWEEN_SENSORS - 90;
						candidate_inter_point_2[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_2[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);

						if(intermediate_inside_field(candidate_inter_point_1)){
							if(intermediate_inside_field(candidate_inter_point_2)){
								if(closest_point_to_obj(candidate_inter_point_1, candidate_inter_point_2) == 1){
									set_intermediate_point(candidate_inter_point_1);
									ir_state = GOING_TO_INTERMEDIATE;
								}else{
									set_intermediate_point(candidate_inter_point_2);
									ir_state = GOING_TO_INTERMEDIATE;
								}
							}else{
								set_intermediate_point(candidate_inter_point_1);
								ir_state = GOING_TO_INTERMEDIATE;
							}
						}else if(intermediate_inside_field(candidate_inter_point_2)){
							set_intermediate_point(candidate_inter_point_2);
							ir_state = GOING_TO_INTERMEDIATE;
						}else{
							//TODO
							//no candidate is good;
							//random turn?
						}
					}
				}else if(active_sensors > 1){
					if(!(active_sensors == 2 && edge_active_sensors[0] == 0 && edge_active_sensors[1] == FRONT_SENSORS-1)){
						angle_to_candidate = get_self_angle() - 90 + edge_active_sensors[0]*ANGLE_BETWEEN_SENSORS - 90;
						candidate_inter_point_1[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_1[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);

						angle_to_candidate = get_self_angle() - 90 + edge_active_sensors[1]*ANGLE_BETWEEN_SENSORS + 90;
						candidate_inter_point_2[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_2[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);

						if(intermediate_inside_field(candidate_inter_point_1)){
							if(intermediate_inside_field(candidate_inter_point_2)){
								if(closest_point_to_obj(candidate_inter_point_1, candidate_inter_point_2) == 1){
									set_intermediate_point(candidate_inter_point_1);
									ir_state = GOING_TO_INTERMEDIATE;
								}else{
									set_intermediate_point(candidate_inter_point_2);
									ir_state = GOING_TO_INTERMEDIATE;
								}
							}else{
								set_intermediate_point(candidate_inter_point_1);
								ir_state = GOING_TO_INTERMEDIATE;
							}
						}else if(intermediate_inside_field(candidate_inter_point_2)){
							set_intermediate_point(candidate_inter_point_2);
							ir_state = GOING_TO_INTERMEDIATE;
						}else{
							//TODO
							//no candidate is good;
							//random turn?
						}
					}
				} //else no ir is active, so no obstacle in front so no problems
				break;
			}
			case GOING_TO_INTERMEDIATE:{
				if(get_intermediate_achieved()){
					continue_main_obj();
					ir_state = LOOK_AROUND;
				}
				break;
			}
			default: break;
		  }
		}
	  chThdSleepMilliseconds(20);
  }
}

uint8_t update_IR_sensors(bool* IR_sensors, uint8_t* ext_active_sensors_idx){
	IR_sensors[0] = (get_calibrated_prox(2)>CLOSE_THRS);
	IR_sensors[1] = (get_calibrated_prox(1)>CLOSE_THRS);
	IR_sensors[2] = (get_calibrated_prox(0)>CLOSE_THRS);
	IR_sensors[3] = (get_calibrated_prox(7)>CLOSE_THRS);
	IR_sensors[4] = (get_calibrated_prox(6)>CLOSE_THRS);
	IR_sensors[5] = (get_calibrated_prox(5)>CLOSE_THRS);
	uint8_t sum = 0;
	bool first_active_sensor_found = false;
	for(uint8_t i = 0; i < FRONT_SENSORS; ++i){
		if(IR_sensors[i] == true){
			++sum;
			if(first_active_sensor_found == false){
				first_active_sensor_found = true;
				ext_active_sensors_idx[0] = i;
			}
			ext_active_sensors_idx[1] = i;
		}
	}
	return sum;
}

bool intermediate_inside_field(int16_t* point){
	if(point[0] < FIELD_WIDTH && point[0] > 0 && point[1] < FIELD_HEIGHT && point[1] > 0){
		return true;
	}else{
		return false;
	}
}

void set_avoid_collision(bool avoid_param){
	avoid_collision = avoid_param;
}

void collision_manager_start(void){
	chThdCreateStatic(waCollisionManager, sizeof(waCollisionManager), NORMALPRIO+1, CollisionManager, NULL);
}
