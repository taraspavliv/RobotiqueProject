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

#define CLOSE_THRS 100 //if IR value larger than this, we set is as an "active sensor"
#define CLOSE_IN_FRONT_THRS 15 //if both front sensors see more than this value, there is an object right in front of the robot
#define INTERMEDIATE_POINT_DIST 40 // in mm
#define ANGLE_BETWEEN_SENSORS 36 //in degrees
#define FRONT_SENSORS 6
#define SMALL_TURN 5 //adds 5 degrees until he can find valid candidates
#define RELATIVE_ANGLE_SENSOR_0 (-90) // IR_active[0] is -90 degrees to the angle of the robot

bool intermediate_inside_field(int16_t* point); //returns true if the point is inside the field (FIELD_WIDTH and FIELD_HEIGHT are defined in main.h)
uint8_t update_IR_sensors(bool* IR_sensors, uint8_t* ext_active_sensors_idx); /*returns the number of active IR sensors. ext_active_sensors_idx is an array[2] and gets the indexes
of the most external active IR sensors (min active index, max active index). If only one active sensor, min active index = max active index*/

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

	//we make an array of sensors. We re-arrange them so that we can use the index of the sensor to find the relative angle
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

			switch(ir_state) {
			case LOOK_AROUND:{
				//if active sensors, we find two candidates for an intermediate point and choose the best one. Then we go to this point before continuing to the main objective
				if(active_sensors == 1){
					if(edge_active_sensors[0] != 0 && edge_active_sensors[0] != FRONT_SENSORS-1){ //look only for 1-4 active, 0 and FRONT_SENSORS-1 would mean there is an obstacle on the side, which doesn't bother
						//calculates the 2 candidates for intermediate points
						angle_to_candidate = get_self_angle() + RELATIVE_ANGLE_SENSOR_0 + edge_active_sensors[0]*ANGLE_BETWEEN_SENSORS + PERP;
						candidate_inter_point_1[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_1[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);

						angle_to_candidate = get_self_angle() + RELATIVE_ANGLE_SENSOR_0 + edge_active_sensors[0]*ANGLE_BETWEEN_SENSORS - PERP;
						candidate_inter_point_2[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_2[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						//chooses the best candidate
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
							//no candidate is good;
							set_angle_obj(get_self_angle() + SMALL_TURN);
							//turn until good
						}
					}
				}else if(active_sensors > 1){
					if(!(active_sensors == 2 && edge_active_sensors[0] == 0 && edge_active_sensors[1] == FRONT_SENSORS-1)){
						//calculates the 2 candidates for intermediate points
						angle_to_candidate = get_self_angle() + RELATIVE_ANGLE_SENSOR_0 + edge_active_sensors[0]*ANGLE_BETWEEN_SENSORS - PERP;
						candidate_inter_point_1[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_1[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);

						angle_to_candidate = get_self_angle() + RELATIVE_ANGLE_SENSOR_0 + edge_active_sensors[1]*ANGLE_BETWEEN_SENSORS + PERP;
						candidate_inter_point_2[0] = (int16_t)(get_self_position()[0] + cosf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						candidate_inter_point_2[1] = (int16_t)(get_self_position()[1] + sinf(DEG_TO_RAD(angle_to_candidate))*INTERMEDIATE_POINT_DIST);
						//chooses the best candidate
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
							//no candidate is good;
							set_angle_obj(get_self_angle() + SMALL_TURN);
							//turn until he can find a solution
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
	//re-arrangemenets of the sensors. IR_sensors[0] is the sensor on the right, IR_sensors[5] is the sensor on the left
	//sums the number of active sensors, and find the smallest and biggest index of active sensors
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
	//checks if the intermediate point is inside the field
	if(point[0] < FIELD_WIDTH - EPUCK_DIAMETER/2 && point[0] > EPUCK_DIAMETER/2 && point[1] < FIELD_HEIGHT- EPUCK_DIAMETER/2 && point[1] > EPUCK_DIAMETER/2){
		return true;
	}else{
		return false;
	}
}

void set_avoid_collision(bool avoid_param){
	avoid_collision = avoid_param;
}

bool object_right_in_front(void){
	if(get_calibrated_prox(0)>10 && get_calibrated_prox(7)>10){
		return true;
	}else{
		return false;
	}
}

void collision_manager_start(void){
	chThdCreateStatic(waCollisionManager, sizeof(waCollisionManager), NORMALPRIO+1, CollisionManager, NULL);
}
