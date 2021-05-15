/*
 * position_calibrator.c
 * modified VL53L0X.c
 *  Created on: May 8, 2021
 *      Author: rosendo
 */
#include <position_calibrator.h>
#include "ch.h"
#include "hal.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/VL53L0X/Api/core/inc/vl53l0x_api.h"
#include "shell.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "usbcfg.h"
#include "motors_controller.h"
#include "motors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include <stdbool.h>

#define CORNER_DISTANCE 150 //uncalibrated distance in mm to each wall
#define AVG_MOV_K 0.01
#define MINIMAL_DIST 100000.0 //a minimal distance big enough needs to be set as initialization
#define WALL_X_ANGLE 270 //to calibrate on the Y axis, it turns to the wall along the X axis. It should be at 270 degrees
#define WALL_Y_ANGLE 180 //to calibrate on the X axis, it turns to the wall along the Y axis. It should be at 180 degrees
#define CALIBRATION_ANGLE 160 //the robot turns almost to the Y wall and starts scanning (in case there is an error on the angle, we start scannin before the WALL_Y_ANGLE)
#define CALIBRATION_SPEED 20 //the speed of the robot while scanning the wall

enum CalibState{POSITIONING, TURNING_TO_WALL_Y, CALIBRATING_PHASE_1, TURNING_TO_WALL_X, CALIBRATING_PHASE_2};

static enum CalibState calibration_state = POSITIONING;
static bool calibration_in_progress = false;

static thread_t *distThd;
static uint16_t dist_mm = 0;
static bool VL53L0X_configured = false;

static THD_WORKING_AREA(waposition_calibrator, 512);
static THD_FUNCTION(position_calibrator, arg) {

	chRegSetThreadName("VL53L0x Thd");
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	(void)arg;
	static VL53L0X_Dev_t device;
	device.I2cDevAddr = VL53L0X_ADDR;
	
	status = VL53L0X_init(&device);

	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configAccuracy(&device, VL53L0X_HIGH_ACCURACY); //the TOF is set in Accuracy mode
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_startMeasure(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	}
	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configured = true;
	}

	float avg_dist_mm = 0.0; //averaged measured distance
	float minimal_distance = MINIMAL_DIST; //the minimum distance found
	//min_x and min_y will be used to update the absolute coordinates once the calibration is done
	int16_t min_x = 0;
	int16_t min_y = 0;

    /* Reader thread loop.*/
    while (chThdShouldTerminateX() == false) {
    	if(VL53L0X_configured){
    		//reads the value
    		VL53L0X_getLastMeasure(&device);
   			dist_mm = device.Data.LastRangeMeasure.RangeMilliMeter;
    	}
    	avg_dist_mm = avg_dist_mm*(1-AVG_MOV_K) + AVG_MOV_K*dist_mm;
    	if(calibration_in_progress){
    		 //start of the calibration Finite State Machine
			switch(calibration_state) {
			case POSITIONING:{
				//gets to the corner to start the calibration
				if(get_position_achieved()){
					set_angle_obj(CALIBRATION_ANGLE);
					calibration_state=TURNING_TO_WALL_Y;
				}
				break;
			}
			case TURNING_TO_WALL_Y: {
				//he turns to the starting angle
				if(get_angle_achieved()){
					//the robot starts rotating to find a local minimum, which will be the x coordinates
					set_rotation_speed(CALIBRATION_SPEED, false);
					minimal_distance = MINIMAL_DIST;
					calibration_state=CALIBRATING_PHASE_1;
				}
				break;
			}
			case CALIBRATING_PHASE_1:{
				//once there he starts to "scan" the wall. When there will a local minimum, it means the robot is perpendicular to the wall
				//so we can correct the angle and the x position
				if(avg_dist_mm< minimal_distance){
					minimal_distance=avg_dist_mm;
				}else if(avg_dist_mm > (minimal_distance)){
					min_x = (int16_t)minimal_distance;
					set_angle_calib(WALL_Y_ANGLE);
					set_angle_obj(WALL_X_ANGLE);
					calibration_state=TURNING_TO_WALL_X;
				}
				break;
			}
			case TURNING_TO_WALL_X: {
				//he turns to the other wall to calibrate along the y axis
				if(get_angle_achieved()){
					minimal_distance = MINIMAL_DIST;
					calibration_state=CALIBRATING_PHASE_2;
				}
				break;
			}
			case CALIBRATING_PHASE_2: {
				//once the measure stabilizes, we update the (x,y) values
				if(avg_dist_mm< minimal_distance){
					minimal_distance=avg_dist_mm;
				}else if(avg_dist_mm > minimal_distance){
					min_y = (int16_t)minimal_distance;
					set_position_calib(min_x, min_y);
					calibration_in_progress = false;
				}
				break;
			}
			}
			chThdSleepMilliseconds(5); //needs to be very reactive to not miss the minimum point
    	}else{
    		chThdSleepMilliseconds(500); //long sleep if not calibrating
    	}
    }
}

void position_calibrator_start(void){
		if(VL53L0X_configured) {
		return;
	}

	i2c_start();
	distThd = chThdCreateStatic(waposition_calibrator, sizeof(waposition_calibrator), NORMALPRIO + 2, position_calibrator, NULL);
}

void calibrate(void){
	int16_t calibration_point[2] = {CORNER_DISTANCE, CORNER_DISTANCE};
	set_position_obj(calibration_point);
	calibration_in_progress = true;
	calibration_state = POSITIONING;
}

int get_is_calibrating(void){
	return calibration_in_progress;
}
