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

#define CORNER_DISTANCE 150 //uncalibrated distance in mm to each wall
#define AVG_MOV_K 0.01
#define MINIMAL_DIST 100000.0 //a minimal distance big enough needs to be set as initialization
#define WALL_X_ANGLE 270
#define WALL_Y_ANGLE 180
#define CALIBRATION_ANGLE 160 //the robot turns almost to the Y wall and starts scanning
#define CALIBRATION_SPEED 20 //the speed of the robot while scanning the wall
#define THREAD_SLEEP_YES 5 //if a calibration is done, then 5 ms is a good value(found by trial and error)
#define THREAD_SLEEP_NO 500

enum CalibState{POSITIONING, TURNING_TO_WALL_Y, CALIBRATING_PHASE_1, TURNING_TO_WALL_X, CALIBRATING_PHASE_2};

static enum CalibState calibration_state = POSITIONING;
static bool calibration_in_progress = false;

static thread_t *distThd;
static uint16_t dist_mm = 0;
static bool VL53L0X_configured = false;

static THD_WORKING_AREA(waposition_calibrator, 512);
static THD_FUNCTION(position_calibrator, arg) {

	 // start of the calibration function

	 //declaration of variables

	float avg_dist_mm = 0.0;
	float minimal_distance = MINIMAL_DIST;
	int16_t min_x = 0;
	int16_t min_y = 0;

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
//start of the loop
    /* Reader thread loop.*/
    while (chThdShouldTerminateX() == false) {
    	if(VL53L0X_configured){
    		VL53L0X_getLastMeasure(&device);
   			dist_mm = device.Data.LastRangeMeasure.RangeMilliMeter;
    	}
    	avg_dist_mm = avg_dist_mm*(1-AVG_MOV_K) + AVG_MOV_K*dist_mm;
    	if(calibration_in_progress){
			switch(calibration_state) {
			case POSITIONING:{
				if(get_position_achieved()){
					set_angle_obj(CALIBRATION_ANGLE);
					calibration_state=TURNING_TO_WALL_Y;
				}
				break;
			}
			case TURNING_TO_WALL_Y: {
				if(get_angle_achieved()){
					right_motor_set_speed(CALIBRATION_SPEED);//the robot starts rotating
					left_motor_set_speed(-CALIBRATION_SPEED);
					minimal_distance = MINIMAL_DIST;
					calibration_state=CALIBRATING_PHASE_1;
				}
				break;
			}
			case CALIBRATING_PHASE_1:{
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
				if(get_angle_achieved()){
					minimal_distance = MINIMAL_DIST;
					calibration_state=CALIBRATING_PHASE_2;
				}
				break;
			}
			case CALIBRATING_PHASE_2: {
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
			chThdSleepMilliseconds(THREAD_SLEEP_YES);
    	}else{
    		chThdSleepMilliseconds(THREAD_SLEEP_NO);
    	}
    }
}

void position_calibrator_start(void){
		if(VL53L0X_configured) {
		return;
	}

	i2c_start();

	distThd = chThdCreateStatic(waposition_calibrator,
                     sizeof(waposition_calibrator),
                     NORMALPRIO + 10,
                     position_calibrator,
                     NULL);
}

void calibrate(void){
	int16_t calibration_point[2] = {CORNER_DISTANCE, CORNER_DISTANCE};
	set_position_obj(calibration_point);
	calibration_in_progress = true;
	calibration_state = POSITIONING;
}
