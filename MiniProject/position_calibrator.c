/*
 * position_calibrator.c
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */
#include <position_calibrator.h>
#include "ch.h"
#include "hal.h"
#include "VL53L0X.h"
#include "Api/core/inc/vl53l0x_api.h"
#include "shell.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "usbcfg.h"
#include "motors.h"

static thread_t *distThd;
static THD_WORKING_AREA(waposition_calibrator, 512);
static THD_FUNCTION(position_calibrator, arg) {

	 // start of the calibration function

	 //declaration of variables
	    int last_distance=1;
	    int new_distance=0;
	    int limit=0;
	    int save_measure1=0;
	    int save_measure2=0;
	    int first_measure=0; //gives the first measurment of distance
	    int second_measure=0;// gives the seconf measurment after rotation
	    int rotationclock=0; //0 is turning clockwise
	    right_motor_set_speed(200);//the robot starts rotating
	    left_motor_set_speed(-200);
	    rotationclock=1;

	 //init of the functions to start the TOF

	chRegSetThreadName("VL53L0x Thd");
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	(void)arg;
	static VL53L0X_Dev_t device;

	device.I2cDevAddr = VL53L0X_ADDR;
	
	status = VL53L0X_init(&device);

	if(status == VL53L0X_ERROR_NONE){
		VL53L0X_configAccuracy(&device, VL53L0X_LONG_RANGE);
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
    	chprintf((BaseSequentialStream *)&SDU1, "testtest=%dus\n", 0);

		chThdSleepMilliseconds(100);

		//aditionnal code to do the calibration routine
//calculation of the new measure using a sort of a filter

		new_distance= dist_mm;
    	/*chprintf((BaseSequentialStream *)&SDU1, "new_dist:=%dus\n", new_distance);
    	chprintf((BaseSequentialStream *)&SDU1, "limit:=%dus\n", limit);
    	chprintf((BaseSequentialStream *)&SDU1, "first_measure:=%dus\n", first_measure);
    	chprintf((BaseSequentialStream *)&SDU1, "first_measure:=%dus\n", second_measure);*/




			if(((last_distance+15)<new_distance) & (limit!=4)& (limit!=8)){
			    		invers_speed(rotationclock);
			    		limit+=1;
			    		//chprintf((BaseSequentialStream *)&SDU1, "invers:=%dus\n", 0);
			    		if (rotationclock==1){
			    			rotationclock=0;
			    		}
			    		else{rotationclock=1;}

				if(limit==3){
					save_measure1=dist_mm;
				}
				if(limit==4){
					first_measure=(save_measure1+dist_mm)/2,
					//chprintf((BaseSequentialStream *)&SDU1, "YES:=%dus\n", 0);
				}
				if(limit==7){
					save_measure2=dist_mm;
				}
				if(limit==8){
				second_measure=(save_measure2+dist_mm)/2,
				//chprintf((BaseSequentialStream *)&SDU1, "YES2:=%dus\n", 0);
				}


			}
			if((limit==4) & (part<25)){
				part+=1;
		    	//chprintf((BaseSequentialStream *)&SDU1, "part:=%dus\n", part);

			}
			if(part>=25){
				part=0;
				limit=5;
			}
			if(limit==8){
			part=0;
		    right_motor_set_speed(0);//the robot starts rotating
		    left_motor_set_speed(0);
			}
		if (last_distance>new_distance){
			last_distance=new_distance;
		}
		else{
			if((last_distance+15)<new_distance){
				last_distance=new_distance;
			}
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

void invers_speed(int sens){
	if(sens==0){
		right_motor_set_speed(200);
		left_motor_set_speed(-200);
	}
	else{
		right_motor_set_speed(-200);
		left_motor_set_speed(200);
	}
}
