/**
 * @file    VL53L0X.c
 * @brief   High level functions to use the VL53L0X TOF sensor.
 *
 * @author  Eliot Ferragni
 */

#include "ch.h"
#include "hal.h"
#include "VL53L0Xa.h"
#include "Api/core/inc/vl53l0x_api.h"
#include "shell.h"
#include "chprintf.h"
#include "i2c_bus.h"
#include "usbcfg.h"
#include "motors.h"

static uint16_t dist_mm = 0;
static thread_t *distThd;
static bool VL53L0X_configured = false;

//////////////////// PUBLIC FUNCTIONS /////////////////////////
static THD_WORKING_AREA(waVL53L0XThd, 512);
static THD_FUNCTION(VL53L0XThd, arg) {

	 // start of the calibration function

	 //declaration of variables
	    int last_distance=0;
	    int new_distance=0;
	    int limit=0;
	    int bools=0;
	    int part=0;
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


		//aditionnal code to do the calibration routine
//calculation of the new measure using a sort of a filter
		new_distance= dist_mm*0.1+0.9*last_distance;

		if ((part==0) || (part==2)){
			if (last_distance>=new_distance){
				//do nothing
			}
			else{
				invers_speed(rotationclock);
				if(limit==1){
					save_measure1=dist_mm;
					limit=2;
				}
				else{
					if(limit==2){
						if(part==2){
							second_measure=(save_measure1+dist_mm)/2;
		//add something to show that the calibration is endeed
						}
						first_measure=(save_measure1+dist_mm)/2,
						part=1;
					}
				}
				if(limit==0){
					limit=1;
				}

			}
		}
		else{
			if(bools==0){
				bools=1;
				left_motor_set_pos(0);
				right_motor_set_pos(0);
				 right_motor_set_speed(200);//the robot starts rotating
				 left_motor_set_speed(-200);
				 rotationclock=1;

			}
			if((bools=1)&(left_motor_get_pos()<1000)){//find correct value
				//do nothing
			}

			else{
				right_motor_set_speed(0);//the robot stops
				left_motor_set_speed(0);
				part=2;

			}
		}

		chThdSleepMilliseconds(20);
    }




}


VL53L0X_Error VL53L0X_init(VL53L0X_Dev_t* device){

	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

//init
	if(status == VL53L0X_ERROR_NONE)
    {
    	// Structure and device initialisation
        status = VL53L0X_DataInit(device);
    }

    if(status == VL53L0X_ERROR_NONE)
    {
    	// Get device info
        status = VL53L0X_GetDeviceInfo(device, &(device->DeviceInfo));
    }

    if(status == VL53L0X_ERROR_NONE)
    {
    	// Device Initialization
        status = VL53L0X_StaticInit(device);
    }

//calibration
 	if(status == VL53L0X_ERROR_NONE)
    {
    	// SPAD calibration
        status = VL53L0X_PerformRefSpadManagement(device,
        		&refSpadCount, &isApertureSpads);
    }

    if(status == VL53L0X_ERROR_NONE)
    {
    	// Calibration
        status = VL53L0X_PerformRefCalibration(device,
        		&VhvSettings, &PhaseCal);
    }

    return status;
}

VL53L0X_Error VL53L0X_configAccuracy(VL53L0X_Dev_t* device, VL53L0X_AccuracyMode accuracy){

	VL53L0X_Error status = VL53L0X_ERROR_NONE;

//Activation Limits
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(device,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(device,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

//specific accuracy config
//copied from ST example and API Guide
    switch(accuracy){

    	case VL53L0X_DEFAULT_MODE:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckEnable(device,
		        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
		    }

		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
		        		(FixPoint1616_t)(1.5*0.023*65536));
		    }
		    break;

		case VL53L0X_HIGH_ACCURACY:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(FixPoint1616_t)(0.25*65536));
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		(FixPoint1616_t)(18*65536));
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device,
		        		200000);
		    }
			break;

		case VL53L0X_LONG_RANGE:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(FixPoint1616_t)(0.1*65536));
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		(FixPoint1616_t)(60*65536));
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device,
		        		33000);
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetVcselPulsePeriod(device,
				        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetVcselPulsePeriod(device,
				        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
		    }
			break;

		case VL53L0X_HIGH_SPEED:
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		        		(FixPoint1616_t)(0.25*65536));
			}
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetLimitCheckValue(device,
		        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		        		(FixPoint1616_t)(32*65536));
		    }
		    if (status == VL53L0X_ERROR_NONE) {
		        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(device,
		        		30000);
		    }
			break;
		default:
			status = VL53L0X_ERROR_INVALID_PARAMS;
			break;
    }

    return status;
}

VL53L0X_Error VL53L0X_startMeasure(VL53L0X_Dev_t* device, VL53L0X_DeviceModes mode){

	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	status = VL53L0X_SetDeviceMode(device, mode);

	if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_StartMeasurement(device);
    }

    return status;
}

VL53L0X_Error VL53L0X_getLastMeasure(VL53L0X_Dev_t* device){
	return VL53L0X_GetRangingMeasurementData(device,
			&(device->Data.LastRangeMeasure));
}

VL53L0X_Error VL53L0X_stopMeasure(VL53L0X_Dev_t* device){
	return VL53L0X_StopMeasurement(device);
}

void VL53L0Xa_start(void){

	if(VL53L0X_configured) {
		return;
	}

	i2c_start();

	distThd = chThdCreateStatic(waVL53L0XThd,
                     sizeof(waVL53L0XThd),
                     NORMALPRIO + 10,
                     VL53L0XThd,
                     NULL);
}

void VL53L0X_stop(void) {
    chThdTerminate(distThd);
    chThdWait(distThd);
    distThd = NULL;
    VL53L0X_configured = false;
}

uint16_t VL53L0X_get_dist_mm(void) {
	return dist_mm;
}


