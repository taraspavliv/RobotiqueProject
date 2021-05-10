#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <leds.h>
#include <spi_comm.h> //for RGB leds
#include <selector.h>

#include "role_selector.h"
#include "process_image.h"
#include "bt_communication.h"
#include "position_calibrator.h"
#include "motors_controller.h"

static void serial_start(void) //TODO:move to bt_communication?
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    //messagebus_init();

	//clear_leds();
	//set_body_led(0);
	//set_front_led(0);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//for rgb leds
	//spi_comm_start();

	//stars the threads for the processing of the image
	process_image_start();
	role_selector_start();
	bt_communication_start();
	motors_controller_start();

	//set_rgb_led(1,200,200,0);
    /* Infinite loop. */
	bool test_ordered = false;
	bool test_ordered_2 = false;
    while (true) {
    	uint8_t joystick_distance = get_BT_controller_joystick_polar()[0];
    	float angle_rad = get_BT_controller_joystick_polar()[1];
    	angle_rad = angle_rad * 3.14159265 / 180;
        //set_led(0, cos(angle_rad)>0);
        //set_led(2, cos(angle_rad)<0);

        //set_led(1, sin(angle_rad)>0);
        //set_led(3, sin(angle_rad)<0);
    	//controlled_command
        /*if(get_BT_controller_shoot() == false){
        	left_motor_set_speed(8*joystick_distance*(cos(angle_rad)+sin(angle_rad)));
        	right_motor_set_speed(8*joystick_distance*(cos(angle_rad)-sin(angle_rad)));
        }else{
        	reset_shoot();
        	left_motor_set_speed(MOTOR_SPEED_LIMIT);
        	right_motor_set_speed(MOTOR_SPEED_LIMIT);
        	chThdSleepMilliseconds(700);
        }*/
        if(test_ordered == false){
        	float target_position[2] = {400,600};
        	//set_angle(90);
        	set_position(target_position);
        	test_ordered = true;
        }else if(get_position_achieved() && test_ordered_2 == false){
        	float target_position[2] = {500,500};
        	set_position(target_position);
        	test_ordered_2 = true;
        }


        chThdSleepMilliseconds(30);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
