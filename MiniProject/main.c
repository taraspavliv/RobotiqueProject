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

#include <role_selector.h>
#include <process_image.h>
#include <bt_communication.h>
#include <position_motion_controller.h>
#include <position_calibrator.h>

#define PI 3.14159265
#define RAYON 53 //en mm a verifier
#define CONVERTER PI*41/1000  //41mm comme diamètre de la roue


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}



static float position_x =0;
static float position_y=0;
static float my_angle=0;
static float stepr_1=0;
static float stepl_1=0;



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

	//set_rgb_led(1,200,200,0);
    /* Infinite loop. */
    while (true) {
    	float angle_rad = get_BT_controller_joystick_polar()[1];
    	angle_rad = angle_rad * 3.14159265 / 180;
        set_led(0, cos(angle_rad)>0);
        set_led(2, cos(angle_rad)<0);

        set_led(1, sin(angle_rad)>0);
        set_led(3, sin(angle_rad)<0);
    	/* controlled_command
    	left_motor_set_speed(8*distance*(cos(angle)+sin(angle)));
    	right_motor_set_speed(8*distance*(cos(angle)-sin(angle)));
    	*/
        chThdSleepMilliseconds(10);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
