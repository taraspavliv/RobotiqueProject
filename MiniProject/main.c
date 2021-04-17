#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"

#include <main.h>

//#include "sensors/ground.h" //TODO:why?
//#include <camera/camera.h>
#include <usbcfg.h>
#include "camera/po8030.h"
#include "motors.h"
#include "leds.h"
#include "spi_comm.h" //for RGB leds
#include "selector.h"

#include <process_image.h>

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

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    //messagebus_init();

	clear_leds();
	set_body_led(0);
	set_front_led(0);

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
	spi_comm_start();

	//stars the threads for the processing of the image
	process_image_start();

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
