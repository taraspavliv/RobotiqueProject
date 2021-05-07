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

#define PI 3.14159265

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

bool is_number(char chara){
    if((int)chara >= 48 && (int)chara <= 57){
        return true;
    }else{
        return false;
    }
}

void ReceiveCommand(BaseSequentialStream* in){
	volatile char incoming_message[16];

	if(chSequentialStreamRead((BaseSequentialStream*) in,(uint8_t *) &incoming_message, 16) == 0){
		return;
	}
	incoming_message[15] = '\0';
	float angle=0;
	uint16_t distance=0;

    for(int i=0;incoming_message[i] != '\0';++i){
		if(incoming_message[i]=='a' || incoming_message[i]=='d'){
			if(incoming_message[i+1]==':'){
			uint16_t k = 0;
				for(int j=0;j<3;++j){
					if(is_number(incoming_message[i+2+j])){
						k+=(int)incoming_message[i+2+j]-(int)'0';
					}else{
						break;
					}
					k *= 10;
				}
				k /= 10;
				if(incoming_message[i]=='a'){
					angle = k;
				}else if(incoming_message[i]=='d'){
					distance = k;
				}
			}
		}
    }

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        /*switch(state){
        	case 0:{
        		if(c1 == 'a')
        			state = 1;
        		else
        			state = 0;
        		break;
        		}
        	case 1:{
        		if(c1 == ':')
        			state = 2;
        		else if(c1 == 'd')
        			state = 1;
        		else
        			state = 0;
        		break;
        	}
        }

	uint16_t angle = 0;
	c1 = chSequentialStreamGet(in);
	if(is_number(c1)){
		angle+=(uint8_t)c1-48;
		c1 = chSequentialStreamGet(in);
		if(is_number(c1)){
			angle*=10;
			angle+=(uint8_t)c1-48;
			if(is_number(c1)){
				angle*=10;
				angle+=(uint8_t)c1-48;
			}
		}
	}

	c1 = chSequentialStreamGet(in);
	c1 = chSequentialStreamGet(in);
	c1 = chSequentialStreamGet(in);

	uint8_t distance = 0;
	c1 = chSequentialStreamGet(in);
	if(is_number(c1)){
		distance+=(uint8_t)c1-48;
		c1 = chSequentialStreamGet(in);
		if(is_number(c1)){
			distance*=10;
			distance+=(uint8_t)c1-48;
			if(is_number(c1)){
				distance*=10;
				distance+=(uint8_t)c1-48;
			}
		}
	}*/
    // Converting to radian
    angle = (angle * PI) / 180;
	//left_motor_set_speed(8*distance*(cos(angle)+sin(angle)));
	//right_motor_set_speed(8*distance*(cos(angle)-sin(angle)));
	//chprintf((BaseSequentialStream *)&SDU1, "L:%d R:%d \r\n", (int)(8*distance*(cos(angle)+sin(angle))), (int)(8*distance*(cos(angle)-sin(angle))));
	//chprintf((BaseSequentialStream *)&SDU1, "A:%f D:%d \r\n", angle, distance);
    if((int)(8*distance*(cos(angle)+sin(angle))) > 0 && (int)(8*distance*(cos(angle)-sin(angle))) > 0){
    	set_led(0,2);
    }else if((int)(8*distance*(cos(angle)+sin(angle))) < 0 && (int)(8*distance*(cos(angle)-sin(angle))) > 0){
    	set_led(1,2);
    }else if((int)(8*distance*(cos(angle)+sin(angle))) < 0 && (int)(8*distance*(cos(angle)-sin(angle))) < 0){
    	set_led(2,2);
    }else if((int)(8*distance*(cos(angle)+sin(angle))) > 0 && (int)(8*distance*(cos(angle)-sin(angle))) < 0){
    	set_led(3,2);
    }

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

	set_rgb_led(1,0,200,200);
    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(100);
        ReceiveCommand((BaseSequentialStream *) &SD3);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
