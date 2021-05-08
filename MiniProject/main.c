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
#define RAYON 53 //en cm a verifier
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

bool is_number(char chara){
    if((int)chara >= 48 && (int)chara <= 57){
        return true;
    }else{
        return false;
    }
}

static float position_x =0;
static float position_y=0;
static float my_angle=0;
static float stepr_1=0;
static float stepl_1=0;

void ReceiveCommand(BaseSequentialStream* in){
	volatile char incoming_message[16];

	if(chSequentialStreamRead((BaseSequentialStream*) in,(uint8_t *) &incoming_message, 16) == 0){
		return;
	}
	incoming_message[15] = '\0';
	float angle=0;
	uint16_t distance=0;
	bool discard = false;

    for(int i=0;incoming_message[i] != '\0';++i){
		if(incoming_message[i]=='a' || incoming_message[i]=='d'){
			if(incoming_message[i+1]==':'){
			uint16_t k = 0;
				for(int j=0;j<3;++j){
					if(is_number(incoming_message[i+2+j])){
						k+=(int)incoming_message[i+2+j]-(int)'0';
					}else if(incoming_message[i+2+j] == '\0'){
						discard = true;
						break;
					}else{
						break;
					}
					k *= 10;
				}
				k /= 10;
				if(discard == false){
					if(incoming_message[i]=='a'){
						angle = k;
					}else if(incoming_message[i]=='d'){
						distance = k;
					}
				}
			}
		}
    }

    angle = (angle * PI) / 180; // Converting to radian
	left_motor_set_speed(8*distance*(cos(angle)+sin(angle)));
	right_motor_set_speed(8*distance*(cos(angle)-sin(angle)));

	//initialisation des variables


	float l1=0; //increment de la roue right
	float l2=0; //increment de la roue left
	l1= (right_motor_get_pos()-stepr_1)*CONVERTER; //de step a mm
	l2= (left_motor_get_pos()-stepl_1)*CONVERTER;

	position_x= position_x+(l1+l2)*cosf(my_angle)/2;
	position_y= position_y+(l1+l2)*sinf(my_angle)/2;
	my_angle=my_angle+tanf((l1-l2)/RAYON);
	my_angle = my_angle*180/PI;

	stepr_1=right_motor_get_pos();
	stepl_1=left_motor_get_pos();
	chprintf((BaseSequentialStream *)&SD3, "x:%f y:%f theta:%f \n\r", position_x, position_y, my_angle);
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

	set_rgb_led(1,200,200,0);
    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(50);
        ReceiveCommand((BaseSequentialStream *) &SD3);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
