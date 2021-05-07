#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include "camera/po8030.h"
#include <leds.h> //no RGB leds included here
#include <math.h>
#include <motors.h>

#include <process_image.h>

#define PI 3.1417
#define RAYON 5.3 //en cm a verifier
#define CONVERTER PI*4.1/1000  //41mm comme diamètre de la roue

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 4096);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    //po8030_set_ae(1);
    //po8030_set_awb(1);
    po8030_advanced_config(FORMAT_RGB565, 0, PO8030_MAX_HEIGHT/2 - LINES_TO_ANALYSE/2, PO8030_MAX_WIDTH, LINES_TO_ANALYSE<2?2:LINES_TO_ANALYSE, SUBSAMPLING_X1, SUBSAMPLING_X1);
    //po8030_config(FORMAT_RGB565, SIZE_QVGA);
    po8030_set_contrast(15);
    po8030_set_ae(0);
    po8030_set_awb(1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	//set_led(3,1);
	//systime_t time = 0;
    while(1){
		//set_led(3,2);
		//chprintf((BaseSequentialStream *)&SD3, "time: %d", chVTGetSystemTime()-time);
		//time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 8192);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	static uint16_t main_lines[PO8030_MAX_WIDTH][RGB_nb] = {0};

	bool ball_seen;
	uint16_t right_ball_side=0;
	uint16_t left_ball_side=0;
	uint16_t ball_center=0;
	float distance_to_ball = 0;


	//initialisation des variables
	float position_x =0;
	float position_y=0;
	float angle=0;
	float stepr_1=0;
	float stepl_1=0;
	//on met le compteur de la position à 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);

    while(1){
    	//waits until an image has been captured
        //chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		ball_seen = false;
		right_ball_side=0;
		left_ball_side=0;

		for(uint16_t i=0; i < PO8030_MAX_WIDTH; ++i){
			main_lines[i][0] = 0;
			main_lines[i][1] = 0;
			main_lines[i][2] = 0;
			//for(uint8_t j=0; j< LINES_TO_ANALYSE;++j){
			int j = 0;
				main_lines[i][0] += (img_buff_ptr[2*i + 2*j*PO8030_MAX_WIDTH]>>3)&0x1F;
				main_lines[i][1] += ((img_buff_ptr[2*i + 2*j*PO8030_MAX_WIDTH]<<3) + (img_buff_ptr[2*i + 1 + 2*j*PO8030_MAX_WIDTH]>>5))&0x3F;
				main_lines[i][2] += (img_buff_ptr[2*i + 1 + 2*j*PO8030_MAX_WIDTH])&0x1F;
			//}
			/*main_lines[i][0] /= LINES_TO_ANALYSE;
			main_lines[i][1] /= LINES_TO_ANALYSE;
			main_lines[i][2] /= LINES_TO_ANALYSE;*/
			if(i >= 2 && is_orange(main_lines[i]) && is_orange(main_lines[i-1]) && is_orange(main_lines[i-2])){
				if(ball_seen == false){
					ball_seen = true;
					left_ball_side = i;
				}else{
					right_ball_side = i;
				}
			}
		}

		if(right_ball_side - left_ball_side < MINIMUM_BALL_SIZE){
			ball_seen = false;
		}else{
			ball_center = (right_ball_side + left_ball_side)/2;
		}

		/*chprintf((BaseSequentialStream *)&SD3, "R: %d ", main_lines[320][0]);
		chprintf((BaseSequentialStream *)&SD3, "G: %d ", main_lines[320][1]);
		chprintf((BaseSequentialStream *)&SD3, "B: %d \n\r", main_lines[320][2]);*/

		//chprintf((BaseSequentialStream *)&SD3, "D: %c", 'c');

		/*if(ball_seen){
			set_led(2,1);
			//chprintf((BaseSequentialStream *)&SD3, "C: %d ", ball_center);
			//chprintf((BaseSequentialStream *)&SD3, "W: %d \n\r", right_ball_side - left_ball_side);
			distance_to_ball = 37.47/(2*tan(3.141592*(right_ball_side - left_ball_side)/5120));
			//chprintf((BaseSequentialStream *)&SD3, "D: %f", distance_to_ball);
			if(ball_center>340){
				left_motor_set_speed(100);
				right_motor_set_speed(-100);
			}else if(ball_center<300){
				left_motor_set_speed(-100);
				right_motor_set_speed(100);
			}else{
				left_motor_set_speed(50);
				right_motor_set_speed(50);
			}
		}else{
			set_led(2,0);
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}*/
        //ReceiveCommand((BaseSequentialStream *) &SD3);
		//set_led(1, 2);


		float l1=0; //increment de la roue right
		float l2=0; //increment de la roue left
		l1= (right_motor_get_pos()-stepr_1)*CONVERTER; //de step a cm
		l2= (left_motor_get_pos()-stepl_1)*CONVERTER;

		position_x= position_x+(l1+l2)*cosf(angle)/2;
		position_y= position_y+(l1+l2)*sinf(angle)/2;
		angle=angle+tanf((l1-l2)/RAYON);

		stepr_1=right_motor_get_pos();
		stepl_1=left_motor_get_pos();
		//chprintf((BaseSequentialStream *)&SD3, "x: %f y: %f theta: %f \n\r", position_x, position_y, angle);

    }
}

void position_update(void){


	//boucle qui actualise les valeurs et de la position
	while(1){

	}

}

bool is_green(uint8_t* pixel){
	if(pixel[1] > pixel[0]*2 + 2 && pixel[1] > pixel[2]*2 - 1 && pixel[1] > 10){
		return true;
	}else{
		return false;
	}
}

bool is_orange(uint16_t* pixel){
	//TODO:check if pixel[0] can be set to 4-5
	if(pixel[0] >= 4 && pixel[1] + pixel[2] <= 1 + pixel[0]/6){
		return true;
	}else{
		return false;
	}
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
