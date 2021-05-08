#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h> //no RGB leds included here
#include <math.h>
#include <motors.h>

#include <process_image.h>

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage,256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    po8030_advanced_config(FORMAT_RGB565, 0, PO8030_MAX_HEIGHT/2 - LINES_TO_ANALYSE/2, PO8030_MAX_WIDTH, LINES_TO_ANALYSE<2?2:LINES_TO_ANALYSE, SUBSAMPLING_X1, SUBSAMPLING_X1);
    po8030_set_contrast(100);
    po8030_set_ae(0);
    po8030_set_awb(0);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(true){
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
	static uint8_t main_lines_averaged[PO8030_MAX_WIDTH] = {0};

	bool ball_seen = false;
	uint16_t right_ball_side=0;
	uint16_t left_ball_side=0;
	uint16_t ball_center=0;
	float distance_to_ball = 0;



	//on met le compteur de la position à 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);

    while(true){
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
			for(uint8_t j=0; j< LINES_TO_ANALYSE;++j){
				main_lines[i][0] += (img_buff_ptr[2*i + 2*j*PO8030_MAX_WIDTH]>>3)&0x1F;
				main_lines[i][1] += ((img_buff_ptr[2*i + 2*j*PO8030_MAX_WIDTH]<<3) + (img_buff_ptr[2*i + 1 + 2*j*PO8030_MAX_WIDTH]>>5))&0x3F;
				main_lines[i][2] += (img_buff_ptr[2*i + 1 + 2*j*PO8030_MAX_WIDTH])&0x1F;
			}
			main_lines[i][0] /= LINES_TO_ANALYSE;
			main_lines[i][1] /= LINES_TO_ANALYSE;
			main_lines[i][2] /= LINES_TO_ANALYSE;
		}

		for(uint16_t i=0; i < PO8030_MAX_WIDTH; ++i){
			main_lines_averaged[i] = (main_lines[i][0]*main_lines[i][0] + main_lines[i][1]*main_lines[i][1] + main_lines[i][2]*main_lines[i][2])/24;
		}

		for(uint16_t i=1; i < PO8030_MAX_WIDTH-1; ++i){
			main_lines_averaged[i] = (main_lines_averaged[i-1] + main_lines_averaged[i] + main_lines_averaged[i+1])/3;
		}

		for(uint16_t i=1; i < PO8030_MAX_WIDTH-1; ++i){
			main_lines_averaged[i] = (main_lines_averaged[i-1] + main_lines_averaged[i] + main_lines_averaged[i+1])/3;
			if(main_lines_averaged[i+1] - main_lines_averaged[i-1] < 5 && ball_seen == false){
				ball_seen = true;
				left_ball_side = i;
			}else if(main_lines_averaged[i+1] - main_lines_averaged[i-1] > 5 && ball_seen == true){
				right_ball_side = i;
			}
		}

		if(right_ball_side - left_ball_side < MINIMUM_BALL_SIZE){
			ball_seen = false;
		}else{
			ball_center = (right_ball_side + left_ball_side)/2;
		}

		/*if(ball_seen){
			set_led(0,1);
		}else{
			set_led(0,0);
		}*/

    }
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
