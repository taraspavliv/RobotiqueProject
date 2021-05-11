#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h> //no RGB leds included here
#include <math.h>
#include "motors_controller.h"

#include "process_image.h"

#define LINES_TO_ANALYSE 3
#define MINIMUM_BALL_SIZE 15
#define RGB_nb 3

#define BALL_EDGE_THRS 3 //threshold of the edge (derivative)

#define BALL_RADIUS_SEEN 18.735 //radius of the ball at the level of the camera, unit: mm TODO: check
#define CAMERA_ANGLE_OF_VIEW 0.7854 //unit: rad

#define LP_POS_K 0.2 //low pass filter for position, an approximation of the moving average of "1/LP_POS_K" positions
#define LP_MOV_K 0.2 //low pass filter for movement, an approximation of the moving average of "1/LP_MOV_K" movements

static int16_t ball_position_filtered[2] = {0, 0}; //(x,y) unit: mm
static int16_t ball_speed_vector_filtered[2] = {0, 0}; //(x,y) unit: mm/s
static enum BallVisibility ball_visibility = NONE;

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
		//TODO: chSleep?
    }
}


static THD_WORKING_AREA(waProcessImage, 8192);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t main_lines_averaged[PO8030_MAX_WIDTH] = {0}; //TODO: static?

	uint16_t left_ball_side = 0;
	uint16_t right_ball_side = 0;

	bool ball_seen = false;

    while(true){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		// estimates the ball position by checking for black pixels
		ball_seen = first_ball_estimate(img_buff_ptr, main_lines_averaged, &left_ball_side, &right_ball_side);

		if(ball_seen){
			//better estimate by doing edge detection
			final_ball_estimate(main_lines_averaged, &left_ball_side, &right_ball_side);
			//estimates ball position based on self position and angle + ball apperance on the camera
			update_ball_pos_mov(left_ball_side, right_ball_side);
		}else{
			ball_visibility = NONE;
		}

    }
}

bool first_ball_estimate(uint8_t *img_buff_ptr, uint8_t* main_lines_averaged, uint16_t* left_ball_side_ref, uint16_t* right_ball_side_ref){
	bool ball_seen = false;
	static uint16_t main_lines[PO8030_MAX_WIDTH][RGB_nb] = {0}; //to store the RGB values of the camera

	for(uint16_t i=0; i < PO8030_MAX_WIDTH; ++i){
		main_lines[i][0] = 0;
		main_lines[i][1] = 0;
		main_lines[i][2] = 0;
		for(uint8_t j=0; j< LINES_TO_ANALYSE;++j){ //splits values from RGB565 to channels R,G,B
			main_lines[i][0] += (img_buff_ptr[2*i + 2*j*PO8030_MAX_WIDTH]>>3)&0x1F;
			main_lines[i][1] += ((img_buff_ptr[2*i + 2*j*PO8030_MAX_WIDTH]<<3) + (img_buff_ptr[2*i + 1 + 2*j*PO8030_MAX_WIDTH]>>5))&0x3F;
			main_lines[i][2] += (img_buff_ptr[2*i + 1 + 2*j*PO8030_MAX_WIDTH])&0x1F;
		}
		main_lines[i][0] /= LINES_TO_ANALYSE;
		main_lines[i][1] /= LINES_TO_ANALYSE;
		main_lines[i][2] /= LINES_TO_ANALYSE;

		main_lines_averaged[i] = (main_lines[i][0]*main_lines[i][0] + main_lines[i][1]*main_lines[i][1]	+ main_lines[i][2]*main_lines[i][2])/24;
		//TODO: just one channel?

		if(main_lines_averaged[i] == 0){
			if(ball_seen == false){
				ball_seen = true;
				*left_ball_side_ref = i;
			}else{
				*right_ball_side_ref = i;
			}
		}
	}

	return ball_seen;
}

void final_ball_estimate(uint8_t* main_lines_averaged, uint16_t* left_ball_side_ref, uint16_t* right_ball_side_ref){
	static int8_t main_lines_deriv[PO8030_MAX_WIDTH] = {0};

	//tries to push the left ball side until there is an edge
	if(*left_ball_side_ref >= 1){
		for(uint16_t i = *left_ball_side_ref; i > 1; --i){
			main_lines_deriv[i] = (main_lines_averaged[i+1] - main_lines_averaged[i-1])/2; //derivative calculation
			*left_ball_side_ref = i;
			if(main_lines_deriv[i] <= -BALL_EDGE_THRS){
				break;
			}
		}
	}//else the left side is too close to the border to calculate an interesting derivative

	//Similar logic for the right side, looking for an edge further
	if(*right_ball_side_ref <= PO8030_MAX_WIDTH - 2){
		for(uint16_t i = *right_ball_side_ref; i < PO8030_MAX_WIDTH-1; ++i){
			main_lines_deriv[i] = (main_lines_averaged[i+1] - main_lines_averaged[i-1])/2; //derivative calculation
			*right_ball_side_ref = i;
			if(main_lines_deriv[i] >= BALL_EDGE_THRS){
				break;
			}
		}
	}//else too close to right side

}

void update_ball_pos_mov(uint16_t left_ball_side, uint16_t right_ball_side){
	if(left_ball_side <= 1 || right_ball_side >= PO8030_MAX_WIDTH - 2){
		ball_visibility = PARTIAL;
	}else if(right_ball_side - left_ball_side < MINIMUM_BALL_SIZE){
		ball_visibility = NONE;
	}else{
		ball_visibility = FULL;
	}

	float ball_angular_size = 0; //apparent angle of the ball, in rad
	float distance_to_ball = 0; //in mm
	float relative_angle_to_ball_center = 0; //in rad
	int16_t ball_current_estimate_pos[2] = {0, 0}; //in mm
	int16_t ball_current_estimate_mov[2] = {0, 0}; //in mm/s
	static int16_t ball_last_filtered_position[2] = {0, 0};
	static systime_t last_systime = 0; //to calculate speed, static to know the last time this function was called
	if(ball_visibility != NONE){
		//math explained in report TODO:explain in report
		ball_angular_size = (right_ball_side - left_ball_side)*CAMERA_ANGLE_OF_VIEW/PO8030_MAX_WIDTH;
		distance_to_ball = BALL_RADIUS_SEEN/sinf(ball_angular_size/2);
		relative_angle_to_ball_center = CAMERA_ANGLE_OF_VIEW*(0.5 - (right_ball_side + left_ball_side)/(2*PO8030_MAX_WIDTH));

		//estimates the ball position in absolute coordinates
		ball_current_estimate_pos[0] = (int16_t)(get_self_position()[0] + distance_to_ball*cosf(relative_angle_to_ball_center + DEG_TO_RAD(get_self_angle())));
		ball_current_estimate_pos[1] = (int16_t)(get_self_position()[1] + distance_to_ball*sinf(relative_angle_to_ball_center + DEG_TO_RAD(get_self_angle())));

		//filter position
		ball_position_filtered[0] = LP_POS_K * ball_current_estimate_pos[0] + (1 - LP_POS_K)*ball_position_filtered[0];
		ball_position_filtered[1] = LP_POS_K * ball_current_estimate_pos[1] + (1 - LP_POS_K)*ball_position_filtered[1];

		//and the same for the movement vector
		ball_current_estimate_mov[0] = (ball_position_filtered[0] - ball_last_filtered_position[0])*1000/(chVTGetSystemTime() - last_systime);
		ball_current_estimate_mov[1] = (ball_position_filtered[1] - ball_last_filtered_position[1])*1000/(chVTGetSystemTime() - last_systime);
		//filter movement vector
		ball_speed_vector_filtered[0] = LP_MOV_K * ball_current_estimate_mov[0] + (1 - LP_MOV_K)*ball_speed_vector_filtered[0];
		ball_speed_vector_filtered[1] = LP_MOV_K * ball_current_estimate_mov[1] + (1 - LP_MOV_K)*ball_speed_vector_filtered[1];

		//save for the next call of the function
		ball_last_filtered_position[0] = ball_position_filtered[0];
		ball_last_filtered_position[1] = ball_position_filtered[1];
		last_systime = chVTGetSystemTime();
	}

}

int16_t* get_ball_position(void){
	return ball_position_filtered;
}

int16_t* get_ball_movement(void){
	return ball_speed_vector_filtered;
}

enum BallVisibility get_ball_visibility(void){
	return ball_visibility;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
