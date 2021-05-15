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

#define BALL_EDGE_THRS 2 //threshold of the edge (derivative)
#define BALL_AOV_MARGIN 10 //in pixels, ball should be BALL_AOV_MARGIN inside the angle of view to be considered fully seen

#define CAMERA_ANGLE_OF_VIEW 0.7854 //unit: rad
#define BALL_RADIUS_SEEN 18.735 //radius of the ball at the level of the camera, unit: mm TODO: check
#define RADIUS_TO_CAMERA 35 //unit : mm (distance from center of epuck to the camera)

#define LP_POS_K 0.2 //low pass filter for position, an approximation of the moving average of "1/LP_POS_K" positions
#define LP_MOV_K 0.2 //low pass filter for movement, an approximation of the moving average of "1/LP_MOV_K" movements

#define SPEED_CONVERTER 100 //converts the speed from mm/ms to mm/s

static int16_t ball_position_filtered[2] = {0, 0}; //(x,y) unit: mm
static float ball_speed_vector_filtered[2] = {0, 0}; //(dx/dt, dy/dt) unit: doesn't really matter, as we want to know the direction not the speed
static enum BallVisibility ball_visibility = NONE;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage,256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //set grayscale image
    po8030_advanced_config(FORMAT_YYYY, 0, PO8030_MAX_HEIGHT/2 - LINES_TO_ANALYSE/2, PO8030_MAX_WIDTH, LINES_TO_ANALYSE<2?2:LINES_TO_ANALYSE, SUBSAMPLING_X1, SUBSAMPLING_X1);
    po8030_set_contrast(45);
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
			//chprintf((BaseSequentialStream *) &SD3, "l:%d r:%d ", left_ball_side, right_ball_side);
			//better estimate by doing edge detection
			final_ball_estimate(main_lines_averaged, &left_ball_side, &right_ball_side);
			//chprintf((BaseSequentialStream *) &SD3, "le:%d re:%d ", left_ball_side, right_ball_side);
			//estimates ball position based on self position and angle + ball apperance on the camera
			update_ball_pos_mov(left_ball_side, right_ball_side);
		}else{
			ball_visibility = NONE;
		}

    }
}

bool first_ball_estimate(uint8_t *img_buff_ptr, uint8_t* main_lines_averaged, uint16_t* left_ball_side_ref, uint16_t* right_ball_side_ref){
	bool ball_seen = false;

	//for spatial averaging we set the edge cases
	main_lines_averaged[0] = img_buff_ptr[0];
	main_lines_averaged[1] = img_buff_ptr[1];
	main_lines_averaged[PO8030_MAX_WIDTH - 1] = img_buff_ptr[PO8030_MAX_WIDTH-1];
	main_lines_averaged[PO8030_MAX_WIDTH - 2] = img_buff_ptr[PO8030_MAX_WIDTH-2];

	for(uint16_t i=2; i < PO8030_MAX_WIDTH-2; ++i){
		main_lines_averaged[i] = 0;
		for(uint8_t j=0; j< LINES_TO_ANALYSE;++j){
			main_lines_averaged[i] += (img_buff_ptr[i - 2 + j*PO8030_MAX_WIDTH] +
									   img_buff_ptr[i - 1 + j*PO8030_MAX_WIDTH] +
									   img_buff_ptr[i   +   j*PO8030_MAX_WIDTH] +
									   img_buff_ptr[i + 1 + j*PO8030_MAX_WIDTH] +
									   img_buff_ptr[i + 2 + j*PO8030_MAX_WIDTH])/(5*LINES_TO_ANALYSE);
		}

		if(main_lines_averaged[i] <= 10){
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
	if(*left_ball_side_ref >= BALL_AOV_MARGIN){
		for(uint16_t i = *left_ball_side_ref; i > 1; --i){
			main_lines_deriv[i] = (main_lines_averaged[i+1] - main_lines_averaged[i-1])/2; //derivative calculation
			*left_ball_side_ref = i;
			if(main_lines_deriv[i] <= -BALL_EDGE_THRS){
				break;
			}
		}
	}//else the left side is too close to the border to calculate an interesting derivative

	//Similar logic for the right side, looking for an edge further
	if(*right_ball_side_ref <= PO8030_MAX_WIDTH - 1 - BALL_AOV_MARGIN){
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
	//TODO
	chprintf((BaseSequentialStream *) &SD3, "0:%d 1:%d\r\n", left_ball_side, right_ball_side);
	if(left_ball_side <= BALL_AOV_MARGIN || right_ball_side >= PO8030_MAX_WIDTH - 1 - BALL_AOV_MARGIN){
		ball_visibility = PARTIAL;
	}else if(left_ball_side <= BALL_AOV_MARGIN && right_ball_side >= PO8030_MAX_WIDTH - 1 - BALL_AOV_MARGIN){
		ball_visibility = CAM_OVERFLOW;
	}else if(right_ball_side - left_ball_side < MINIMUM_BALL_SIZE){
		ball_visibility = NONE;
	}else{
		ball_visibility = FULL;
	}

	float ball_angular_size = 0; //apparent angle of the ball, in rad
	float distance_to_ball = 0; //in mm
	float relative_angle_to_ball_center = 0; //in rad
	int16_t ball_current_estimate_pos[2] = {0, 0}; //in mm
	float ball_current_estimate_mov[2] = {0, 0}; //in mm/s
	static int16_t ball_last_filtered_position[2] = {0, 0};
	static systime_t last_systick = 0; //to calculate speed, static to know the last time this function was called
	if(ball_visibility != NONE){
		//math explained in report TODO:explain in report
		ball_angular_size = (right_ball_side - left_ball_side)*CAMERA_ANGLE_OF_VIEW/PO8030_MAX_WIDTH;
		distance_to_ball = BALL_RADIUS_SEEN/sinf(ball_angular_size/2);
		relative_angle_to_ball_center = CAMERA_ANGLE_OF_VIEW*(0.5 - (float)(right_ball_side + left_ball_side)/(2*PO8030_MAX_WIDTH));

		//estimates the ball position in absolute coordinates
		ball_current_estimate_pos[0] = (int16_t)(get_self_position()[0] + RADIUS_TO_CAMERA*cosf(DEG_TO_RAD(get_self_angle())) + distance_to_ball*cosf(relative_angle_to_ball_center + DEG_TO_RAD(get_self_angle())));
		ball_current_estimate_pos[1] = (int16_t)(get_self_position()[1] + RADIUS_TO_CAMERA*sinf(DEG_TO_RAD(get_self_angle())) + distance_to_ball*sinf(relative_angle_to_ball_center + DEG_TO_RAD(get_self_angle())));

		//filter position
		ball_position_filtered[0] = LP_POS_K * ball_current_estimate_pos[0] + (1 - LP_POS_K)*ball_position_filtered[0];
		ball_position_filtered[1] = LP_POS_K * ball_current_estimate_pos[1] + (1 - LP_POS_K)*ball_position_filtered[1];

		//and the same for the movement vector
		//chprintf((BaseSequentialStream *) &SD3, "dt:%d ", chVTGetSystemTime() - last_systick);
		ball_current_estimate_mov[0] = ((ball_position_filtered[0] - ball_last_filtered_position[0])*1.0/ST2MS(chVTGetSystemTime() - last_systick))*SPEED_CONVERTER;
		ball_current_estimate_mov[1] = ((ball_position_filtered[1] - ball_last_filtered_position[1])*1.0/ST2MS(chVTGetSystemTime() - last_systick))*SPEED_CONVERTER;

		//filter movement vector
		ball_speed_vector_filtered[0] = LP_MOV_K * ball_current_estimate_mov[0] + (1 - LP_MOV_K)*ball_speed_vector_filtered[0];
		ball_speed_vector_filtered[1] = LP_MOV_K * ball_current_estimate_mov[1] + (1 - LP_MOV_K)*ball_speed_vector_filtered[1];
		//chprintf((BaseSequentialStream *) &SD3, "dx:%f dy:%f ", ball_speed_vector_filtered[0], ball_speed_vector_filtered[1]);

		//save for the next call of the function
		ball_last_filtered_position[0] = ball_position_filtered[0];
		ball_last_filtered_position[1] = ball_position_filtered[1];
		last_systick = chVTGetSystemTime();
	}

}

int16_t* get_ball_position(void){
	return ball_position_filtered;
}

int16_t* get_ball_movement(void){
    static int16_t ball_movement_vector_int[2] = {0};
    ball_movement_vector_int[0] = (int16_t)ball_speed_vector_filtered[0];
    ball_movement_vector_int[1] = (int16_t)ball_speed_vector_filtered[1];
    return ball_movement_vector_int;
}

enum BallVisibility get_ball_visibility(void){
	return ball_visibility;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
