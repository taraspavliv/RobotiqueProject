#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdio.h>
#include <math.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h> // add the absolute position of the ball
// add #include <process_position.h> witch gives the absolute position of the e-puck and of the goal

#define SEPARATION 10;
#define MAX 1000

static    uint16_t timer_orientation=0;
static 	uint16_t position_counter=0;

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //declaration of variables used in the thread, some will be replaced by functions

    int16_t final_angle=0;
    int16_t final_position_x=0;
    int16_t final_position_y=0;
    int16_t distance_x_steps=0;
    int16_t distance_y_steps=0;
    int16_t absolute_distance_steps=0; //sqrt(x*x+y*y),
    uint16_t steps=0;
    uint16_t nb_steps=0;
    uint16_t nb_steps2=0;


    //the initial position of the counters of the motors is set to 0
    right_motor_set_speed(0);
    left_motor_set_speed(0);

    final_angle= (get_y_goal()-get_y_ball())/(get_y_goal()-get_x_ball());

    final_position_x=(ball_x_position()-SEPARATION*cos(final_angle));
    final_position_y=(ball_y_position()-SEPARATION*sin(final_angle));

    distance_x_steps= (final_position_x-robot_position_x)*CONVERTER;//converter to steps
    distance_y_steps= (final_position_y-robot_position_y)*CONVERTER;

    absolute_distance_steps=sqrt(distance_x_steps*distance_x_steps+distance_y_steps*distance_y_steps;

    //calculate the angle that the epucks needs to do during the first rotation

    angle_init=acrt(distance_y_steps/distance_x_steps);
    		if (distance_y_steps>0){
    			angle_init= angle_init*180/pi;
    		}
    		else{
    			angle_init=(angle_init+ pi/2)*180/pi;
    		}
  nb_steps=angle_init*1300/360;

  //calculate the angle that the epucks needs to do during the second rotation rotation
 angle_init2= angle_init-final_angle;
 nb_steps2= angle_init2*1300/360;

    while(1){
        time = chVTGetSystemTime();
        chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", right_motor_get_pos());

        if (robot_centered()){ //hits the ball
        	right_motor_set_speed(MAX);
        	left_motor_set_speed(MAX);
        }
        else{
        	switch (position_correction){

        	case 0:
        		steps=left_motor_get_pos();
        		++position_correction;
        		right_motor_set_speed(200);
        		left_motor_set_speed(-200);
        		break;
        	case 1:
        		while ((steps+nb_steps)<left_motor_get_pos()){
        			break;
        		}
        		++position_correction;
        		break;
        	case 2:
        		steps=left_motor_get_pos();
        		right_motor_set_speed(400);
        		left_motor_set_speed(400);
        		++position_correction;
        		break;
        	case 3:
        		while ((steps+absolute_distance_steps)<left_motor_get_pos()){
        			break;
        		}
        		++position_correction;
        		break;
        	case 4:
        		steps=left_motor_get_pos();
        		++position_correction;
        		right_motor_set_speed(200);
        		left_motor_set_speed(-200);
        		break;
        	case 5:
        		while ((steps+nb_steps2)<left_motor_get_pos()){ //trouverle nb_steps2 pour s'orienter vers la balle cette fois.
        			break;
        		}
        		++position_correction;
        		break;

        	}

        }
   }

		        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}


//verifies if the epuck is alligned behind the ball
int robot_centered(void){

uint16_t optimal_angle=0;
optimal_angle=(get_y_goal()-get_y_robot())/(get_x_goal()-get_x_robot());

	 if (abs(final_angle-optimal_angle)<5) {
	        	return 1;
	 }
	 else {
		 return 0;
	 }
}

