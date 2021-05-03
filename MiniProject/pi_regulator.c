#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdio.h>
#include <math.h>

#include <main.h>
#include <motors.h>
#include <Pi_regulator.h>
#include <process_image.h> // add the absolute position of the ball
// add #include <process_position.h> witch gives the absolute position of the e-puck and of the goal

#define SEPARATION 10
#define MAX 1000
#define PI 3.1417

static    uint16_t timer_orientation=0;
static 	uint16_t position_corrector=0;
static  uint16_t test=0;

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //declaration of variables used in the thread, some will be replaced by functions

    int16_t final_angle=0;
    int16_t distance_x_steps=0;
    int16_t distance_y_steps=0;
    int16_t absolute_distance_steps=0; //sqrt(x*x+y*y),
    uint16_t steps=0;
    uint16_t nb_steps=0;
    uint16_t nb_steps2=0;
    uint16_t angle_init=0;
    uint16_t angle_init2=0;
    double final_position_x=0;
    double final_position_y=0;


    //the initial position of the counters of the motors is set to 0
    right_motor_set_speed(0);
    left_motor_set_speed(0);

    final_angle= (get_y_goal()-get_y_ball())/(get_x_goal()-get_x_ball()+5);
    //chprintf((BaseSequentialStream *)&SDU1, "final_angle=%dus\n", final_angle);


    final_position_x=10*cos(get_x_ball()-10*(final_angle)); //remplacer 10*cos par SEPARATION
    final_position_y=10*cos(get_y_ball()-10*(final_angle));

    distance_x_steps= 10*(final_position_x-get_x_robot());//remplacer 10 par CONVERTER, le facteur de conversion de distance en steps
    distance_y_steps= 10*(final_position_y-get_y_robot());

    absolute_distance_steps=sqrt(distance_x_steps*distance_x_steps+distance_y_steps*distance_y_steps); //add sqrt

    //calculate the angle that the epucks needs to do during the first rotation

    angle_init=(distance_y_steps/(distance_x_steps+1)); //add arctan
    		if (distance_y_steps>0){
    			angle_init= angle_init*180/PI;
    		}
    		else{
    			angle_init=(angle_init+ PI/2)*180/PI;
    		}
  nb_steps=angle_init*1300/360;

  //calculate the angle that the epucks needs to do during the second rotation rotation
 angle_init2= angle_init-final_angle;
 nb_steps2= angle_init2*1300/360;

    while(1){
        time = chVTGetSystemTime();
        //chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", right_motor_get_pos());

        if (robot_centered(final_angle)){ //hits the ball
            //chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", 1001);
        	right_motor_set_speed(MAX);
        	left_motor_set_speed(MAX);
        }

        else{
            //chprintf((BaseSequentialStream *)&SDU1, "pos_co=%dus\n", position_corrector);

      switch (position_corrector){
        	case 0:
        		//chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", 2002);
        		//chprintf((BaseSequentialStream *)&SDU1, "position_corrector=%dus\n", position_corrector);

        		steps=left_motor_get_pos();
        		position_corrector=1;
        		right_motor_set_speed(200);
        		left_motor_set_speed(-200);
        		//chprintf((BaseSequentialStream *)&SDU1, "steps+nb_steps=%dus\n", steps+nb_steps);

        		break;
        	case 1:
        		//while ((steps+nb_steps)<left_motor_get_pos()){
        		for(int i=0; i<2000; ++i){ //i<steps+nb_steps
        	        //chprintf((BaseSequentialStream *)&SDU1, "case1=%dus\n", 1);
        	        //chprintf((BaseSequentialStream *)&SDU1, "i=%dus\n", i);
        	        chThdSleepUntilWindowed(time, time + MS2ST(10));



        		}
        		position_corrector=2;
        		//break;
        	case 2:
        		steps=left_motor_get_pos();
        		right_motor_set_speed(400);
        		left_motor_set_speed(400);
    	        //chprintf((BaseSequentialStream *)&SDU1, "case1=%dus\n", 2);

    	        position_corrector=3;
        		//break;
        	case 3:
        		for(int i=0; i<7000; ++i){ //i<steps+absolute_distance_steps
        		/*while ((steps+absolute_distance_steps)<left_motor_get_pos()){*/
        	        //chprintf((BaseSequentialStream *)&SDU1, "case3=%dus\n", 3);
        	        //chprintf((BaseSequentialStream *)&SDU1, "i=%dus\n", i);
        	        chThdSleepUntilWindowed(time, time + MS2ST(10));

        		}
        		 position_corrector=4;
        	case 4:
        		steps=left_motor_get_pos();
        		 position_corrector=5;
        		right_motor_set_speed(200);
        		left_motor_set_speed(-200);
    	        //chprintf((BaseSequentialStream *)&SDU1, "case4=%dus\n", 4);

        		//break;
        	case 5:
        		for(int i=0; i<2000; ++i){ //i<steps+nb_steps
        		  //chprintf((BaseSequentialStream *)&SDU1, "case5=%dus\n", 5);
        		  //chprintf((BaseSequentialStream *)&SDU1, "i=%dus\n", i);
        		  chThdSleepUntilWindowed(time, time + MS2ST(10));
        		  test=1;
        		    }
        		/*while ((steps+nb_steps2)<left_motor_get_pos()){ //trouverle nb_steps2 pour s'orienter vers la balle cette fois.

        			break;
        		}*/
        		/*++position_corrector;
        		break;*/
        	default:
        		break;

        	}

        }
   }

		        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

}

void pi_regulator_start(void){
	//chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}


//verifies if the epuck is alligned behind the ball
int robot_centered(uint16_t final_angle){

uint16_t optimal_angle=0;
optimal_angle=(get_y_goal()-get_y_robot())/(get_x_goal()-get_x_robot()+1);
//chprintf((BaseSequentialStream *)&SDU1, "optimal=%dus\n", optimal_angle);
//chprintf((BaseSequentialStream *)&SDU1, "final_angle=%dus\n", final_angle);

if(test==1){
	optimal_angle=final_angle;
}

	 if (abs(final_angle-optimal_angle)<5) {
	        	return 1;
	 }
	 else {
		 return 0;
	 }
}

int get_y_goal(void){
	return 15;
}

int get_x_goal(void){
	return 5;
}

int get_y_robot(void){
	return 5;
}

int get_x_robot(void){
	return 5;
}

int get_y_ball(void){
	return 5;
}

int get_x_ball(void){
	return 5;
}

int ball_x_position(void){
	return 5;
}

int ball_y_position(void){
	return 5;
}
