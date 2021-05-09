/*
 * position_motion_controller.c
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */
#include <position_motion_controller.h>




/*
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
stepl_1=left_motor_get_pos();*/
//chprintf((BaseSequentialStream *)&SD3, "x:%f y:%f theta:%f \n\r", position_x, position_y, my_angle);
