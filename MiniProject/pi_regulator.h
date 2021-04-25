#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);
void correct_orientation( int angle);
void move_to_positon(int absolute_distance_steps);
int robot_centered(void);

/*int get_y_goal(){
	return 5;
}

int get_x_goal(){
	return 5;
}

int get_y_robot(){
	return 5;
}

int get_x_robot(){
	return 5;
}

int get_y_ball(){
	return 5;
}

int get_x_ball(){
	return 5;
}*/
#endif /* PI_REGULATOR_H */
