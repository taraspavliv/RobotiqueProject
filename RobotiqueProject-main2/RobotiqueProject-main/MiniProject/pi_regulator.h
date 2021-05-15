#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);
void correct_orientation( int angle);
void move_to_positon(int absolute_distance_steps);
int robot_centered(uint16_t final_angle);

int get_x_goal(void);
int get_y_goal(void);

int get_x_robot(void);
int get_y_robot(void);

int get_x_ball(void);
int get_y_ball(void);

int ball_x_position(void);
int ball_y_position(void);
#endif /* PI_REGULATOR_H */
