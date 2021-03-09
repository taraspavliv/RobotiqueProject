#ifndef MOTOR_H
#define MOTOR_H


void motor_init1(void);
void motor_set_speed(float speed_r, float speed_l);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void motor_stop(void);
extern uint8_t out1;
extern uint8_t out2;
#endif /* MOTOR_H */
