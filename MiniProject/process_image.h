#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

enum BallVisibility{FULL, PARTIAL, CAM_OVERFLOW, NONE};
//FULL means the ball is entirely in the camera angle of view
//PARTIAL mean the ball isn't entirely visible by the camera
//NONE means no ball is seen with the camera
//CAM_OVERFLOW means the ball is too close to the camera and takes the whole

void process_image_start(void); //starts one tread to acquire the image and one to process the image

int16_t* get_ball_position(void); //returns an array[2] with the estimated (x,y) coordinates of the ball
int16_t* get_ball_movement(void); //returns an array[2] with the estimated (dx/dt, dy/dt) speed vector of the ball
enum BallVisibility get_ball_visibility(void); //returns the visibility of the ball

#endif /* PROCESS_IMAGE_H */
