#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

enum BallVisibility{FULL, PARTIAL, CAM_OVERFLOW, NONE};

void process_image_start(void);

bool first_ball_estimate(uint8_t *img_buff_ptr, uint8_t* main_lines_averaged, uint16_t* left_ball_side_ref, uint16_t* right_ball_side_ref);
void final_ball_estimate(uint8_t* main_lines_averaged, uint16_t* left_ball_side_ref, uint16_t* right_ball_side_ref);
void update_ball_pos_mov(uint16_t left_ball_side, uint16_t right_ball_side);

int16_t* get_ball_position(void);
int16_t* get_ball_movement(void);
enum BallVisibility get_ball_visibility(void);

#endif /* PROCESS_IMAGE_H */
