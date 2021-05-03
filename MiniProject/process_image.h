#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 640
#define LINES_TO_ANALYSE 5 //strange results with 3
#define MINIMUM_BALL_SIZE 7
#define RGB_nb 3

void process_image_start(void);
bool is_green(uint8_t* pixel);
bool is_orange(uint16_t* pixel);

#endif /* PROCESS_IMAGE_H */
