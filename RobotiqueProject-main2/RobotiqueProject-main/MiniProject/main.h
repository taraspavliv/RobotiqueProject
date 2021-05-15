#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define FIELD_WIDTH 600 //in mm
#define FIELD_HEIGHT 1000 //in mm

/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

float distance_between(int16_t* point1, int16_t* point2);
uint32_t distance_squared(int16_t* point1, int16_t* point2);
int16_t attacking_score(int16_t* player_pos, int16_t* ball_pos, int16_t* goal_pos);


#ifdef __cplusplus
}
#endif

#endif
