/*
 * collision_manager.h
 *
 *  Created on: May 13, 2021
 *      Author: rosendo
 */
#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

uint8_t update_IR_sensors(bool* IR_sensors, uint8_t* ext_active_sensors_idx); /*returns the number of active IR sensors. ext_active_sensors_idx gets the indexes
of the most external active IR sensors (min active index, max active index). If only one active sensor, min active index = max active index*/
bool intermediate_inside_field(int16_t* point);
void set_avoid_collision(bool avoid_param);

void collision_manager_start(void);

#endif /* COLLISION_MANAGER_H */
