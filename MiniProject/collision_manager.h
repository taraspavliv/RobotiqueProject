/*
 * collision_manager.h
 *
 *  Created on: May 13, 2021
 *      Author: rosendo
 */
#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

void set_avoid_collision(bool avoid_param); //we can set to false when we don't want to avoid objects, for example when shooting the ball

void collision_manager_start(void); //starts the thread that manages the IR sensors

bool object_right_in_front(void); //returns true if the 2 most front IR sensors see something

#endif /* COLLISION_MANAGER_H */
