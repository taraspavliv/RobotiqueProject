/*
 * role_selector.h
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */
#ifndef ROLE_SELECTOR_H_
#define ROLE_SELECTOR_H_

enum Role{GOALKEEPER, ATTACKER, SMART, CONTROLLED};

void role_selector_start(void); //starts the thread for the selector
enum Role get_role(void); //returns the role based on the selector position


#endif /* ROLE_SELECTOR_H_ */
