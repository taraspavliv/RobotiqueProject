/*
 * role_selector.h
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */
#ifndef ROLE_SELECTOR_H_
#define ROLE_SELECTOR_H_

#define IS_GOALKEEPER(selector_position) (selector_position >= 0x03 && selector_position <= 0x05)
#define IS_SMART(selector_position) (selector_position >= 0x06 && selector_position <= 0x0A)
#define IS_ATTACKER(selector_position) (selector_position >= 0x0B && selector_position <= 0x0D)

enum Role{GOALKEEPER, ATTACKER, SMART, CONTROLLED};

void role_selector_start(void);
enum Role get_role(void);

#endif /* ROLE_SELECTOR_H_ */
