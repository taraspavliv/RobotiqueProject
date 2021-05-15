/*
 * role_selector.c
 *
 *  Created on: May 8, 2021
 *      Author: taras
 */
#include "ch.h"
#include <selector.h>

#include "role_selector.h"

#define IS_GOALKEEPER(selector_position) (selector_position >= 0x03 && selector_position <= 0x05)
#define IS_SMART(selector_position) (selector_position >= 0x06 && selector_position <= 0x0A)
#define IS_ATTACKER(selector_position) (selector_position >= 0x0B && selector_position <= 0x0D)

static enum Role epuck_role = SMART;

static THD_WORKING_AREA(waSelectRole, 128);
static THD_FUNCTION(SelectRole, arg) {

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  uint8_t selector_position = 0;
  while(true) {
	  //sets epucks role based on selector position
	  selector_position = get_selector();
	  if(IS_GOALKEEPER(selector_position)){
		  epuck_role = GOALKEEPER;
	  }else if(IS_ATTACKER(selector_position)){
		  epuck_role = ATTACKER;
	  }else if(IS_SMART(selector_position)){
		  epuck_role = SMART;
	  }else{
		  epuck_role = CONTROLLED;
	  }
	  //doesn't need to be very reactive, so long sleep time
	  chThdSleepMilliseconds(500);
  }
}

enum Role get_role(void){
	return epuck_role;
}

void role_selector_start(void){
	chThdCreateStatic(waSelectRole, sizeof(waSelectRole), NORMALPRIO-1, SelectRole, NULL);
}
