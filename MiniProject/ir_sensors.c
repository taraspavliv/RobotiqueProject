/*
 * ir_sensors.c
 *
 *  Created on: May 13, 2021
 *      Author: rosendo
 */

#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "ir_sensors.h"
#include "sensors/proximity.h"


static THD_WORKING_AREA(waSensor_IR, 128);
static THD_FUNCTION(Sensor_IR, arg) {

	proximity_start();
	calibrate_ir();

 /** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

  chRegSetThreadName(__FUNCTION__);
  (void)arg;

  uint8_t selector_position = 0;
  while(true) {
	  messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	  chprintf((BaseSequentialStream *) &SD3, "x:%d \r\n", get_calibrated_prox(1));

	  //doesn't need to be very reactive, so long sleep time
	  chThdSleepMilliseconds(500);
  }
}


void role_selector_start(void){
	chThdCreateStatic(waSensor_IR, sizeof(waSensor_IR), NORMALPRIO-1, Sensor_IR, NULL);
}
