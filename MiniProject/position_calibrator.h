/*
 * position_calibrator.h
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */

#ifndef POSITION_CALIBRATOR_H_
#define POSITION_CALIBRATOR_H_

void position_calibrator_start(void); //starts the thread used for calibration
int get_is_calibrating(void);

void calibrate(void); //to start the calibration

#endif /* POSITION_CALIBRATOR_H_ */
