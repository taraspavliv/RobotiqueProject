/*
 * position_calibrator.h
 *
 *  Created on: May 8, 2021
 *      Author: rosendo
 */

#ifndef POSITION_CALIBRATOR_H_
#define POSITION_CALIBRATOR_H_

#define FIELD_WIDTH 600 //in mm
#define FIELD_HEIGHT 1000 //in mm

void position_calibrator_start(void);

void calibrate(void);

#endif /* POSITION_CALIBRATOR_H_ */
