/*
 * USManager.h
 *
 *  Created on: 19 avr. 2018
 *      Author: robot
 */

#ifndef LIB_USMANAGER_H_
#define LIB_USMANAGER_H_
#include "Arduino.h"
#include "Ultrasound.h"
#include "../params.h"

#define FRONT_LEFT_INDEX 2
#define FRONT_RIGHT_INDEX 3
#define REAR_RIGHT_INDEX 0
#define REAR_LEFT_INDEX 1

typedef struct {
	uint16_t front_left;
	uint16_t front_right;
	uint16_t rear_right;
	uint16_t rear_left;
} USDistances;

class USManager {
public:
	USManager();
	virtual ~USManager();



	void init(uint8_t *tab_address);

	void setMinRange(USDistances*);

	void update();

	bool obstacleDetected();

	USDistances getRanges();


private:
	Ultrasound *tab_ultrasound[NB_US];
	int current_us;
	USDistances usDistances;
};

extern USManager usManager;
#endif /* LIB_USMANAGER_H_ */
