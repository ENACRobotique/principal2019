/*
 * Ultrasound.h
 *
 *  Created on: 18 avr. 2018
 *      Author: robot
 */

#ifndef LIB_ULTRASOUND_H_
#define LIB_ULTRASOUND_H_

#include "Arduino.h"

class Ultrasound {
public:

	Ultrasound();
	Ultrasound(uint8_t addr);
	virtual ~Ultrasound();
	int update(bool doRange);

	bool obstacleDetected();

	uint8_t getAddress() const {
		return address;
	}

	void setAddress(uint8_t address) {
		this->address = address;
	}

	uint16_t getMinRange() const {
		return min_range;
	}

	void setMinRange(uint16_t minRange) {
		min_range = minRange;
	}

	uint16_t getRange() const {
		return range;
	}

	unsigned long getRangeTime() const {
		return range_time;
	}

private:
	//! Address of the ultrasound sensor
	uint8_t address;

	//! Measured distance to obstacle
	uint16_t range;

	//! Minimum acceptable distance from obstacle
	uint16_t min_range;

	//! Time of last call of startRange
	unsigned long range_time;

	//! True while last value not read
	bool ranging;

	int startRange();

	void readRangeResult();
};

#endif /* LIB_ULTRASOUND_H_ */
