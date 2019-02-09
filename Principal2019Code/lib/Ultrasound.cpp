/*
 * Ultrasound.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: robot
 */

#include "Ultrasound.h"
//#include "WireKinetis.h"
#include "../libraries/i2c_t3/i2c_t3.h"
#include "Arduino.h"

Ultrasound::Ultrasound() {
	address = 0;
	range = 0;
	min_range = 0;
	range_time = 0;
	ranging = false;
}

Ultrasound::Ultrasound(uint8_t addr) {
	address = addr;
	range = 0;
	min_range = 0;
	range_time = 0;
	ranging = false;
}

Ultrasound::~Ultrasound() {
	// TODO Auto-generated destructor stub
}

int Ultrasound::update(bool doRange) {
	int error = 0;
	if(ranging && millis()-range_time > 70){
		readRangeResult();
		ranging = false;
	}
	if(doRange && !ranging) {
		error = startRange();
	}
	return error;
}

bool Ultrasound::obstacleDetected() {
	if(min_range == 0 || range == 0) {
		return false;
	}
	return range < min_range;
}

int Ultrasound::startRange() {
	// send command to start ranging
	Wire2.beginTransmission(address);
	Wire2.write(byte(0x00));      // Write to register 0
	Wire2.write(byte(0x51));      // start ranging: measure in inches (0x50), centimeters (0x51) or ping µseconds (0x52)
	int error = (int)Wire2.endTransmission();
	if(!error){
	  ranging = true;
	}
	range_time = millis();
	return error;
}

void Ultrasound::readRangeResult() {
	// ask data
	Wire2.beginTransmission(address);
	Wire2.write(byte(0x02));// Read data starting by register 2

	if(Wire2.endTransmission()){
		return;
	}

	Wire2.requestFrom(address, uint8_t(2));    // Request 2 bytes

	// wait for it
	while(Wire2.available() < 2);

	// read, prepare and return data
	range = Wire2.read() << 8;
	range |= Wire2.read();
}
