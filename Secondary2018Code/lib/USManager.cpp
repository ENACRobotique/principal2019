/*
 * USManager.cpp
 *
 *  Created on: 19 avr. 2018
 *      Author: robot
 */

#include "USManager.h"
#include "Ultrasound.h"
#include "Arduino.h"
#include "../Navigator.h"

USManager usManager = USManager();

USManager::USManager() {
	current_us = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

USManager::~USManager() {
	// TODO Auto-generated destructor stub
}

void USManager::init(uint8_t* tab_address) {
	for(int i=0;i<NB_US;i++){
		tab_ultrasound[i] = new Ultrasound(tab_address[i]);
	}
}

void USManager::setMinRange(USDistances* distances) {
	tab_ultrasound[FRONT_LEFT_INDEX]->setMinRange(distances->front_left);
	tab_ultrasound[FRONT_RIGHT_INDEX]->setMinRange(distances->front_right);
	tab_ultrasound[REAR_LEFT_INDEX]->setMinRange(distances->rear_left);
	tab_ultrasound[REAR_RIGHT_INDEX]->setMinRange(distances->rear_right);
}

void USManager::update() {
	unsigned long time_current_us = millis() - tab_ultrasound[current_us]->getRangeTime();
	tab_ultrasound[current_us]->update(false);
	if(time_current_us > 200){
		current_us = (current_us+1)%NB_US;
		tab_ultrasound[current_us]->update(true);
	}
}

bool USManager::obstacleDetected() {
	bool obstacle_detected = false;
	if(navigator.get_move_state() != INITIAL_TURN){
		for(int i=0;i<NB_US;i++){
			obstacle_detected = obstacle_detected || tab_ultrasound[i]->obstacleDetected();
		}
	}
	return obstacle_detected;
}

USDistances USManager::getRanges() {
	usDistances.front_left = tab_ultrasound[FRONT_LEFT_INDEX]->getRange();
	usDistances.front_right = tab_ultrasound[FRONT_RIGHT_INDEX]->getRange();
	usDistances.rear_left = tab_ultrasound[REAR_LEFT_INDEX]->getRange();
	usDistances.rear_right = tab_ultrasound[REAR_RIGHT_INDEX]->getRange();
	return usDistances;
}
