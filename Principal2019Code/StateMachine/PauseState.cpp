/*
 * PauseState.cpp
 *
 *  Created on: 19 avr. 2018
 *      Author: robot
 */

#include "PauseState.h"
#include "Arduino.h"
#include "../Navigator.h"
#include "../params.h"

PauseState pauseState = PauseState();
PauseState::PauseState() {
	pauseStartTime = 0;
	flags = E_ULTRASOUND;
	isLong = false;
}

PauseState::~PauseState() {
	// TODO Auto-generated destructor stub
}

void PauseState::doIt() {
	if(millis() - pauseStartTime > PAUSE_TIME){
		isLong = true;
	}
}

void PauseState::leave() {
}

void PauseState::enter() {
	Serial.println("Etat pause");
	pauseStartTime = millis();
	navigator.forceStop();
	isLong = false;
}

void PauseState::reEnter(unsigned long interruptTime) {
}

void PauseState::forceLeave() {
}


unsigned long PauseState::getPauseTime() {
	return millis() - pauseStartTime;
}

bool PauseState::isTooLong(){
	return isLong;
}

void PauseState::pauseNextState(){

}
