/*
 * TiretteState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "DeadState.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../Navigator.h"

DeadState deadState = DeadState();

DeadState::DeadState() {

}

DeadState::~DeadState() {
	// TODO Auto-generated destructor stub
}

void DeadState::enter() {
	Serial.println("Etat mort");
	navigator.forceStop();
}

void DeadState::leave() {

}

void DeadState::doIt() {
	//Le robot ne fait plus rien dans cet état
}

void DeadState::reEnter(unsigned long interruptTime){

}

void DeadState::forceLeave(){

}

void DeadState::pauseNextState(){

}
