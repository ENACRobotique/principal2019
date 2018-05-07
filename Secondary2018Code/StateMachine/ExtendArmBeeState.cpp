 /*
* ExtendArmBeeState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "ExtendArmBeeState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "MoveBeeLaunchState.h"

ExtendArmBeeState extendArmBeeState = ExtendArmBeeState();

ExtendArmBeeState::ExtendArmBeeState() {
	time_start = 0;
	time_servo = 0;
}

ExtendArmBeeState::~ExtendArmBeeState() {
	// TODO Auto-generated destructor stub
}

void ExtendArmBeeState::enter() {
	Serial.println("Etat rotation vers l'abeille");
	arm.write(EXTENDED_ARM);
	time_start = millis();
}

void ExtendArmBeeState::leave() {

}

void ExtendArmBeeState::doIt() {
	if(millis() - time_servo > SERVO_MOVEMENT_DURATION){
		fsmSupervisor.setNextState(&moveBeeLaunchState);
	}

}

void ExtendArmBeeState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	arm.write(EXTENDED_ARM);
}

void ExtendArmBeeState::forceLeave(){

}
