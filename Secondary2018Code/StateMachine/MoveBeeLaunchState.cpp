 /*
* MoveBeeLaunchState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveBeeLaunchState.h"

#include "CalibrateBeeState.h"
#include "DeadState.h"
#include "RetractArmBeeState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "TiretteState.h"
#include "FSMSupervisor.h"

MoveBeeLaunchState moveBeeLaunchState = MoveBeeLaunchState();

MoveBeeLaunchState::MoveBeeLaunchState() {
	time_start = 0;
	time_servo = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveBeeLaunchState::~MoveBeeLaunchState() {
	// TODO Auto-generated destructor stub
}

void MoveBeeLaunchState::enter() {
	Serial.println("Etat rotation vers l'abeille");
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(1830,300);
	}
	else{
		navigator.move_to(1830,2700);
	}
	if(navigator.moveForward()){
		Serial.println("Forward");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
	}
	else{
		Serial.println("Backwards");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
	}
	usManager.setMinRange(&usDistances);
	time_start = millis();
}

void MoveBeeLaunchState::leave() {

}

void MoveBeeLaunchState::doIt() {
	if(navigator.isTrajectoryFinished()){
		if(time_servo == 0){
			time_servo = millis();
			arm.write(RETRACTED_ARM);
		}
		if(millis() - time_servo > SERVO_MOVEMENT_DURATION){
			fsmSupervisor.setNextState(&calibrateBeeState);
		}
	}
}

void MoveBeeLaunchState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(navigator.isTrajectoryFinished()){
		arm.write(RETRACTED_ARM);
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(1830,300);
		}
		else{
			navigator.move_to(1830,2700);
		}
		if(navigator.moveForward()){
			Serial.println("Forward");
			usDistances.front_left = 0;
			usDistances.front_right = 0;
			usDistances.rear_left = 0;
			usDistances.rear_right = 0;
		}
		else{
			Serial.println("Backwards");
			usDistances.front_left = 0;
			usDistances.front_right = 0;
			usDistances.rear_left = 0;
			usDistances.rear_right = 0;
		}
		usManager.setMinRange(&usDistances);
	}
}

void MoveBeeLaunchState::forceLeave(){

}
