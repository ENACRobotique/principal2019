/*
 * MoveToButtonPause.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToButtonPause.h"
#include "MoveToButtonState.h"
#include "TiretteState.h"
#include "DeadState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"

MoveToButtonPause moveToButtonPause = MoveToButtonPause();

MoveToButtonPause::MoveToButtonPause() {
	time_start = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToButtonPause::~MoveToButtonPause() {
	// TODO Auto-generated destructor stub
}

void MoveToButtonPause::enter() {
	Serial.println("Etat rupture vers l'interrupteur");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(1400,1300);
	}
	else{
		navigator.move_to(1400,1800);
	}

	if(navigator.moveForward()){
//		Serial.println("Forward");
		usDistances.front_left = US_RANGE;
		usDistances.front_right = US_RANGE;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
	}
	else{
//		Serial.println("Backwards");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = US_RANGE;
		usDistances.rear_right = US_RANGE;
	}
	usManager.setMinRange(&usDistances);

	time_start = millis();
}

void MoveToButtonPause::leave() {

}

void MoveToButtonPause::doIt() {

	if(navigator.isTrajectoryFinished()){
		fsmSupervisor.setNextState(&moveToButtonState);
	}
}


void MoveToButtonPause::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(1400,1300);
	}
	else{
		navigator.move_to(1400,1700);
	}
}

void MoveToButtonPause::forceLeave(){
}

void MoveToButtonPause::pauseNextState(){
}
