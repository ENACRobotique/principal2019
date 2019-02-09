/*
 * MoveToCubePause.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToCubePause.h"
#include "MoveToCube2Pause.h"
#include "DeadState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "../odometry.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "MoveToCubeState.h"

MoveToCubePause moveToCubePause = MoveToCubePause();


MoveToCubePause::MoveToCubePause() {
	time_start = 0;
	flags = E_ULTRASOUND;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToCubePause::~MoveToCubePause() {
	// TODO Auto-generated destructor stub
}

void MoveToCubePause::enter() {
	Serial.println("Etat rupture vers les cubes");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(1400,845);
	}
	else{
		navigator.move_to(1400,2155);
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
void MoveToCubePause::leave() {

}

void MoveToCubePause::doIt() {
	if(navigator.isTrajectoryFinished()){
		fsmSupervisor.setNextState(&moveToCubeState);
	}
}

void MoveToCubePause::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(1400,845);
	}
	else{
		navigator.move_to(1400,2155);
	}
}


void MoveToCubePause::forceLeave(){

}

void MoveToCubePause::pauseNextState(){
	fsmSupervisor.setNextState(&moveToCube2Pause);
}
