/*
 * MoveToWaterState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToWaterState.h"
#include "MoveToCubePause.h"
#include "MoveToButtonPause.h"
#include "ThrowState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"

MoveToWaterState moveToWaterState = MoveToWaterState();


MoveToWaterState::MoveToWaterState() {
	time_start = 0;
	flags = E_ULTRASOUND;
	time_us = 0;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToWaterState::~MoveToWaterState() {
	// TODO Auto-generated destructor stub
}

void MoveToWaterState::enter() {
	Serial.println("Etat déplacement vers l'eau");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(POS_X_WATER_GREEN,POS_Y_WATER_GREEN);
	}
	else{
		navigator.move_to(POS_X_WATER_ORANGE,POS_Y_WATER_ORANGE);
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
	time_us = millis();
}

void MoveToWaterState::leave() {

}

void MoveToWaterState::doIt() {
	if(time_us != 0 && millis() - time_us > TIME_US){
		Serial.println("Plus d'ultrasons");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
		usManager.setMinRange(&usDistances);
		time_us = 0;
	}
	if(navigator.isTrajectoryFinished()){
		fsmSupervisor.setNextState(&throwState);
	}
}

void MoveToWaterState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(time_us != 0){
		time_us += interruptTime;
	}
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(POS_X_WATER_GREEN,POS_Y_WATER_GREEN);
	}
	else{
		navigator.move_to(POS_X_WATER_ORANGE,POS_Y_WATER_ORANGE);
	}
}

void MoveToWaterState::forceLeave(){
}

void MoveToWaterState::pauseNextState(){
	fsmSupervisor.setNextState(&moveToCubePause);
}
