/*
 * ThrowState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "ThrowState.h"

#include "TiretteState.h"
#include "MoveToButtonTransition.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "Servo.h"

#include "../libraries/DynamixelSerial5/DynamixelSerial5.h"
#include "../Navigator.h"
#include "MoveToCubeTransition.h"

ThrowState throwState = ThrowState();

float vibration_green[] = {POS_X_WATER_GREEN +20, POS_X_WATER_GREEN -20};
float vibration_orange[] = {POS_X_WATER_ORANGE +20, POS_X_WATER_GREEN -20};

ThrowState::ThrowState() {
	time_start = 0;
	MOTOR_START_DURATION = 1500;
	VIBRATION_DURATION = 1500;
	dynamixel_not_started = true;
	time_last_vibration = 0;
	vibration_index = 0;
}

ThrowState::~ThrowState() {
	// TODO Auto-generated destructor stub
}

void ThrowState::enter() {
	Serial.println("Etat throw");
	time_start = millis();
	time_last_vibration = millis();
	analogWrite(MOT_GALET_L,66);
}

void ThrowState::leave() {
	analogWrite(MOT_GALET_L,0);
	//Dynamixel.turn(DYNAMIXEL_ID,false,0);
}

void ThrowState::doIt() {
	if (millis() - time_start > THROW_DURATION) {
//		fsmSupervisor.setNextState(&moveToCubeTransition);
		fsmSupervisor.setNextState(&moveToCubeTransition);
	}

	if((millis() - time_start > MOTOR_START_DURATION)&& dynamixel_not_started){
		//Dynamixel.setEndless(DYNAMIXEL_ID,true);
		//Dynamixel.turn(DYNAMIXEL_ID,true,1023);
		dynamixel_not_started = false;
	}

	if(millis() - time_last_vibration > VIBRATION_DURATION ){
			time_last_vibration = millis();
			if(digitalRead(COLOR) == GREEN){
				navigator.throw_to(vibration_green[vibration_index],POS_Y_WATER_GREEN,-0.02);
			}
			else{
				navigator.throw_to(vibration_orange[vibration_index],POS_Y_WATER_ORANGE,0.02);
			}
			vibration_index = (vibration_index+1)%2;
	}

}

void ThrowState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	analogWrite(MOT_GALET_L,66);
	dynamixel_not_started = true;
}

void ThrowState::forceLeave(){
	analogWrite(MOT_GALET_L,0);
	//Dynamixel.turn(DYNAMIXEL_ID,false,0);
}

void ThrowState::pauseNextState(){

}
