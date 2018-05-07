/*
 * TiretteState.cpp
 *
 *  Created on: 11 avr. 2018
 *      Author: robot
 */

#include "TiretteState.h"
#include "MoveToWaterState.h"
#include "MoveToBeeState.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "DynamixelSerial4.h"
#include "../odometry.h"
#include "../libraries/i2c_t3/i2c_t3.h"
#include "../lib/USManager.h"

TiretteState tiretteState = TiretteState();
Servo arm = Servo();


unsigned long time_us = 0;
TiretteState::TiretteState() {
	time_start = 0;
	flags = E_ULTRASOUND;
	COLOR_BEGIN =1;
}

TiretteState::~TiretteState() {
	// TODO Auto-generated destructor stub
}

void TiretteState::enter() {
	time_start = millis();
	Serial.println("Etat tirette");
	Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
	arm.attach(SERVO3);
	arm.write(RETRACTED_ARM);

	pinMode(TIRETTE,INPUT_PULLUP);
	pinMode(COLOR,INPUT_PULLUP);
    Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, 400000);
	Wire2.setSDA(4);
	Wire2.setSCL(3);
	uint8_t USadresses[] = {0X75, 0X71, 0X70, 0X73};
	usManager.init(USadresses);
}

void TiretteState::leave() {
	if(digitalRead(COLOR) == GREEN){
		Odometry::set_pos(550, 185, 0);
		COLOR_BEGIN = GREEN;
	}
	else{
		Odometry::set_pos(550,2815,0);
		COLOR_BEGIN = ORANGE;
	}
}

void TiretteState::doIt() {
	time_start = millis();
//	USDistances distances =usManager.getRanges();
//	Serial.print("front left:");
//	Serial.print(distances.front_left);
//	Serial.print("\t");
//	Serial.print("front right:");
//	Serial.print(distances.front_right);
//	Serial.print("\t");
//	Serial.print("rear left:");
//	Serial.print(distances.rear_left);
//	Serial.print("\t");
//	Serial.print("rear right:");
//	Serial.println(distances.rear_right);
	if (!digitalRead(TIRETTE)) {
		Serial.println("On change d'etat : gooooo!!");
		time_start = millis();
		fsmSupervisor.setNextState(&moveToBeeState);
	}
//	if(digitalRead(COLOR) == GREEN){
//		Serial.println("GREEN");
//	}
//	else{
//		Serial.println("ORANGE");
//	}
}

void TiretteState::reEnter(unsigned long interruptTime){

}

void TiretteState::forceLeave(){

}

int TiretteState::get_color(){
	return COLOR_BEGIN;
}
