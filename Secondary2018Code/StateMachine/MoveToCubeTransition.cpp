/*
 * MoveToCubeTransition.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToCubeTransition.h"
#include "MoveToCubeState.h"
#include "MoveToCube2Pause.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "../odometry.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"

MoveToCubeTransition moveToCubeTransition = MoveToCubeTransition();

float traj_cube_trans_green[][2] = {	{1400,POS_Y_WATER_GREEN},
										{1400,845},
};

float traj_cube_trans_orange[][2] = {	{1400, POS_Y_WATER_ORANGE},
										{1400,2165},
};

MoveToCubeTransition::MoveToCubeTransition() {
	time_start = 0;
	time_us = 0;
	trajectory_index = 0;
	flags = E_ULTRASOUND;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToCubeTransition::~MoveToCubeTransition() {
	// TODO Auto-generated destructor stub
}

void MoveToCubeTransition::enter() {
	Serial.println("Etat transition vers les cubes");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_cube_trans_green[0][0],traj_cube_trans_green[0][1]);
	}
	else{
		navigator.move_to(traj_cube_trans_orange[0][0],traj_cube_trans_orange[0][1]);
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
void MoveToCubeTransition::leave() {

}

void MoveToCubeTransition::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 1){
			fsmSupervisor.setNextState(&moveToCubeState);
		}
		else{
			trajectory_index+=1;
			if(tiretteState.get_color() == GREEN){
				navigator.move_to(traj_cube_trans_green[trajectory_index][0],traj_cube_trans_green[trajectory_index][1]);
			}
			else{
				Serial.println("Orange");
				navigator.move_to(traj_cube_trans_orange[trajectory_index][0],traj_cube_trans_orange[trajectory_index][1]);
			}

			if(navigator.moveForward()){
//				Serial.println("Forward");
				usDistances.front_left = US_RANGE;
				usDistances.front_right = US_RANGE;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
			}
			else{
//				Serial.println("Backwards");
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = US_RANGE;
				usDistances.rear_right = US_RANGE;
			}
			usManager.setMinRange(&usDistances);
		}
	}
}


void MoveToCubeTransition::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_cube_trans_green[trajectory_index][0],traj_cube_trans_green[trajectory_index][1]);
	}
	else{
		navigator.move_to(traj_cube_trans_orange[trajectory_index][0],traj_cube_trans_orange[trajectory_index][1]);
	}
}

void MoveToCubeTransition::forceLeave(){

}

void MoveToCubeTransition::pauseNextState(){
	fsmSupervisor.setNextState(&moveToCube2Pause);
}
