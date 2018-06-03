/*
 * MoveToCube2Pause.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToCube2Pause.h"
#include "MoveToCubeState2.h"
#include "DeadState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "../odometry.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"

MoveToCube2Pause moveToCube2Pause = MoveToCube2Pause();

float traj_cube2_pause_green[][2] = {	{1500,845},
										{1500,200},
};

float traj_cube2_pause_orange[][2] = {	{1500, 2165},
										{1500,2800},
};

MoveToCube2Pause::MoveToCube2Pause() {
	time_start = 0;
	time_us = 0;
	trajectory_index = 0;
	flags = E_ULTRASOUND;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToCube2Pause::~MoveToCube2Pause() {
	// TODO Auto-generated destructor stub
}

void MoveToCube2Pause::enter() {
	Serial.println("Etat Pause vers les cubes 2");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_cube2_pause_green[0][0],traj_cube2_pause_green[0][1]);
	}
	else{
		navigator.move_to(traj_cube2_pause_orange[0][0],traj_cube2_pause_orange[0][1]);
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
void MoveToCube2Pause::leave() {

}

void MoveToCube2Pause::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 1){
			fsmSupervisor.setNextState(&moveToCubeState2);
		}
		else{
			trajectory_index+=1;
			if(tiretteState.get_color() == GREEN){
				navigator.move_to(traj_cube2_pause_green[trajectory_index][0],traj_cube2_pause_green[trajectory_index][1]);
			}
			else{
				Serial.println("Orange");
				navigator.move_to(traj_cube2_pause_orange[trajectory_index][0],traj_cube2_pause_orange[trajectory_index][1]);
			}

			if(navigator.moveForward()){
//				Serial.println("Forward");
				usDistances.front_left = US_RANGE_DIMINUSHED;
				usDistances.front_right = US_RANGE_DIMINUSHED;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
			}
			else{
//				Serial.println("Backwards");
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = US_RANGE_DIMINUSHED;
				usDistances.rear_right = US_RANGE_DIMINUSHED;
			}
			usManager.setMinRange(&usDistances);
		}
	}
}


void MoveToCube2Pause::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_cube2_pause_green[trajectory_index][0],traj_cube2_pause_green[trajectory_index][1]);
	}
	else{
		navigator.move_to(traj_cube2_pause_orange[trajectory_index][0],traj_cube2_pause_orange[trajectory_index][1]);
	}
}

void MoveToCube2Pause::forceLeave(){

}

void MoveToCube2Pause::pauseNextState(){

}

