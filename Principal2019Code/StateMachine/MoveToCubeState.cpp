/*
 * MoveToCubeState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToCubeState.h"
#include "MoveToCube2Pause.h"
#include "MoveToCube2Transition.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "../odometry.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"

MoveToCubeState moveToCubeState = MoveToCubeState();

float traj_cube_green[][2] = {	{0,0},
								{200,855}
};

float traj_cube_orange[][2] = {	{0,0},
								{200,2155}
};

MoveToCubeState::MoveToCubeState() {
	time_start = 0;
	time_us = 0;
	trajectory_index = 0;
	flags = E_ULTRASOUND;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToCubeState::~MoveToCubeState() {
	// TODO Auto-generated destructor stub
}

void MoveToCubeState::enter() {
	Serial.println("Etat déplacement vers les cubes");
	navigator.turn_to(0);
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
	usManager.setMinRange(&usDistances);

	time_start = millis();
}
void MoveToCubeState::leave() {

}

void MoveToCubeState::doIt() {
	if(time_us != 0 && (millis() - time_us > TIME_US_CUBE)){
		Serial.println("Plus d'ultrasons");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
		usManager.setMinRange(&usDistances);
		time_us = 0;
	}
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 1){
			if(tiretteState.get_color() == GREEN){
				Odometry::set_pos(200,855,0);
			}
			else{
				Odometry::set_pos(200,2155,0);
			}
			fsmSupervisor.setNextState(&moveToCube2Transition);
		}
		else{
			trajectory_index+=1;
			time_us = millis();

			if(tiretteState.get_color() == GREEN){
				navigator.move_to(traj_cube_green[trajectory_index][0],traj_cube_green[trajectory_index][1]);
			}
			else{
				Serial.println("Orange");
				navigator.move_to(traj_cube_orange[trajectory_index][0],traj_cube_orange[trajectory_index][1]);
			}

			if(navigator.moveForward()){
				usDistances.front_left = US_RANGE;
				usDistances.front_right = US_RANGE;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
			}
			else{
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = US_RANGE;
				usDistances.rear_right = US_RANGE;
			}
			usManager.setMinRange(&usDistances);
		}
	}
}

void MoveToCubeState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(time_us != 0){
		time_us+=interruptTime;
	}
	if(trajectory_index == 0){
		navigator.turn_to(0);
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_cube_green[trajectory_index][0],traj_cube_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_cube_orange[trajectory_index][0],traj_cube_orange[trajectory_index][1]);
		}
	}
}

void MoveToCubeState::forceLeave(){

}

void MoveToCubeState::pauseNextState(){
	fsmSupervisor.setNextState(&moveToCube2Pause);
}
