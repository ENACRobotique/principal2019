/*
 * MoveToCubeState2.cpp
 *
 *  Created on: 5 mai 2018
 *      Author: Maxime
 */

#include "MoveToCubeState2.h"
#include "DeadState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "../odometry.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"

MoveToCubeState2 moveToCubeState2 = MoveToCubeState2();

float traj_cube2_green[][2] = {{150,0},
							   {170,500}
};

float traj_cube2_orange[][2] = {{150,0},
								{170,2500}
};

MoveToCubeState2::MoveToCubeState2() {
	time_start = 0;
	us_time = 0;
	trajectory_index = 0;
	flags = E_ULTRASOUND;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToCubeState2::~MoveToCubeState2() {
	// TODO Auto-generated destructor stub
}

void MoveToCubeState2::enter() {
	Serial.println("Etat déplacement vers les cubes 2");

	if(tiretteState.get_color() == GREEN){
		navigator.turn_to(traj_cube2_green[0][0]);
	}
	else{
		navigator.turn_to(traj_cube2_orange[0][0]);
	}
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
	usManager.setMinRange(&usDistances);

	time_start = millis();
}
void MoveToCubeState2::leave() {

}

void MoveToCubeState2::doIt() {
	if(us_time != 0 && millis() - us_time > US_TIME_CUBE2){
		Serial.print("Plus d'ultrasons");
		us_time = 0;
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
		usManager.setMinRange(&usDistances);
	}
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 1){
			fsmSupervisor.setNextState(&deadState);
		}
		else{
			trajectory_index+=1;
			us_time = millis();
			if(tiretteState.get_color() == GREEN){
				navigator.move_to(traj_cube2_green[trajectory_index][0],traj_cube2_green[trajectory_index][1]);
			}
			else{
				Serial.println("Orange");
				navigator.move_to(traj_cube2_orange[trajectory_index][0],traj_cube2_orange[trajectory_index][1]);
			}

			if(navigator.moveForward()){
				Serial.println("Forward");
				usDistances.front_left = US_RANGE;
				usDistances.front_right = US_RANGE;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
			}
			else{
				Serial.println("Backwards");
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = US_RANGE;
				usDistances.rear_right = US_RANGE;
			}
			usManager.setMinRange(&usDistances);
		}
	}

}

void MoveToCubeState2::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(us_time != 0){
		us_time+=interruptTime;
	}
	if(trajectory_index == 1){
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_cube2_green[trajectory_index][0],traj_cube2_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_cube2_orange[trajectory_index][0],traj_cube2_orange[trajectory_index][1]);
		}
	}
	else{
		navigator.turn_to(traj_cube2_green[trajectory_index][0]);
	}
}

void MoveToCubeState2::forceLeave(){

}

void MoveToCubeState2::pauseNextState(){
}

