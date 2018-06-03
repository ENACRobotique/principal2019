/*
 * MoveToButtonTransition.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToButtonTransition.h"
#include "MoveToButtonState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"

MoveToButtonTransition moveToButtonTransition = MoveToButtonTransition();


float traj_button_trans_green[][2] = {{1400,POS_Y_WATER_GREEN},
								{1400,1300}
};

float traj_button_trans_orange[][2] = {{1400, POS_Y_WATER_ORANGE},
								{1400,1800}
};

MoveToButtonTransition::MoveToButtonTransition() {
	time_start = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToButtonTransition::~MoveToButtonTransition() {
	// TODO Auto-generated destructor stub
}

void MoveToButtonTransition::enter() {
	Serial.println("Etat transition vers l'interrupteur");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_button_trans_green[0][0],traj_button_trans_green[0][1]);
	}
	else{
		navigator.move_to(traj_button_trans_orange[0][0],traj_button_trans_orange[0][1]);
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

void MoveToButtonTransition::leave() {

}

void MoveToButtonTransition::doIt() {

	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 1){
			fsmSupervisor.setNextState(&moveToButtonState);
		}
		else{
			trajectory_index+=1;
			if(tiretteState.get_color() == GREEN){
				navigator.move_to(traj_button_trans_green[trajectory_index][0],traj_button_trans_green[trajectory_index][1]);
			}
			else{
				Serial.println("Orange");
				navigator.move_to(traj_button_trans_orange[trajectory_index][0],traj_button_trans_orange[trajectory_index][1]);
			}

			if(navigator.moveForward()){
//				Serial.println("Forward");
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


void MoveToButtonTransition::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_button_trans_green[trajectory_index][0],traj_button_trans_green[trajectory_index][1]);
	}
	else{
		navigator.move_to(traj_button_trans_orange[trajectory_index][0],traj_button_trans_orange[trajectory_index][1]);
	}
}

void MoveToButtonTransition::forceLeave(){
}

void MoveToButtonTransition::pauseNextState(){
}
