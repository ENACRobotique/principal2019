/*
 * MoveToBeeState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToBeeState.h"
#include "TurnToBeeState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"

MoveToBeeState moveToBeeState = MoveToBeeState();


float traj_bee_green[][2] = { 	{600,200},
								{600,600},
								{1550,600},
								{1550,200},
								{0,0},
								{1950,200}
};

float traj_bee_orange[][2] = {	{600, 2800},
								{600, 2400},
								{1550,2400},
								{1550,2800},
								{0,0},
								{1950,2800}
};

MoveToBeeState::MoveToBeeState() {
	time_start = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 30;
	usDistances.front_right = 30;
	usDistances.rear_left = 30;
	usDistances.rear_right = 30;
}

MoveToBeeState::~MoveToBeeState() {
	// TODO Auto-generated destructor stub
}

void MoveToBeeState::enter() {
	Serial.println("Etat déplacement vers l'abeille");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_bee_green[0][0],traj_bee_green[0][1]);
	}
	else{
		navigator.move_to(traj_bee_orange[0][0],traj_bee_orange[0][1]);
	}

	if(navigator.moveForward()){
		Serial.println("Forward");
		usDistances.front_left = 30;
		usDistances.front_right = 30;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
	}
	else{
		Serial.println("Backwards");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 30;
		usDistances.rear_right = 30;
	}
	usManager.setMinRange(&usDistances);

	time_start = millis();
}

void MoveToBeeState::leave() {

}

void MoveToBeeState::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 5){
			if(tiretteState.get_color() == GREEN){
				Odometry::set_pos(1910,200,0);
			}
			else{
				Odometry::set_pos(1910,2800,0);
			}
			fsmSupervisor.setNextState(&turnToBeeState);
		}
		else{
			trajectory_index+=1;
			if(trajectory_index == 4){
				navigator.turn_to(traj_bee_green[trajectory_index][0]);
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
			}
			else{
				if(tiretteState.get_color() == GREEN){
					navigator.move_to(traj_bee_green[trajectory_index][0],traj_bee_green[trajectory_index][1]);
				}
				else{
					Serial.println("Orange");
					navigator.move_to(traj_bee_orange[trajectory_index][0],traj_bee_orange[trajectory_index][1]);
				}

				if(navigator.moveForward()){
					Serial.println("Forward");
					if(trajectory_index==4){
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = 0;
						usDistances.rear_right = 0;
					}
					else{
						usDistances.front_left = 30;
						usDistances.front_right = 30;
						usDistances.rear_left = 0;
						usDistances.rear_right = 0;
					}
				}
				else{

					Serial.println("Backwards");
					if(trajectory_index==4){
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = 0;
						usDistances.rear_right = 0;
					}
					else{
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = 30;
						usDistances.rear_right = 30;
					}
				}
				usManager.setMinRange(&usDistances);
			}
		}
	}

}

void MoveToBeeState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_bee_green[trajectory_index][0],traj_bee_green[trajectory_index][1]);
	}
	else{
		navigator.move_to(traj_bee_orange[trajectory_index][0],traj_bee_orange[trajectory_index][1]);
	}
}

void MoveToBeeState::forceLeave(){
}
