/*
 * MoveToWaterPause.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToCubePause.h"
#include "MoveToButtonPause.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"
#include "MoveToWaterPause.h"
#include "MoveToWaterState.h"

MoveToWaterPause moveToWaterPause = MoveToWaterPause();


float traj_water_pause_green[][2] = {	{1400,POS_Y_WATER_GREEN},
										{-90,0},
										{1400,50},
										{1400,POS_Y_WATER_GREEN},
										{0,0}
};

float traj_water_pause_orange[][2] = {	{1400,POS_Y_WATER_ORANGE},
										{-90,0},
										{1400,2950},
										{1400,POS_Y_WATER_ORANGE},
										{0,0}
};

MoveToWaterPause::MoveToWaterPause() {
	time_start = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToWaterPause::~MoveToWaterPause() {
	// TODO Auto-generated destructor stub
}

void MoveToWaterPause::enter() {
	Serial.println("Etat rupture vers l'eau");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_water_pause_green[0][0],traj_water_pause_green[0][1]);
	}
	else{
		navigator.move_to(traj_water_pause_orange[0][0],traj_water_pause_orange[0][1]);
	}

	if(navigator.moveForward()){
//		Serial.println("Forward");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
	}
	else{
//		Serial.println("Backwards");
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
	}
	usManager.setMinRange(&usDistances);

	time_start = millis();
}

void MoveToWaterPause::leave() {

}

void MoveToWaterPause::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 4){
			fsmSupervisor.setNextState(&moveToWaterState);
		}
		else{
			trajectory_index+=1;
			if(trajectory_index == 3){
				if(tiretteState.get_color() == GREEN){
					Odometry::set_pos(1400,110,-90);
				}
				else{
					Odometry::set_pos(1400,2890,-90);
				}
			}
			if(trajectory_index == 1 or trajectory_index ==4){
				navigator.turn_to(traj_water_pause_green[trajectory_index][0]);
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
				usManager.setMinRange(&usDistances);
			}
			else{
				if(tiretteState.get_color() == GREEN){
					navigator.move_to(traj_water_pause_green[trajectory_index][0],traj_water_pause_green[trajectory_index][1]);
				}
				else{
					navigator.move_to(traj_water_pause_orange[trajectory_index][0],traj_water_pause_orange[trajectory_index][1]);
				}
				if(trajectory_index == 2){
					usDistances.front_left = 0;
					usDistances.front_right = 0;
					usDistances.rear_left = 0;
					usDistances.rear_right = 0;
				}
				else{
					if(navigator.moveForward()){
						usDistances.front_left = 30;
						usDistances.front_right = 30;
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

void MoveToWaterPause::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(trajectory_index == 1 or trajectory_index == 4){
		if(tiretteState.get_color() == GREEN){
			navigator.turn_to(traj_water_pause_green[trajectory_index][0]);
		}
		else{
			navigator.turn_to(traj_water_pause_orange[trajectory_index][0]);
		}
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_water_pause_green[trajectory_index][0],traj_water_pause_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_water_pause_orange[trajectory_index][0],traj_water_pause_orange[trajectory_index][1]);
		}
	}
}

void MoveToWaterPause::forceLeave(){
}

void MoveToWaterPause::pauseNextState(){
	fsmSupervisor.setNextState(&moveToCubePause);
}
