/*
 * MoveToWaterState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToWaterState.h"
#include "ThrowState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"
#include "ExtendArmBeeState.h"

MoveToWaterState moveToWaterState = MoveToWaterState();


float traj_Water_green[][2] = {	{1850,POS_Y_WATER_GREEN},
								{0,0},
								{1950,POS_Y_WATER_GREEN},
								{POS_X_WATER,POS_Y_WATER_GREEN}
};

float traj_Water_orange[][2] = {{1850,POS_Y_WATER_ORANGE},
								{0,0},
								{1950,POS_Y_WATER_ORANGE},
								{POS_X_WATER,POS_Y_WATER_ORANGE}
};

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
	Serial.println("Etat déplacement vers l'abeille");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_Water_green[0][0],traj_Water_green[0][1]);
	}
	else{
		navigator.move_to(traj_Water_orange[0][0],traj_Water_orange[0][1]);
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

	time_start = millis();
}

void MoveToWaterState::leave() {

}

void MoveToWaterState::doIt() {
	if(time_us != 0 && millis() - time_us > TIME_US){
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
		if(trajectory_index == 3){
			fsmSupervisor.setNextState(&throwState);
		}
		else{
			trajectory_index+=1;
			if(trajectory_index == 3){
				if(tiretteState.get_color() == GREEN){
					Odometry::set_pos(1910,POS_Y_WATER_GREEN,0);
				}
				else{
					Odometry::set_pos(1910,POS_Y_WATER_ORANGE,0);
				}
				time_us = millis();
			}
			if(trajectory_index == 1){
				navigator.turn_to(traj_Water_green[trajectory_index][0]);
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
				usManager.setMinRange(&usDistances);
			}
			else{
				if(tiretteState.get_color() == GREEN){
					navigator.move_to(traj_Water_green[trajectory_index][0],traj_Water_green[trajectory_index][1]);
				}
				else{
					Serial.println("Orange");
					navigator.move_to(traj_Water_orange[trajectory_index][0],traj_Water_orange[trajectory_index][1]);
				}

				if(navigator.moveForward()){
					Serial.println("Forward");
					if(trajectory_index==4 or trajectory_index == 2){
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = 0;
						usDistances.rear_right = 0;
					}
					else{
						usDistances.front_left = US_RANGE;
						usDistances.front_right = US_RANGE;
						usDistances.rear_left = 0;
						usDistances.rear_right = 0;
					}
				}
				else{

					Serial.println("Backwards");
					if(trajectory_index == 2){
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = 0;
						usDistances.rear_right = 0;
					}
					else{
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = US_RANGE;
						usDistances.rear_right = US_RANGE;
					}
				}
				usManager.setMinRange(&usDistances);
			}
		}
	}

}

void MoveToWaterState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(time_us != 0){
		time_us += interruptTime;
	}
	if(trajectory_index == 1){
		if(tiretteState.get_color() == GREEN){
			navigator.turn_to(traj_Water_green[trajectory_index][0]);
		}
		else{
			navigator.turn_to(traj_Water_orange[trajectory_index][0]);
		}
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_Water_green[trajectory_index][0],traj_Water_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_Water_orange[trajectory_index][0],traj_Water_orange[trajectory_index][1]);
		}
	}
}

void MoveToWaterState::forceLeave(){
}
