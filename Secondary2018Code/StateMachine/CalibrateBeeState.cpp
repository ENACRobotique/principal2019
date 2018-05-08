/*
 * CalibrateBeeState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "CalibrateBeeState.h"
#include "ExtendArmBeeState.h"
#include "TiretteState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"
#include "MoveToWaterState.h"

CalibrateBeeState calibrateBeeState = CalibrateBeeState();


float traj_calibrate_green[][2] = { {1830,170},
									{0,0},
									{1950,170},
									{1850,170},
									{-90,0},
									{1850,50}
};

float traj_calibrate_orange[][2] = {{1830,2830},
									{0,0},
									{1950,2830},
									{1850,2830},
									{-90,0},
									{1850,2950}
};

CalibrateBeeState::CalibrateBeeState() {
	time_start = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

CalibrateBeeState::~CalibrateBeeState() {
	// TODO Auto-generated destructor stub
}

void CalibrateBeeState::enter() {
	Serial.println("Etat déplacement vers l'abeille");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_calibrate_green[0][0],traj_calibrate_green[0][1]);
	}
	else{
		navigator.move_to(traj_calibrate_orange[0][0],traj_calibrate_orange[0][1]);
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

void CalibrateBeeState::leave() {

}

void CalibrateBeeState::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 5){
			fsmSupervisor.setNextState(&extendArmBeeState);
			if(tiretteState.get_color() == GREEN){
				Odometry::set_pos(1850,110,-90);
			}
			else{
				Odometry::set_pos(1850,2890,-90);
			}
		}
		else{
			trajectory_index+=1;
			if(trajectory_index == 3){
				if(tiretteState.get_color() == GREEN){
					Odometry::set_pos(1890,170,0);
				}
				else{
					Odometry::set_pos(1890,2830,0);
				}
			}
			if(trajectory_index == 1 or trajectory_index == 4){
				navigator.turn_to(traj_calibrate_green[trajectory_index][0]);
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
				usManager.setMinRange(&usDistances);
			}
			else{
				if(tiretteState.get_color() == GREEN){
					navigator.move_to(traj_calibrate_green[trajectory_index][0],traj_calibrate_green[trajectory_index][1]);
				}
				else{
					Serial.println("Orange");
					navigator.move_to(traj_calibrate_orange[trajectory_index][0],traj_calibrate_orange[trajectory_index][1]);
				}

				if(navigator.moveForward()){
					Serial.println("Forward");
					if(trajectory_index == 2){
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

void CalibrateBeeState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(trajectory_index == 1 or trajectory_index == 4){
		if(tiretteState.get_color() == GREEN){
			navigator.turn_to(traj_calibrate_green[trajectory_index][0]);
		}
		else{
			navigator.turn_to(traj_calibrate_orange[trajectory_index][0]);
		}
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_calibrate_green[trajectory_index][0],traj_calibrate_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_calibrate_orange[trajectory_index][0],traj_calibrate_orange[trajectory_index][1]);
		}
	}
}

void CalibrateBeeState::forceLeave(){
}
