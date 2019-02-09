/*
 * MoveToWaterTrans.cpp
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
#include "MoveToWaterTransition.h"
#include "MoveToWaterState.h"

MoveToWaterTrans moveToWaterTrans = MoveToWaterTrans();


float traj_water_trans_green[][2] = {	{1850,POS_Y_WATER_GREEN},
								{0,0},
								{1950,POS_Y_WATER_GREEN}
};

float traj_water_trans_orange[][2] = {{1850,POS_Y_WATER_ORANGE},
								{0,0},
								{1950,POS_Y_WATER_ORANGE}
};

MoveToWaterTrans::MoveToWaterTrans() {
	time_start = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToWaterTrans::~MoveToWaterTrans() {
	// TODO Auto-generated destructor stub
}

void MoveToWaterTrans::enter() {
	Serial.println("Etat transition vers l'eau");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_water_trans_green[0][0],traj_water_trans_green[0][1]);
	}
	else{
		navigator.move_to(traj_water_trans_orange[0][0],traj_water_trans_orange[0][1]);
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

void MoveToWaterTrans::leave() {

}

void MoveToWaterTrans::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 2){
			if(tiretteState.get_color() == GREEN){
				Odometry::set_pos(1910,POS_Y_WATER_GREEN,0);
			}
			else{
				Odometry::set_pos(1910,POS_Y_WATER_ORANGE,0);
			}
			Serial.print("New position:");
			Serial.print(Odometry::get_pos_x());
			Serial.print("\t");
			Serial.println(Odometry::get_pos_y());
			fsmSupervisor.setNextState(&moveToWaterState);
		}
		else{
			trajectory_index+=1;
			if(trajectory_index == 1){
				navigator.turn_to(traj_water_trans_green[trajectory_index][0]);
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
				usManager.setMinRange(&usDistances);
			}
			else{
				if(tiretteState.get_color() == GREEN){
					navigator.move_to(traj_water_trans_green[trajectory_index][0],traj_water_trans_green[trajectory_index][1]);
				}
				else{
					Serial.println("Orange");
					navigator.move_to(traj_water_trans_orange[trajectory_index][0],traj_water_trans_orange[trajectory_index][1]);
				}

				if(navigator.moveForward()){
//					Serial.println("Forward");
					if(trajectory_index == 2){
						Serial.println("Pas d'ultrasons");
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

//					Serial.println("Backwards");
					if(trajectory_index == 2){
						Serial.println("Pas d'ultrasons");
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

void MoveToWaterTrans::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(trajectory_index == 1){
		if(tiretteState.get_color() == GREEN){
			navigator.turn_to(traj_water_trans_green[trajectory_index][0]);
		}
		else{
			navigator.turn_to(traj_water_trans_orange[trajectory_index][0]);
		}
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_water_trans_green[trajectory_index][0],traj_water_trans_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_water_trans_orange[trajectory_index][0],traj_water_trans_orange[trajectory_index][1]);
		}
	}
}

void MoveToWaterTrans::forceLeave(){
}

void MoveToWaterTrans::pauseNextState(){
	fsmSupervisor.setNextState(&moveToCubePause);
}
