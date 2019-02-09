/*
 * MoveToButtonState.cpp
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "MoveToButtonState.h"
#include "MoveToCubeTransition.h"
#include "TiretteState.h"
#include "DeadState.h"
#include "../Navigator.h"
#include "Arduino.h"
#include "../params.h"
#include "FSMSupervisor.h"
#include "../lib/USManager.h"
#include "../odometry.h"

MoveToButtonState moveToButtonState = MoveToButtonState();


float traj_button_green[][2] = {{180,0},
								{30,1300},
								{150,1300},
								{-90,0},
								{50,700}
};

float traj_button_orange[][2] ={{180,0},
								{30,1800},
								{150,1800},
								{-90,0},
								{50,2400}
};

MoveToButtonState::MoveToButtonState() {
	time_start = 0;
	time_us = 0;
	flags = E_ULTRASOUND;
	trajectory_index = 0;
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
}

MoveToButtonState::~MoveToButtonState() {
	// TODO Auto-generated destructor stub
}

void MoveToButtonState::enter() {
	Serial.println("Etat déplacement vers l'interrupteur");

	navigator.turn_to(traj_button_green[trajectory_index][0]);
	usDistances.front_left = 0;
	usDistances.front_right = 0;
	usDistances.rear_left = 0;
	usDistances.rear_right = 0;
	usManager.setMinRange(&usDistances);

	time_start = millis();
}

void MoveToButtonState::leave() {

}

void MoveToButtonState::doIt() {
	if(time_us != 0 && millis() - time_us > TIME_US_BUTTON){
		time_us = 0;
		usDistances.front_left = 0;
		usDistances.front_right = 0;
		usDistances.rear_left = 0;
		usDistances.rear_right = 0;
		usManager.setMinRange(&usDistances);
	}

	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 4){
			if(tiretteState.get_color() == GREEN){
				Odometry::set_pos(110,700,180);
			}
			else{
				Odometry::set_pos(110,2400,180);
			}
			Serial.print("New position:");
			Serial.print(Odometry::get_pos_x());
			Serial.print("\t");
			Serial.println(Odometry::get_pos_y());
			fsmSupervisor.setNextState(&moveToCubeTransition);
		}
		else{
			trajectory_index+=1;
			if(trajectory_index == 1){
				time_us = millis();
			}
			if(trajectory_index == 2){
				if(tiretteState.get_color() == GREEN){
					Odometry::set_pos(110,1300,180);
				}
				else{
					Odometry::set_pos(110,1800,180);
				}
				Serial.print("New position:");
				Serial.print(Odometry::get_pos_x());
				Serial.print("\t");
				Serial.println(Odometry::get_pos_y());
			}
			if(trajectory_index == 3){
				navigator.turn_to(traj_button_green[trajectory_index][0]);
				usDistances.front_left = 0;
				usDistances.front_right = 0;
				usDistances.rear_left = 0;
				usDistances.rear_right = 0;
				usManager.setMinRange(&usDistances);
			}
			else{
				if(tiretteState.get_color() == GREEN){
					navigator.move_to(traj_button_green[trajectory_index][0],traj_button_green[trajectory_index][1]);
				}
				else{
					Serial.println("Orange");
					navigator.move_to(traj_button_orange[trajectory_index][0],traj_button_orange[trajectory_index][1]);
				}

				if(navigator.moveForward()){
//					Serial.println("Forward");
					if(trajectory_index == 4){
						Serial.println("Pas d'ultrasons");
						usDistances.front_left = US_RANGE;
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
					if(trajectory_index == 4){
						Serial.println("Pas d'ultrasons");
						usDistances.front_left = 0;
						usDistances.front_right = 0;
						usDistances.rear_left = 0;
						usDistances.rear_right = US_RANGE;
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

void MoveToButtonState::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(time_us != 0){
		time_us+=interruptTime;
	}
	if(trajectory_index == 3){
		if(tiretteState.get_color() == GREEN){
			navigator.turn_to(traj_button_green[trajectory_index][0]);
		}
		else{
			navigator.turn_to(traj_button_orange[trajectory_index][0]);
		}
	}
	else{
		if(tiretteState.get_color() == GREEN){
			navigator.move_to(traj_button_green[trajectory_index][0],traj_button_green[trajectory_index][1]);
		}
		else{
			navigator.move_to(traj_button_orange[trajectory_index][0],traj_button_orange[trajectory_index][1]);
		}
	}
}

void MoveToButtonState::forceLeave(){
}

void MoveToButtonState::pauseNextState(){
}
