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

float traj_cube2_green[][2] = { {550,200},
								{550,700},
								{1550,700},
								{1550,300},
								{200,650}
};

float traj_cube2_orange[][2] = {{550,2800},
								{550,2300},
								{1550, 2300},
								{1550,2700},
								{200,2350}
};

MoveToCubeState2::MoveToCubeState2() {
	time_start = 0;
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
	Serial.println("Etat déplacement vers l'abeille");

	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_cube2_green[0][0],traj_cube2_green[0][1]);
	}
	else{
		navigator.move_to(traj_cube2_orange[0][0],traj_cube2_orange[0][1]);
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
void MoveToCubeState2::leave() {

}

void MoveToCubeState2::doIt() {
	if(navigator.isTrajectoryFinished()){
		Serial.print("trajectory:");
		Serial.println(trajectory_index);
		if(trajectory_index == 3){
			fsmSupervisor.setNextState(&deadState);
			if(tiretteState.get_color() == GREEN){
				Odometry::set_pos(260,860,0);
			}
			else{
				Odometry::set_pos(260,2140,0);
			}
		}
		else{
			trajectory_index+=1;
			if(tiretteState.get_color() == GREEN){
				navigator.move_to(traj_cube2_green[trajectory_index][0],traj_cube2_green[trajectory_index][1]);
			}
			else{
				Serial.println("Orange");
				navigator.move_to(traj_cube2_orange[trajectory_index][0],traj_cube2_orange[trajectory_index][1]);
			}

			if(navigator.moveForward()){
				Serial.println("Forward");
				if(trajectory_index==2){
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
				if(trajectory_index==2){
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

void MoveToCubeState2::reEnter(unsigned long interruptTime){
	time_start+=interruptTime;
	if(tiretteState.get_color() == GREEN){
		navigator.move_to(traj_cube2_green[trajectory_index][0],traj_cube2_green[trajectory_index][1]);
	}
	else{
		navigator.move_to(traj_cube2_orange[trajectory_index][0],traj_cube2_orange[trajectory_index][1]);
	}
}

void MoveToCubeState2::forceLeave(){

}
