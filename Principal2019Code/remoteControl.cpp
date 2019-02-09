/*
 * remoteControl.cpp
 *
 *  Created on: 18 sept. 2018
 *      Author: Maxime Le Du
 */

#include "remoteControl.h"
#include "motorControl.h"
#include "Secondary2018Code.h"
#include "params.h"
#include "Arduino.h"


RemoteController remoteController = RemoteController();

RemoteController::RemoteController(){
	data[0]='2';
	data[1]='0';
	data[2]='0';
	data[3]='2';
	data[4]='0';
	data[5]='0';
	data[6]='\0';
	index = 0;
}

void RemoteController::set_cons(){
	int x,y,index;
	float v,omega;
	char str_x[4],str_y[4];
	for(index=0;index<3;index++){
		str_x[index] = data[index];
		str_y[index] = data[3+index];
	}
	str_x[3]='\0';
	str_y[3]='\0';
	sscanf(str_x, "%d",&x);
	sscanf(str_y,"%d",&y);
	y -= 200;
	x -= 200;
	Serial.print(y*SPEED_REMOTE);
	Serial.print(" ");
	Serial.println(-x*OMEGA_REMOTE);
	v  = y*SPEED_REMOTE;
	if(v>=REMOTE_THRESHOLD){
		omega = -x*OMEGA_REMOTE;
	}
	else{
		omega = x*OMEGA_REMOTE;
	}
		MotorControl::set_cons(v,omega);
}


bool RemoteController::parse_data(){
	if(Serial1.available()){
		char c= Serial1.read();
		if(c == 3) {
			return false;
		}
		if(c == 2) {
			index = 0;
			return true;
		}

	   data[index] = c;
	   index++;
	  }
	/*if(Serial1.available()){
		Serial.println("Le serial est available !");

		if(Serial1.peek() == -1){
			data[0]='2';
			data[1]='0';
			data[2]='0';
			data[3]='2';
			data[4]='0';
			data[5]='0';
		}
		else{
			while(Serial1.peek()!=' '){
				data[index] = Serial1.read();
				index++;
			}
		}
	}*/

	/*for(index=0;index<6;index++){
		data[index] = Serial.read();
	}
	Serial.read();*/
	return false;
}


void RemoteController::update(){
	if(parse_data()){
		set_cons();
	}
}
