/*
 * communication.cpp
 *
 *  Created on: 19 févr. 2019
 *      Author: elie et adrien et fabien
 */

#include "communication.h"

Velocity _velocity;

Message _message;
ReceivingState _receiving_state = IDLE;

Message make_pos_vel_message(float x, float y, float theta, float speed, float omega) {
	Message msg;
	msg.length = 12;// ID + message utile (10 octets) + CHECKSUM
	msg.id = (uint8_t)POS_VEL;
	msg.payload.pos_vel.x = (uint16_t)(x+XY_ADDER);
	msg.payload.pos_vel.y = (uint16_t)(y+XY_ADDER);
	msg.payload.pos_vel.theta = (uint16_t)((theta + RADIAN_TO_MSG_ADDER) * RADIAN_TO_MSG_FACTOR);
	msg.payload.pos_vel.speed = (uint16_t)(speed+SPEED_ADDER);
	msg.payload.pos_vel.omega = (uint16_t)((omega * ANGULAR_SPEED_TO_MSG_FACTOR) + ANGULAR_SPEED_TO_MSG_ADDER) ;

	uint8_t checksum = msg.length + msg.id;
	for(size_t i=0; i<sizeof(Pos_vel);i++){
		checksum += msg.payload.data[i];
	}
	checksum = ~checksum;
	msg.checksum = checksum;
	return msg;
}

void send_message(Message msg){
	uint8_t buf[msg.length+3]; // buffer = start1 + srart2 + lenght + lenght_message
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	memcpy(buf+2, &msg, msg.length);  // msg.lenght - checksum + lenght
	buf[msg.length + 2] = msg.checksum;

	Serial1.write(buf, msg.length + 3);
}

float get_omega_received(Message* p_message){
	float omega_received = (p_message->payload.velocity.omega - ANGULAR_SPEED_TO_MSG_ADDER) / ANGULAR_SPEED_TO_MSG_FACTOR;
	return omega_received;
}

float get_speed_received(Message* p_message){
	float speed_received = p_message->payload.velocity.speed - SPEED_ADDER;
	return speed_received;
}

float get_x_received(Message* p_message){
	float x_received = p_message->payload.position.x - XY_ADDER;
	return x_received;
}

float get_y_received(Message* p_message){
	float y_received = p_message->payload.position.y - XY_ADDER;
	return y_received;
}

float get_theta_received(Message* p_message){
	float theta_received = (p_message->payload.position.theta/RADIAN_TO_MSG_FACTOR - RADIAN_TO_MSG_ADDER);
	return theta_received;
}



//------------------------------Lecture message--------------------------------

int receive_message(void){
	int retour = 0; // 0 : rien à signaler. pas d'erreur. Le message n'est pas complet ou y a rien à lire
	uint8_t b;
	uint8_t checksum_readed;
	if(Serial1.available()>_message.length){
		if(_receiving_state==IDLE){
			b=Serial1.read();
			if(b==0xFF){
				_receiving_state = INIT1;
			}
		}
		if(_receiving_state==INIT1){
			if((b=Serial1.read())==0xFF){
				_receiving_state = INIT2;
			}
			else {
				_receiving_state = IDLE;
			}
		}
		if(_receiving_state==INIT2){
			_message.length = Serial1.read();
			_message.checksum = _message.length;
			_receiving_state = READ;
		}
		if(_receiving_state==READ){
			_message.id = Serial1.read();
			_message.checksum += _message.id;
			for (int i =0; i< _message.length-2; i++){
				(_message.payload).data[i] = Serial1.read();
				_message.checksum += (_message.payload).data[i];
			}
			_message.checksum = ~_message.checksum;
			checksum_readed = Serial1.read();
			if(checksum_readed==_message.checksum){
				retour = 1; // message lu et ok
			}
			else{
				retour = -1; //erreur de checksum
			}
			_message.length = 0;
			_receiving_state = IDLE;
		}
	}
	return retour;
}

void get_received_message(Message* msg) {
	msg->checksum = _message.checksum;
	msg->id = _message.id;
	msg->length = _message.length;
	memcpy(&msg->payload, &_message.payload, sizeof(Pos_vel));
}