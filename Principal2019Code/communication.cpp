/*
 * communication.cpp
 *
 *  Created on: 19 févr. 2019
 *      Author: elie et adrien et fabien
 */

#include "communication.h"


Velocity _velocity;

ReceivingState receiving_state = IDLE;

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
	 //checksum = ~checksum; //cf AX12 protocol
	//Serial.print("uninverted: ");
	//Serial.print(checksum);

	checksum = ~checksum;
	 msg.checksum = checksum;
	 //Serial.print("\tinverted: ");
	 //Serial.println(checksum);
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

void velocity_decode(Message* p_message, Velocity* p_velocity){
	uint16_t speed_readed;
	uint16_t omega_readed;
	speed_readed = (p_message->payload).data[0];
	speed_readed = (speed_readed<<8) | (p_message->payload).data[1];
	p_velocity->speed = (float)speed_readed - SPEED_ADDER;

	omega_readed = (p_message->payload).data[2];
	omega_readed = (omega_readed<<8) | (p_message->payload).data[3];
	p_velocity->omega = ((float)omega_readed - ANGULAR_SPEED_TO_MSG_ADDER) / ANGULAR_SPEED_TO_MSG_FACTOR;

}

//------------------------------Lecture message--------------------------------

void receive_message(Message* p_message){
	uint8_t b;
	uint8_t checksum_readed;
	p_message->inprogress = 1;
	if(Serial1.available()>p_message->length){
		switch(p_message->state){
		case IDLE:
			if((b=Serial1.read())==0xFF){
				p_message->state = INIT1;
			}
			break;
		case INIT1:
			if((b=Serial1.read())==0xFF){
				p_message->state = INIT2;
			}
			else {
				p_message->state = IDLE;
			}
			break;
		case INIT2:
			p_message->length = Serial1.read();
			p_message->checksum += p_message->length;
			p_message->state = READ;
			break;
		case READ:
			p_message->id = Serial1.read();
			p_message->checksum += p_message->id;
			for (int i =0; i< p_message->length-2; i++){
				(p_message->payload).data[i] = Serial1.read();
				p_message->checksum += (p_message->payload).data[i];
			}
			checksum_readed = Serial1.read();
			if(checksum_readed==p_message->checksum){
				p_message->inprogress = 0;
			}
			p_message->length = 0;
			p_message->id = 0;
			p_message->checksum = 0;

			break;
		}
	}
}








