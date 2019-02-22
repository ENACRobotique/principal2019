/*
 * communication.cpp
 *
 *  Created on: 19 févr. 2019
 *      Author: elie et adrien et fabien
 */

#include "communication.h"


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
	Serial.print("uninverted: ");
	Serial.print(checksum);

	checksum = ~checksum;
	 msg.checksum = checksum;
	 Serial.print("\tinverted: ");
	 Serial.println(checksum);
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
