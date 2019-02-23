/*
 * communication.h
 *
 *  Created on: 19 févr. 2019
 *      Author: robot
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

//#ifndef DEBUG_COMM
//#define DEBUG_COMM 0
//#endif

#include <Arduino.h>
#include <HardwareSerial.h>
#include "params.h"

#define SPEED_ADDER (1<<15)
#define XY_ADDER (1<<15)
#define RADIAN_TO_MSG_FACTOR 1000
#define RADIAN_TO_MSG_ADDER PI
#define ANGULAR_SPEED_TO_MSG_FACTOR ((1<<15)/30.0) //max omega=30rad/s
#define ANGULAR_SPEED_TO_MSG_ADDER (1<<15) //split uint16 in two

enum MessagesID{
	//up
	POS_VEL,
	BUTTONS,
	VOLTAGE,
	//down
	VELOCITY
};

enum ReceivingState{
	IDLE,
	INIT1,
	INIT2,
	READ
};

typedef struct __attribute__((__packed__)) Pos_vel{
	uint16_t x;
	uint16_t y;
	uint16_t theta;
	uint16_t speed;
	uint16_t omega;
}Pos_vel;

typedef struct  __attribute__((__packed__)) Buttons{
	uint8_t tirette;
	uint8_t color;
}Buttons;

union  __attribute__((__packed__)) Payload{
	Pos_vel pos_vel;
	Buttons buttons;
	uint8_t data[sizeof(Pos_vel)];
};

typedef struct Velocity{
	float speed;
	float omega;
}Velocity;

typedef struct Message{
	uint8_t length;
	uint8_t id;
	union Payload payload;
	uint8_t checksum;
	ReceivingState state;
	int inprogress;
}Message;

Message make_pos_vel_message(float x, float y, float theta, float speed, float omega);

void send_message(Message msg);
void receive_message(Message* p_message);
void velocity_decode(Message* p_message, Velocity* p_velocity);

extern Velocity _velocity;

#endif
