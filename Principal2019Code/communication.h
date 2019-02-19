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

enum MessagesID{
	POS_VEL,
	BUTTONS,
	VOLTAGE
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

typedef struct Message{
	uint8_t length;
	uint8_t id;
	union Payload payload;
	uint8_t checksum;
}Message;

Message make_pos_vel_message(float x, float y, float theta, float speed, float omega);

void send_message(Message msg);

#endif
