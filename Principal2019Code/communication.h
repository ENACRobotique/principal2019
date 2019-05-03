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

const float SPEED_ADDER = (1<<15);
const float XY_ADDER = (1<<15);
const float RADIAN_TO_MSG_FACTOR = 1000.0;
const float RADIAN_TO_MSG_ADDER = PI;
const float ANGULAR_SPEED_TO_MSG_FACTOR = ((1<<15)/30.0); //max omega=30rad/s
const float ANGULAR_SPEED_TO_MSG_ADDER = (1<<15); //split uint16 in two

enum MessagesID{
	//up
	POS_VEL,
	BUTTONS,
	VOLTAGE,
	ACK,
	//down
	VELOCITY,
	POSITION,
	PUMP,
	GATE,
	DYN
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

typedef struct __attribute__((__packed__)) Velocity{
	uint16_t speed;
	uint16_t omega;
}Velocity;

typedef struct __attribute__((__packed__)) Position{
	uint16_t x;
	uint16_t y;
	uint16_t theta;
}Position;

typedef struct __attribute__((__packed__)) US{
	uint16_t front_left;
	uint16_t front_right;
	uint16_t rear_left;
	uint16_t rear_right;
}US;

typedef struct __attribute__((__packed__)) Pump{
	uint8_t activation;
}Pump;

typedef struct __attribute__((__packed__)) Gate{
	uint8_t activation;
}Gate;

typedef struct __attribute__((__packed__)) Ack{
	uint8_t acknowledgement;
}Ack;

typedef struct __attribute__((__packed__)) Dyn{
	uint16_t angle;
	uint16_t speed;
}Dyn;


union  __attribute__((__packed__)) Payload{
	Pos_vel pos_vel;
	Buttons buttons;
	Ack ack;
	Velocity velocity;
	Position position;
	US us;
	Pump pump;
	Gate gate;
	Dyn dyn;
	uint8_t data[sizeof(Pos_vel)];
};

typedef struct Message{
	uint8_t length;
	uint8_t id;
	union Payload payload;
	uint8_t checksum;
}Message;

Message make_pos_vel_message(float x, float y, float theta, float speed, float omega);
Message make_ack_message(void);

void send_message(Message msg);
int receive_message();

void get_received_message(Message* msg);

float get_omega_received(Message* p_message);
float get_speed_received(Message* p_message);

float get_x_received(Message* p_message);
float get_y_received(Message* p_message);
float get_theta_received(Message* p_message);

int get_pump_received(Message* p_message);
int get_gate_received(Message* p_message);

int get_dynSpeed_received(Message* p_message);
int get_dynAngle_received(Message* p_message);

extern Velocity _velocity;

#endif
