/*
 * params.h
 *
 *  Created on: 18 mars 2018
 *      Author: elie
 */

#ifndef PARAMS_H_
#define PARAMS_H_
#include "Arduino.h"

const unsigned long TIME_RACE = 100000;

const int ENCODEUR1_A = 3;
const int ENCODEUR1_B = 2;
const int ENCODEUR2_A = 5;
const int ENCODEUR2_B = 4;

const int MOT1_PWM = 7;
const int MOT1_DIR = 6;
const int MOT2_PWM = 9;
const int MOT2_DIR = 8;

const int LID_PIN_OUT1 = 16;
const int LID_PIN_OUT2 = 17;
const int LID_PIN_OUT3 = 18;

const int LID_PIN_IN1 = 23;
const int LID_PIN_IN2 = 22;
const int LID_PIN_IN3 = 21;
const int LID_PIN_IN4 = 20;
const int LID_PIN_IN5 = 19;

const int SERVO1 = 2;
//const int SERVO2 = 15;
const int SERVO3 = 37;

const int MOT_GALET_L = 35;

const int POMPE = 15;
const int VANN = 14;

const int LED = 13;

const int COLOR = 25;
const int TIRETTE = 24;

//const int SPARE1 = 10;
//const int SPARE2 = 9;
//const int SPARE3 = 8;
//const int SPARE4 = 7;

const int DYNAMIXEL_CONTROL = 32;
const int DYN_MAX_SPEED=750;
const int DYN_BROADCAST_ID = 254 ;

const int DYN_HOLDER_ID = 6 ;
const int DYN_HOLDER_UP=220;
const int DYN_HOLDER_DOWN=875;

const int PIN_HOLDER_LOCK = 22 ;
const int HOLDER_LOCK_OPEN=80;
const int HOLDER_LOCK_LOCK=112;

const int PIN_DUMBO_EAR = 21 ;
const int DUMBO_EAR_OPEN=75;
const int DUMBO_EAR_CLOSE=105;

const int PIN_LOCK=23;
const int LOCK_OPEN=45;
const int LOCK_LOCK=80;

const int PIN_TROMPE_POMPE=30;
const int TROMPE_POMPE_ON=HIGH;
const int TROMPE_POMPE_OFF=LOW;

const int DYN_TROMPE_ID = 1 ;
const int DYN_TROMPE_OUTSIDE=215;
const int DYN_TROMPE_INSIDE=800;


const int EMERGENCY_BRAKE = 2500;
const int ACCEL_MAX = 800;
const int ACCEL_MAX_THROW = 100;
const int SPEED_MAX = 500;

const float SPEED_REMOTE = 4;
const float OMEGA_REMOTE = 0.02;

const int US_RANGE = 0;
const int US_RANGE_DIMINUSHED = 0;

const float ACCEL_OMEGA_MAX = 2;
const float OMEGA_MAX = 2.5;

const float ADMITTED_OMEGA_ERROR = 0.03;
const float ADMITTED_SPEED_ERROR = 5;

const float ADMITTED_POSITION_ERROR = 5;
const float ADMITTED_ANGLE_ERROR = 0.01;

const float MAX_DRIFT = 0.1;

const float WHEEL_DIAMETER = 71.906;
const float WHEELBASE = 245.72;

const float INCR_TR = 500.0;
const float REDUCTION = 676/49;
const float INCR_TO_MM = PI*WHEEL_DIAMETER/(INCR_TR*REDUCTION);

const float CONTROL_PERIOD = 0.02;
const float NAVIGATOR_TIME_PERIOD = 0.05;
const float RECEIVE_TIME_PERIOD = 0.01;
const float REMOTE_PERIOD = 0.02;
const float REMOTE_THRESHOLD = 40;

const unsigned long THROW_DURATION = 20000;
const unsigned long SERVO_MOVEMENT_DURATION = 1000;
const unsigned long DETECTION_STOP_TIME = 1500;
const unsigned long PAUSE_TIME = 20000;

const unsigned long COMMAND_TIMEOUT = 1000;

const int RETRACTED_ARM = 45;
const int EXTENDED_ARM = 105;

const int GREEN = 1;
const int ORANGE = !GREEN;

const int NB_US = 4;

#endif /* PARAMS_H_ */
