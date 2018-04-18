/*
 * params.h
 *
 *  Created on: 18 mars 2018
 *      Author: elie
 */

#ifndef PARAMS_H_
#define PARAMS_H_
#include "Arduino.h"

const unsigned long TIME_RACE = 100;

const int ENCODEUR1_A = 21;
const int ENCODEUR1_B = 20;
const int ENCODEUR2_A = 17;
const int ENCODEUR2_B = 16;

const int MOT1_PWM = 6;
const int MOT1_DIR = 5;
const int MOT2_PWM = 29;
const int MOT2_DIR = 28;

const int SERVO1 = 39;
const int SERVO2 = 15;
const int SERVO3 = 37;

const int MOT_GALET_L = 35;
const int MOT_GALET_R = 36;

const int COLOR = 26;
const int TIRETTE = 25;

const int SPARE1 = 10;
const int SPARE2 = 9;
const int SPARE3 = 8;
const int SPARE4 = 7;

const int DYNAMIXEL_CONTROL = 30;

const int ACCEL_MAX = 150;
const int SPEED_MAX = 350;

const float ACCEL_OMEGA_MAX = 0.5;
const float OMEGA_MAX = 1;

const float ADMITTED_OMEGA_ERROR = 0.03;
const float ADMITTED_SPEED_ERROR = 10;

const float ADMITTED_POSITION_ERROR = 10;
const float ADMITTED_ANGLE_ERROR = 0.03;

const float MAX_DRIFT = 0.1;

const float WHEEL_DIAMETER = 54.0;
const float WHEELBASE = 199.0;

const float INCR_TO_MM = PI*WHEEL_DIAMETER/800.0;

const float CONTROL_PERIOD = 0.02;
const float NAVIGATOR_TIME_PERIOD = 0.05;

const unsigned long THROW_DURATION = 15;
const unsigned long BEE_LAUNCH_DURATION = 3;

#endif /* PARAMS_H_ */
