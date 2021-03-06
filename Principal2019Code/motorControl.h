/*
 * motorControl.h
 *
 *  Created on: 20 mars 2018
 *      Author: elie
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_


namespace MotorControl {
	extern unsigned long time_last_command;
	void update();
	void init();
	void reset();
	float get_cons_speed();
	float get_cons_omega();

	void set_cons(float speed, float omega);
	void set_radius(float speed, float radius);
}


#endif /* MOTORCONTROL_H_ */
