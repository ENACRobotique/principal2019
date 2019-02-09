/*
 * ThrowState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_THROWSTATE_H_
#define STATEMACHINE_THROWSTATE_H_

#include "AbstractState.h"

class ThrowState : public AbstractState {
public:
	ThrowState();
	virtual ~ThrowState();

	void doIt();
	void leave();
	void enter();
	void reEnter(unsigned long interruptTime);
	void forceLeave();
	void pauseNextState();
	unsigned long get_time_start(){
		return time_start;
	}

private:

	unsigned long time_start;
	unsigned long MOTOR_START_DURATION;
	unsigned long VIBRATION_DURATION;
	unsigned long time_last_vibration;
	bool dynamixel_not_started;
	int vibration_index;
};

extern ThrowState throwState;

#endif /* STATEMACHINE_THROWSTATE_H_ */
