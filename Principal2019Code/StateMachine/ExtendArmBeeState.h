/*
 * ExtendArmBeeState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_EXTENDARMBEESTATE_H_
#define STATEMACHINE_EXTENDARMBEESTATE_H_

#include "AbstractState.h"

class ExtendArmBeeState : public AbstractState {
public:
	ExtendArmBeeState();
	virtual ~ExtendArmBeeState();

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
	unsigned long time_servo;
};

extern ExtendArmBeeState extendArmBeeState;

#endif /* STATEMACHINE_EXTENDARMBEESTATE_H_ */
