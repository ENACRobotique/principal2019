/*
 * MoveToBeeState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOBEESTATE_H_
#define STATEMACHINE_MOVETOBEESTATE_H_

#include "AbstractState.h"

class MoveToBeeState : public AbstractState {
public:
	MoveToBeeState();
	virtual ~MoveToBeeState();

	void doIt();
	void leave();
	void enter();
	void reEnter(unsigned long interruptTime);
	unsigned long get_time_start(){
		return time_start;
	}

private:

	unsigned long time_start;
};

extern MoveToBeeState moveToBeeState;

#endif /* STATEMACHINE_MOVETOBEESTATE_H_ */