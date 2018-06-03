/*
 * MoveToBeeState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOBEESTATE_H_
#define STATEMACHINE_MOVETOBEESTATE_H_

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToBeeState : public AbstractState {
public:
	MoveToBeeState();
	virtual ~MoveToBeeState();

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
	int trajectory_index;
	USDistances usDistances;
};

extern MoveToBeeState moveToBeeState;

#endif /* STATEMACHINE_MOVETOBEESTATE_H_ */
