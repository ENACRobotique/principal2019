/*
 * MoveToButtonTransition.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOBUTTONTRANSITION_H_
#define STATEMACHINE_MOVETOBUTTONTRANSITION_H_

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToButtonTransition : public AbstractState {
public:
	MoveToButtonTransition();
	virtual ~MoveToButtonTransition();

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

extern MoveToButtonTransition moveToButtonTransition;

#endif /* STATEMACHINE_MOVETOBUTTONTRANSITION_H_ */
