/*
 * DeadState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_DEADSTATE_H_
#define STATEMACHINE_DEADSTATE_H_

#include "AbstractState.h"

class DeadState : public AbstractState {
public:
	DeadState();
	virtual ~DeadState();

	void doIt();
	void leave();
	void enter();
	void reEnter(unsigned long interruptTime);
	void forceLeave();
	void pauseNextState();
};

extern DeadState deadState;

#endif /* STATEMACHINE_DEADSTATE_H_ */
