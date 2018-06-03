/*
 * MoveToButtonState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOBUTTONSTATE_H_
#define STATEMACHINE_MOVETOBUTTONSTATE_H_

#include "AbstractState.h"
#include "../lib/USManager.h"

#define TIME_US_BUTTON 3000

class MoveToButtonState : public AbstractState {
public:
	MoveToButtonState();
	virtual ~MoveToButtonState();

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
	unsigned long time_us;
	int trajectory_index;
	USDistances usDistances;
};

extern MoveToButtonState moveToButtonState;

#endif /* STATEMACHINE_MOVETOBUTTONSTATE_H_ */
