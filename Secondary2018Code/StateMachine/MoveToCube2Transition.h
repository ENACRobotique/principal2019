/*
 * MoveToCube2Transition.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOCUBE2TRANSITION_H_
#define STATEMACHINE_MOVETOCUBE2TRANSITION_H_

#define TIME_US_CUBE 3000

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToCube2Transition : public AbstractState {
public:
	MoveToCube2Transition();
	virtual ~MoveToCube2Transition();

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

	unsigned long trajectory_index;
	unsigned long time_start;
	unsigned long time_us;
	USDistances usDistances;
};

extern MoveToCube2Transition moveToCube2Transition;

#endif /* STATEMACHINE_MOVETOCUBE2TRANSITION_H_ */
