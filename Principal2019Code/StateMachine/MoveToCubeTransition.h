/*
 * MoveToCubeTransition.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOCUBETRANSITION_H_
#define STATEMACHINE_MOVETOCUBETRANSITION_H_

#define TIME_US_CUBE 3000

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToCubeTransition : public AbstractState {
public:
	MoveToCubeTransition();
	virtual ~MoveToCubeTransition();

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

extern MoveToCubeTransition moveToCubeTransition;

#endif /* STATEMACHINE_MOVETOCUBETRANSITION_H_ */
