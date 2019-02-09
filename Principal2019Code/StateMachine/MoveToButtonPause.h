/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOBUTTONPAUSE_H_
#define STATEMACHINE_MOVETOBUTTONPAUSE_H_

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToButtonPause : public AbstractState {
public:
	MoveToButtonPause();
	virtual ~MoveToButtonPause();

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

extern MoveToButtonPause moveToButtonPause;

#endif /* STATEMACHINE_MOVETOBUTTONPAUSE_H_ */
