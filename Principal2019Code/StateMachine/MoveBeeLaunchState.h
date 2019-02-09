/*
 * MoveBeeLaunchState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVEBEELAUNCHSTATE_H_
#define STATEMACHINE_MOVEBEELAUNCHSTATE_H_

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveBeeLaunchState : public AbstractState {
public:
	MoveBeeLaunchState();
	virtual ~MoveBeeLaunchState();

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
	unsigned long time_servo;
	unsigned long time_start;
	USDistances usDistances;
};

extern MoveBeeLaunchState moveBeeLaunchState;

#endif /* STATEMACHINE_MOVEBEELAUNCHSTATE_H_ */
