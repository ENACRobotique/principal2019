/*
 * MoveToWaterState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOWATERSTATE_H_
#define STATEMACHINE_MOVETOWATERSTATE_H_

#define TIME_US 2000

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToWaterState : public AbstractState {
public:
	MoveToWaterState();
	virtual ~MoveToWaterState();

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
	unsigned long time_us;
};

extern MoveToWaterState moveToWaterState;

#endif /* STATEMACHINE_MOVETOWATERSTATE_H_ */
