/*
 * MoveToWaterState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOWATERSTATE_H_
#define STATEMACHINE_MOVETOWATERSTATE_H_

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
	unsigned long get_time_start(){
		return time_start;
	}

private:

	unsigned long time_start;
	USDistances usDistances;
};

extern MoveToWaterState moveToWaterState;

#endif /* STATEMACHINE_MOVETOWATERSTATE_H_ */
