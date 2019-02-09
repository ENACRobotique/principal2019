/*
 * CalibrateBeeState.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_CALIBRATEBEESTATE_H_
#define STATEMACHINE_CALIBRATEBEESTATE_H_

#include "AbstractState.h"
#include "../lib/USManager.h"

class CalibrateBeeState : public AbstractState {
public:
	CalibrateBeeState();
	virtual ~CalibrateBeeState();

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

extern CalibrateBeeState calibrateBeeState;

#endif /* STATEMACHINE_CALIBRATEBEESTATE_H_ */
