/*
 * PauseState.h
 *
 *  Created on: 19 avr. 2018
 *      Author: robot
 */

#ifndef STATEMACHINE_PAUSESTATE_H_
#define STATEMACHINE_PAUSESTATE_H_
#include "AbstractState.h"

class PauseState : public AbstractState {
public:
	PauseState();
	virtual ~PauseState();

	void doIt();
	void leave();
	void enter();
	void reEnter(unsigned long interruptTime);
	void forceLeave();
	void pauseNextState();
	unsigned long getPauseTime();
	bool isTooLong();

private:
	unsigned long pauseStartTime;
	bool isLong;
};

extern PauseState pauseState;
#endif /* STATEMACHINE_PAUSESTATE_H_ */
