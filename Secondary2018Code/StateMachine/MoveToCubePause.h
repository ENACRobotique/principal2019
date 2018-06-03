/*
 * MoveToCubePause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOCUBEPAUSE_H_
#define STATEMACHINE_MOVETOCUBEPAUSE_H_

#define TIME_US_CUBE 3000

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToCubePause : public AbstractState {
public:
	MoveToCubePause();
	virtual ~MoveToCubePause();

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
	USDistances usDistances;
};

extern MoveToCubePause moveToCubePause;

#endif /* STATEMACHINE_MOVETOCUBEPAUSE_H_ */
