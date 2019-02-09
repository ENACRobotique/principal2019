/*
 * MoveToCube2Pause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOCUBE2PAUSE_H_
#define STATEMACHINE_MOVETOCUBE2PAUSE_H_

#define TIME_US_CUBE 3000

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToCube2Pause : public AbstractState {
public:
	MoveToCube2Pause();
	virtual ~MoveToCube2Pause();

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

extern MoveToCube2Pause moveToCube2Pause;

#endif /* STATEMACHINE_MOVETOCUBE2PAUSE_H_ */
