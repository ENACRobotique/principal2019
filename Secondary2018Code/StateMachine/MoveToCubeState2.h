/*
 * MoveToCubeState2.h
 *
 *  Created on: 5 mai 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOCUBESTATE2_H_
#define STATEMACHINE_MOVETOCUBESTATE2_H_

#define US_TIME_CUBE2 2000

#include "AbstractState.h"
#include "../lib/USManager.h"

#define POS_X_CUBE_GREEN
#define POS_X_CUBE_ORANGE
#define POS_Y_CUBE_GREEN
#define POS_Y_CUBE_ORANGE

class MoveToCubeState2 : public AbstractState {
public:
	MoveToCubeState2();
	virtual ~MoveToCubeState2();

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
	unsigned long us_time;
	USDistances usDistances;
};

extern MoveToCubeState2 moveToCubeState2;

#endif /* STATEMACHINE_MOVETOCUBESTATE2_H_ */
