/*
 * MoveToWaterTrans.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_MOVETOWATERTRANS_H_
#define STATEMACHINE_MOVETOWATERTRANS_H_

#define TIME_US 2000

#include "AbstractState.h"
#include "../lib/USManager.h"

class MoveToWaterTrans : public AbstractState {
public:
	MoveToWaterTrans();
	virtual ~MoveToWaterTrans();

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

extern MoveToWaterTrans moveToWaterTrans;

#endif /* STATEMACHINE_MOVETOWATERTRANS_H_ */
