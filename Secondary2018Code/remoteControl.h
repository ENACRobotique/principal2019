/*
 * remoteControl.h
 *
 *  Created on: 18 sept. 2018
 *      Author: Maxime Le Du
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

class RemoteController{
public:
		RemoteController();
		void set_cons();
		void update();
		bool parse_data();

private:
		char data[7];
		int index;
};

extern RemoteController remoteController;

#endif /* REMOTECONTROL_H_ */
