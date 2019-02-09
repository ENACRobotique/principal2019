// Do not remove the include below
#include "Principal2019Code.h"

#include "remoteControl.h"
#include "Metro.h"
#include "params.h"
#include "odometry.h"
#include "motorControl.h"
#include "Navigator.h"
#include "StateMachine/FSMSupervisor.h"
#include "StateMachine/TiretteState.h"
Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD * 1000));
Metro navigatorTime = Metro(NAVIGATOR_TIME_PERIOD * 1000);

Metro asservTime = Metro((unsigned long)2*1000);

int i=0;
float tab[4] = {400, 0, -400, 0};

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200);
	//Serial1.begin(115200);
	//while(!Serial);
	Odometry::init();
	MotorControl::init();
	//fsmSupervisor.init(&tiretteState);
	controlTime.reset();
	//navigatorTime.reset();

}

// The loop function is called in an endless loop
void loop()
{
	//fsmSupervisor.update();

	if(controlTime.check()) {
		Odometry::update();
		MotorControl::update();
	}

	if(asservTime.check()){
		MotorControl::set_cons(tab[i], 0);
		i=(i+1)%4;
	}
	//if(navigatorTime.check()) {
	//	navigator.update();
	//}

	//remoteController.update();
}
