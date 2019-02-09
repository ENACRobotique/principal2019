// Do not remove the include below
#include "Secondary2018Code.h"
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

//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200);
	//Serial1.begin(115200);
	Odometry::init();
	MotorControl::init();
	fsmSupervisor.init(&tiretteState);
	controlTime.reset();
	navigatorTime.reset();

}

// The loop function is called in an endless loop
void loop()
{
	//fsmSupervisor.update();

	if(controlTime.check()) {
		Odometry::update();
		MotorControl::update();
	}

	//if(navigatorTime.check()) {
	//	navigator.update();
	//}

	remoteController.update();
}
