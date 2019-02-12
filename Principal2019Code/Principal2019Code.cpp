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

Metro asservTime = Metro((unsigned long)0.5*1000);


unsigned long t0;
float temps = 12000;
float vitesse_init = 200;
float vitesse = vitesse_init;
//The setup function is called once at startup of the sketch
void setup()
{
	Serial1.begin(115200);
	//Serial1.begin(115200);
	//while(!Serial);
	Odometry::init();
	MotorControl::init();
	//fsmSupervisor.init(&tiretteState);
	controlTime.reset();
	//navigatorTime.reset();
	t0 = millis();

}



// The loop function is called in an endless loop
void loop()
{
	//fsmSupervisor.update();
	if (Serial1.available()){
		char receive = Serial1.read();
		if (receive == 'r'){
			Odometry::reset();
			MotorControl::reset();
			t0=millis();
			Serial1.println("reset de la teensy");
			vitesse = vitesse_init;
		}
		if (receive == 's'){
			Serial1.println("Stop");
			vitesse = 0;
		}
	}
	if(controlTime.check()) {
		Odometry::update();
		MotorControl::update();
	}

	if ((millis() - t0)< temps){
		MotorControl::set_cons(vitesse,0);
	}
	else{
		MotorControl::set_cons(0,0);
	}



	if(asservTime.check()){
		Serial1.println(Odometry::get_pos_x());
	}
	//if(navigatorTime.check()) {
	//	navigator.update();
	//}

	//remoteController.update();
}
