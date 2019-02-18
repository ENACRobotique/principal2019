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
float temps = 20*1000; //temps en ms
float vitesse_init = 1.14;
float vitesse = vitesse_init;
//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200);
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
	if (Serial.available()){
		char receive = Serial.read();
		if (receive == 'r'){
			Odometry::reset();
			MotorControl::reset();
			t0=millis();
			Serial.println("reset de la teensy");
			vitesse = vitesse_init;
		}
		if (receive == 's'){
			Serial.println("Stop");
			vitesse = 0;
		}
	}
	if(controlTime.check()) {
		Odometry::update();
		MotorControl::update();
	}

	if ((millis() - t0)< temps){
		MotorControl::set_radius(300,-250);
	}
	else{
		MotorControl::set_cons(0,0);
	}



	if(asservTime.check()){
		//Serial1.println(Odometry::get_pos_x());
		Serial.println(Odometry::get_pos_theta());
	}
	if(navigatorTime.check()) {
	//	navigator.update();
		int x = Odometry::get_pos_x();
		int y = Odometry::get_pos_y();
		int theta_com = (Odometry::get_pos_theta() + PI) * 1000;
		uint8_t buffer[9];

		buffer[0] = 0xFF;
		buffer[1] = 0x01;
		buffer[2] = (x & 0xFF00)>>8;
		buffer[3] = x & 0x00FF;
		buffer[4] = (y & 0xFF00)>>8;
		buffer[5] = y & 0xFF;
		buffer[6] = (theta_com & 0xFF00)>>8;
		buffer[7] = theta_com & 0xFF;
		buffer[8] = '\n';

		Serial1.write(buffer, 9);

	}

	//remoteController.update();
}
