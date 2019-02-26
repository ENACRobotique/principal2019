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
#include "communication.h"
Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD * 1000));
Metro navigatorTime = Metro(NAVIGATOR_TIME_PERIOD * 1000);

Metro asservTime = Metro((unsigned long)0.5*1000);


unsigned long t0;
float temps = 20*1000; //temps en ms
float vitesse_init = 1.14;
float vitesse = vitesse_init;


Message downmessage;

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

//	if ((millis() - t0)< temps){
//		MotorControl::set_radius(300,-250);
//	}
//	else{
//		MotorControl::set_cons(0,0);
//	}



	if(asservTime.check()){
		//Serial1.println(Odometry::get_pos_x());
		//Serial.println(Odometry::get_pos_theta());
	}
	if(navigatorTime.check()) {
	//	navigator.update();

		if(receive_message()==1){
			get_received_message(&downmessage);

			if(downmessage.id==POSITION){
				float x = get_x_received(&downmessage);
				float y = get_y_received(&downmessage);
				float theta = get_theta_received(&downmessage);
				Odometry::set_pos(x, y, theta);
			}

			if(downmessage.id==VELOCITY){
				float omega = get_omega_received(&downmessage);
				float speed = get_speed_received(&downmessage);
				MotorControl::set_cons(speed, omega);
			}
		}


		Message upmessage = make_pos_vel_message(Odometry::get_pos_x(), Odometry::get_pos_y(), Odometry::get_pos_theta(), Odometry::get_speed(), Odometry::get_omega());
		send_message(upmessage);
	}

	//remoteController.update();
}
