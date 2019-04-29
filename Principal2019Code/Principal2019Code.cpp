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
#include "libraries/DynamixelSerial5/DynamixelSerial5.h"
Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD * 1000));
Metro navigatorTime = Metro(NAVIGATOR_TIME_PERIOD * 1000);

Metro asservTime = Metro((unsigned long)0.5*1000);


unsigned long t0;
float temps = 1*500; //temps en ms
float vitesse_init =80;//1.14;
float vitesse = vitesse_init;

unsigned long time_last_command_pump;


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

	pinMode(POMPE, OUTPUT);
	pinMode(13, OUTPUT);
	digitalWrite(POMPE, LOW);

	Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
	//Dynamixel.setEndless(DYNAMIXEL_ID,true);
	//Dynamixel.turn(DYNAMIXEL_ID,true,100);//MAX SPEED 1023
	//Dynamixel.setID(254,1);
	/*Dynamixel.setLEDAlarm(1, 1);
	delay(500);
	Dynamixel.setLEDAlarm(1, 0);
	delay(500);
	Dynamixel.setLEDAlarm(1, 1);
*/

}

int led_status = 0;

// The loop function is called in an endless loop
void loop()
{

	//fsmSupervisor.update();
	/*if (Serial.available()){
		char receive = Serial.read();
		if (receive == 'r'){
			//Odometry::reset();
			//MotorControl::reset();
			//t0=millis();
			//Serial.println("reset de la teensy");
			//vitesse = vitesse_init;
			Serial.println("pompe activee");
			digitalWrite(POMPE, HIGH);
		}
		if (receive == 's'){
			//Serial.println("Stop");
			//vitesse = 0;
			Serial.println("pompe desactivee");
			digitalWrite(POMPE, LOW);
		}
	}//fin serial available*/


	if(controlTime.check()) {
		Odometry::update();
		MotorControl::update();
	}

	if ((millis() - t0)> temps){
		//MotorControl::set_radius(300,-250);
		t0 = millis();
		//Dynamixel.setLEDAlarm(1, led_status);
		Dynamixel.setEndless(1,false);
		//Dynamixel.setAngleLimit(1, int CWLimit, int CCWLimit);
		Dynamixel.move(1,502+20*led_status);
		digitalWrite(13, led_status);
		//digitalWrite(POMPE, led_status);
		led_status ^= 1;
	}
//	else{
//		MotorControl::set_cons(0,0);
//	}


//
//	if(asservTime.check()){
//		Serial1.println(Odometry::get_pos_x());
//		Serial.println(Odometry::get_pos_theta());
//	}


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
				MotorControl::time_last_command = millis();
				float omega = get_omega_received(&downmessage);
				float speed = get_speed_received(&downmessage);
//				Serial.print(omega);
//				Serial.print("\t");
//				Serial.println(speed);
				//speed = 100;

				MotorControl::set_cons(speed, omega);
			}

			if(downmessage.id==PUMP){
				//Serial.print("pompe \n");
				int activation = get_pump_received(&downmessage);
				digitalWrite(POMPE, activation);
				time_last_command_pump = millis();
			}
		//MotorControl::set_cons(vitesse, 0);
		//MotorControl::set_cons(vitesse, 0);
		//Serial.print(Odometry::get_speed());
		//Serial.println(Odometry::get_omega());



		Message upmessage = make_pos_vel_message(Odometry::get_pos_x(), Odometry::get_pos_y(), Odometry::get_pos_theta(), Odometry::get_speed(), Odometry::get_omega());
		send_message(upmessage);

		}//fin if receive message

		if(millis()-time_last_command_pump > COMMAND_TIMEOUT){
			digitalWrite(POMPE, LOW);
		}

		//MotorControl::set_cons(0, 0);
		//analogWrite(POMPE, HIGH);


	//remoteController.update();

	}//fin navigatorTime.check()

}//fin loop
