// Do not remove the include below
#include "Principal2019Code.h"

#include "remoteControl.h"
#include "Metro.h"
#include "lidar.h"
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
Metro receiveTime = Metro(RECEIVE_TIME_PERIOD *1000);

Metro asservTime = Metro((unsigned long)0.5*1000);


unsigned long t0;
float temps = 1*500; //temps en ms
float vitesse_init =100;//1.14;
float vitesse = vitesse_init;

unsigned long time_last_command_pump;


Message downmessage;
Lidar lidar;

Servo ServoLocker;
Servo ServoHolderLocker;
Servo DumboEar;

//The setup function is called once at startup of the sketch
void setup()
{

	Serial.begin(115200);
	Serial1.begin(115200);
	//while(!Serial){}
	Serial.println("Je suis au début du setup");

	/*while(true) {
		Serial1.println("plop");
		Serial.println("plop");
	}*/

	Odometry::init();
	MotorControl::init();
	//fsmSupervisor.init(&tiretteState);
	controlTime.reset();
	//navigatorTime.reset();
	t0 = millis();

	lidar = Lidar();

	pinMode(POMPE, OUTPUT);
	//pinMode(LED, OUTPUT);//LED intégrée à la Teensy
	digitalWrite(POMPE, LOW);

	pinMode(LID_PIN_IN1, OUTPUT);
	pinMode(LID_PIN_IN2, OUTPUT);
	pinMode(LID_PIN_IN3, OUTPUT);
	pinMode(LID_PIN_IN4, OUTPUT);
	pinMode(LID_PIN_IN5, OUTPUT);

	pinMode(LID_PIN_OUT1, INPUT_PULLUP);
	pinMode(LID_PIN_OUT2, INPUT_PULLUP);
	pinMode(LID_PIN_OUT3, INPUT_PULLUP);

	pinMode(PIN_TROMPE_POMPE,OUTPUT);
	digitalWrite(PIN_TROMPE_POMPE,TROMPE_POMPE_OFF);

	Dynamixel.begin(1000000, DYNAMIXEL_CONTROL);
	Dynamixel.setEndless(DYN_BROADCAST_ID,false);
	ServoLocker.attach(PIN_LOCK);
	ServoLocker.write(LOCK_LOCK);

	ServoHolderLocker.attach(PIN_HOLDER_LOCK);
	ServoHolderLocker.write(HOLDER_LOCK_LOCK);

	DumboEar.attach(PIN_DUMBO_EAR);
	DumboEar.write(DUMBO_EAR_OPEN);

	Dynamixel.moveSpeed(DYN_HOLDER_ID,DYN_HOLDER_DOWN, DYN_MAX_SPEED);
	Dynamixel.moveSpeed(DYN_TROMPE_ID,DYN_TROMPE_INSIDE, DYN_MAX_SPEED);


//	Dynamixel.setID(254,1);
//	Dynamixel.ledStatus(DYN_BROADCAST_ID, 1);
//	delay(500);
//	Dynamixel.ledStatus(DYN_BROADCAST_ID, 0);
//	delay(500);
//	Dynamixel.ledStatus(DYN_BROADCAST_ID, 1);

	//Dynamixel.moveSpeed(DYN_HOLDER_ID,DYN_HOLDER_DOWN, DYN_MAX_SPEED);

	//Get 3 Atoms
	/*delay(2000);
	for(int i=0;i<3;i++){
		DumboEar.write(DUMBO_EAR_CLOSE);
		delay(500);
		DumboEar.write(DUMBO_EAR_OPEN);
		delay(500);
	}*/

	//Move Up
	//Dynamixel.moveSpeed(DYN_HOLDER_ID,DYN_HOLDER_UP, DYN_MAX_SPEED);
	//delay(2500);

	//Open the lock
	//ServoHolderLocker.write(HOLDER_LOCK_OPEN);
	//delay(1000);


	//Go down
	//Dynamixel.moveSpeed(DYN_HOLDER_ID,DYN_HOLDER_DOWN, DYN_MAX_SPEED);
	//delay(2500);
	//ServoHolderLocker.write(HOLDER_LOCK_LOCK);

	//Handle Atom exit
	//for(int i=0;i<3;i++){
		//ServoLocker.write(LOCK_OPEN);
		//delay(1500);
		//ServoLocker.write(LOCK_LOCK);
		//delay(1500);
	//}
	Serial.println("Fin du setup");
}

//int led_status = 0;

// The loop function is called in an endless loop
void loop()
{
	/*if (Serial.available()){
		char receive = Serial.read();
		if (receive == 'r'){
			Odometry::reset();
			MotorControl::reset();
			t0=millis();
			Serial.println("reset de la teensy");
			//vitesse = vitesse_init;
			Serial.println("pompe activee");
			digitalWrite(POMPE, HIGH);
		}
		if (receive == 's'){
			Serial.println("Stop");
			vitesse = 0;
			Serial.println("pompe desactivee");
			digitalWrite(POMPE, LOW);
		}
	}//fin serial available
*/

	if(controlTime.check()) {
		Odometry::update();
		MotorControl::update();
	}

	/*if ((millis() - t0)> temps){
		//MotorControl::set_radius(300,-250);
		t0 = millis();
		//Dynamixel.setLEDAlarm(1, led_status);
		//Dynamixel.setEndless(1,false);
		//Dynamixel.setAngleLimit(1, int CWLimit, int CCWLimit);
		Dynamixel.moveSpeed(1,502+200*led_status, 100);
		digitalWrite(13, led_status);
		//digitalWrite(POMPE, led_status);
		led_status ^= 1;
	}*/
//	else{
//		MotorControl::set_cons(0,0);
//	}


//
//	if(asservTime.check()){
//		Serial1.println(Odometry::get_pos_x());
//		Serial.println(Odometry::get_pos_theta());
//	}


	if(receiveTime.check()){
		if(receive_message()==1){
			Serial.print("Message received !");
			get_received_message(&downmessage);
			Message upmessageack;

			if(downmessage.id==POSITION) {
				Serial.println("Message position");
				float x = get_x_received(&downmessage);
				float y = get_y_received(&downmessage);
				float theta = get_theta_received(&downmessage);
				Odometry::set_pos(x, y, theta);
			}

			if(downmessage.id==VELOCITY) {
				Serial.println("Message velocity");
				MotorControl::time_last_command = millis();
				float omega = get_omega_received(&downmessage);
				float speed = get_speed_received(&downmessage);
				Serial.print("Omega and speed received : ");
				Serial.print(omega);
				Serial.print("\t");
				Serial.println(speed);
				//speed = 100;
				MotorControl::set_cons(speed, omega);
			}

			if(downmessage.id==PUMP){
				Serial.println("Message pump");
				int activation = get_pump_received(&downmessage);
				digitalWrite(POMPE, activation);
				time_last_command_pump = millis();
				upmessageack = make_ack_message();
				send_message(upmessageack);
			}

			if(downmessage.id==DYN){
				Serial.println("dyn");
				int dyn_angle = get_dynAngle_received(&downmessage);
				int dyn_speed = get_dynSpeed_received(&downmessage);
				Message upmessageack = make_ack_message();
				//Dynamixel.moveSpeed(DYNAMIXEL_ID, dyn_angle, dyn_speed);

				upmessageack = make_ack_message();
				send_message(upmessageack);
			}

			if(downmessage.id==LID_DOWN){
				Serial.println("Lidar down");
				int pin1 = get_pin1_received(&downmessage);
				int pin2 = get_pin2_received(&downmessage);
				int pin3 = get_pin3_received(&downmessage);
				int pin4 = get_pin4_received(&downmessage);
				int pin5 = get_pin5_received(&downmessage);
				lidar.set_pin_in(pin1,pin2,pin3,pin4,pin5);
				lidar.comm_up();
			}

			if(downmessage.id==EAR_DOWN){
				Serial.println("Ear down");
				//activation est un booleen 0 ou 1
				int activation = get_ear_received(&downmessage);
				if(activation){
					DumboEar.write(DUMBO_EAR_OPEN);
				}
				else{
					DumboEar.write(DUMBO_EAR_CLOSE);
				}
				upmessageack = make_ack_message();
				send_message(upmessageack);
			}

			if(downmessage.id==LOCKER_DOWN){
				Serial.println("Locker down");
				//activation est un booleen 0 ou 1
				int activation = get_locker_received(&downmessage);
				if(activation){
					ServoLocker.write(LOCK_OPEN);
				}
				else{
					ServoLocker.write(LOCK_LOCK);
				}
				upmessageack = make_ack_message();
				send_message(upmessageack);
			}

			if(downmessage.id==HOLDER_DOWN){
				Serial.println("Holder down");
				//activation est un booleen 0 ou 1
				int activation = get_holder_received(&downmessage);
				if(activation){
					ServoHolderLocker.write(HOLDER_LOCK_OPEN);
				}
				else{
					ServoHolderLocker.write(HOLDER_LOCK_LOCK);
				}
				upmessageack = make_ack_message();
				send_message(upmessageack);
			}

			if(downmessage.id==DYNAMIC_HOLDER_DOWN){
				Serial.println("Dynamic holder down");
				//activation est un booleen 0 ou 1
				int activation = get_dynamic_holder_received(&downmessage);
				if(activation){
					Dynamixel.moveSpeed(DYN_HOLDER_ID,DYN_HOLDER_UP, DYN_MAX_SPEED);
				}
				else{
					Dynamixel.moveSpeed(DYN_HOLDER_ID,DYN_HOLDER_DOWN, DYN_MAX_SPEED);
				}
				upmessageack = make_ack_message();
				send_message(upmessageack);
			}

			if(downmessage.id == TIRETTE_DOWN){
				Serial.println("Demande de tirette");

				Message upmessage;
			}

			if(downmessage.id == COLOR_DOWN){
				Serial.println("Demande de couleur");

				Message upmessage;
			}


		}
	}


	if(navigatorTime.check()) {
		Serial.println("navigatorTime !");




		Serial.print("Message sent : omega = ");
		Serial.println(Odometry::get_omega());

		Message upmessage = make_pos_vel_message(Odometry::get_pos_x(), Odometry::get_pos_y(), Odometry::get_pos_theta(), Odometry::get_speed(), Odometry::get_omega());
		send_message(upmessage);

		//Message USupmessage = make_US_message();
		//send_message(USupmessage);

		if(lidar.comm_down()){
			upmessage = make_lidar_message((uint8_t)lidar.get_zone1(),(uint8_t)lidar.get_zone2(),(uint8_t)lidar.get_zone3());
			send_message(upmessage);
		}
		/*Serial.print("Lidar zone 1 :");
		Serial.print(lidar.get_zone1());
		Serial.print("\t");
		Serial.print("Lidar zone 2 :");
		Serial.print(lidar.get_zone2());
		Serial.print("\t");
		Serial.print("Lidar zone 3 :");
		Serial.println(lidar.get_zone3());*/

	}//fin navigatorTime.check()

}//fin loop
