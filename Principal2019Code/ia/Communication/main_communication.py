import serial, bitstring

import communication as com
import robot

if __name__ == '__main__':
	robot = robot.RobotPosition()

	upCommunication = com.CommunicationReceived()
	downCommunication = com.CommunicationSend()

	positionReceived = com.PositionReceived()

	messageVelocity = com.MakeVelocityMessage()
	messageVelocity.update(100,0)
	downCommunication.send_message(messageVelocity.serial_encode())
	while True:

		receive_message = upCommunication.receive_message()
		if receive_message is not None:
			id_message, payload = receive_message
			if id_message == com.Type.POS_VEL.value:
				positionReceived.serial_decode(payload)
				robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
			print(robot)




