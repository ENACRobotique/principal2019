import communication as com
import serial, bitstring

if __name__ == '__main__':
	robot = com.RobotPosition()
	upCommunication = com.Communication()
	positionReceived = com.PositionReceived()
	while True:
		receive_message = upCommunication.receive_message()
		if receive_message is not None:
			id_message, payload = receive_message
			if id_message == com.Type.POS_VEL.value:
				positionReceived.serial_decode(payload)
				robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
			print(robot)




