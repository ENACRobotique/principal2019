import serial, bitstring
import numpy as np

import communication as com
import robot

import sys
path = "../PurePursuit"
sys.path.append(path)
path = "../PurePursuit"
sys.path.append(path)
import pure_pursuit as pp
from path import Path, Point

if __name__ == '__main__':
    robot = robot.RobotPosition(10, 10, 0, 0, 0)
    print(robot)

    upCommunication = com.CommunicationReceived()
    downCommunication = com.CommunicationSend()

    positionReceived = com.PositionReceived()
    messagePosition = com.MakePositionMessage()
    messageVelocity = com.MakeVelocityMessage()
    # messageVelocity.update(100, -1)
    # downCommunication.send_message(messageVelocity.serial_encode())

    messagePosition.update(robot.x, robot.y, robot.theta)
    downCommunication.send_message(messagePosition.serial_encode())

    Y = np.linspace(0, 1500, 1000)
    X = np.linspace(0, 0, 1000)
    path = Path(np.array([Point(X[i], Y[i]) for i in range(1000)]))

    tracking = pp.PurePursuit(path)
    look_ahead_distance = 200
    _speed_tracking = 100

    while True:

        receive_message = upCommunication.receive_message()

        if receive_message is not None:
            id_message, payload = receive_message

            if id_message == com.Type.POS_VEL.value:
                positionReceived.serial_decode(payload)
                robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
                #print(robot)

            omega = tracking.compute(robot, look_ahead_distance)
            print("omega_cons = ", omega)
            messageVelocity.update(_speed_tracking, omega)
            downCommunication.send_message(messageVelocity.serial_encode())