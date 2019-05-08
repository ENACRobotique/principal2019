import sys
path = "../Communication"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)

from time import time

from Communication import communication as com

from Communication import robot
import params as p

from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
from PurePursuit.path_manager import Path, Point

if __name__ == '__main__':
    
    robot = robot.RobotPosition(0, 0, 0, 0, 0)
    upCommunication = com.CommunicationReceived()
    downCommunication = com.CommunicationSend()
    positionReceived = com.PositionReceived()
    messagePosition = com.MakePositionMessage()
    messageVelocity = com.MakeVelocityMessage()
    
    messagePosition.update(robot.x, robot.y, robot.theta)
    downCommunication.send_message(messagePosition.serial_encode())
    
    Nbpoints = 10000
    
    path = pf.lemniscate(Nbpoints, 1000, 600)
    path.compute_speed(0.2,0.85,200)
    
    tracking = pp.PurePursuit(path)
    
    t0 = time()
    
    print("la")
    
    while True:
        
        receive_message = upCommunication.receive_message()
        if receive_message is not None:
            id_message, payload = receive_message
            if id_message == com.Type.POS_VEL.value:
                positionReceived.serial_decode(payload)
                robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
                print("({}, {}, {}, {}, {})".format(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega))
                
            omega, speed = tracking.compute(robot, False)
            print("(consigne : {}, {})".format(omega, speed))
            messageVelocity.update(speed, omega)
            downCommunication.send_message(messageVelocity.serial_encode())