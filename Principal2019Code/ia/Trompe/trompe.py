#--------Elie--------Test for the trunk---------------

import serial, bitstring
from time import time, sleep
import sys
path = "../Communication"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)


import communication as com
import params as p


def pumpCommand(activation):
    messagePump = com.MakePumpMessage()
    messagePump.update(activation)
    downCommunication = com.CommunicationSendWithAck(messagePump.serial_encode())
    downCommunication.start()
    downCommunication.join()
    
def trunkCommand(dyn_angle, dyn_speed):
    messageTrunk = com.MakeDynamixelMessage()
    messageTrunk.update(dyn_angle, dyn_speed)
    downCommunication = com.CommunicationSendWithAck(messageTrunk.serial_encode())
    downCommunication.start()
    downCommunication.join()


if __name__ == '__main__':

    
    
    t0 = time()
    activation = 1
    #print(activation) 
    #pumpCommand(activation)
    n = 0   
    while True:
        if time()-t0>5: #tries to turn the pump on for 3 sec and turn it off for 5 sec
            activation = (activation+1)%2
            n+=1
            pumpCommand(activation)
            angle = ((-1)**n)*90
            #print(angle)
            trunkCommand(angle, 1000)
            t0 = time()