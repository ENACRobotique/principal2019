#--------Elie--------Test for the pump---------------


import serial, bitstring

from time import time, sleep


import sys
path = "../Communication"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)


import communication as com

import params as p

if __name__ == '__main__':
    
    upCommunication = com.CommunicationReceived()
    downCommunication = com.CommunicationSend()
    
    positionReceived = com.PositionReceived()
    
    messagePump = com.MakePumpMessage()
    t0 = time()
    while True:
        
        if time()-t0<10: #tries to turn the pump on for 5 sec and turn it off for 5 sec
            if time()-t0<5:
                messagePump.update(1)
            else :
                messagePump.update(0)
        else:
            t0 = time()
            
        
        #messagePump.update(1)
        #sleep(5)
        downCommunication.send_message(messagePump.serial_encode())
        print("ici")


