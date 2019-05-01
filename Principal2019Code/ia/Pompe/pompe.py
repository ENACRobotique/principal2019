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


def pumpCommand(activation):
    messagePump = com.MakePumpMessage()
    messagePump.update(activation)
    downCommunication = com.CommunicationSendWithAck(messagePump.serial_encode())
    downCommunication.start()
    downCommunication.join()


if __name__ == '__main__':
    
    
    
    
    
    messagePump = com.MakePumpMessage()
    
    messagePump.update(1)
    downCommunication = com.CommunicationSendWithAck(messagePump.serial_encode())
    
    
        
        
    '''
    while True:
        
        
        if time()-t0<10: #tries to turn the pump on for 5 sec and turn it off for 5 sec
            if time()-t0<5:
                messagePump.update(1)
            else :
                messagePump.update(0)
        else:
            t0 = time()
            '''
        
        #messagePump.update(1)
        #sleep(5)
    downCommunication.start()
    print("ici")
    downCommunication.join()
    print("")


