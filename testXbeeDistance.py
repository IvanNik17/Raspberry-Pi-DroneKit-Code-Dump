import serial
import numpy as np
from checkUSBs import selectSerialPort

import json

from IOFunctions import waitForResponse,sendDataToGround

portDictionary = selectSerialPort()

## Initialize serial for IO communication 
serial_IO = serial.Serial(portDictionary['1'], 115200,timeout=1)

startComm = False
counter = 0

print("alg go")
try:
    while True:
        str_return = ''
        str_return = waitForResponse(serial_IO)

##        if str_return is not '':
##            print(str_return)

        if str_return == 'Start':
            startComm = True
            print("start")

        if str_return == 'Stop':
            print("stop")
            break

        if startComm is False:
            continue

        if str_return == 'test':
            print("received")
            message = 'test_return' + str(counter)
            jsonResponce = json.dumps(message)
            jsonResponce = jsonResponce + '\n'

            serial_IO.write(jsonResponce)

            counter+=1


except KeyboardInterrupt:
    print("Force Stop")

serial_IO.close()

        



##counter = 0
##try:
##    while True:
##        sendData = 'Send' + str(counter) + '\n'
##        counter +=1
##        serial_IO.write(sendData)
##        print(sendData)
##        
##except KeyboardInterrupt:
##    print("Force stop")



##try:
##    while 1:
##        str_return = ''
##        str_return = waitForResponse(serial_IO)
##
##        if str_return == 'Send':
####            print("Received")
##            
####            sendAll = '7Receive'
##            arraySend = np.random.rand(100,2)
##            print(len(arraySend))
##            sendDataToGround(serial_IO,arraySend)
####            serial_IO.write(sendAll)
##        elif str_return == 'Stop':
##            print("Exit loop")
##            break
##
##        
##except KeyboardInterrupt:
##    print("Force stop")


serial_IO.close()
