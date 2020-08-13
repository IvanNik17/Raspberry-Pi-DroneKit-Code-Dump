import serial
import numpy
import json

import sys


def waitForResponse(ser):

    data_str = ''
    if (ser.inWaiting()>0): #if incoming bytes are waiting to be read from the serial input buffer
##            data_str = serialSend.read(serialSend.inWaiting()).decode() #read the bytes and convert from binary array to ASCII
        num_read = ser.read(1).decode()
        if (num_read.isdigit() is True):
            
            data_str = ser.read(int(num_read)).decode()
        else:
            ser.flushInput()
        return data_str


def sendDataToGround(ser,arrayData):

        arrayData = numpy.round(arrayData,1)

        listData = arrayData.tolist()

        jsonPoints = json.dumps(listData)

        sendAll = jsonPoints + '\n'
        print(sys.getsizeof(sendAll))
        ser.write(sendAll)
