import serial
from checkUSBs import selectSerialPort
import timeit

def getDataFromIMU(ser, oldRead):

    returnedList = oldRead
    try:
        if (ser.inWaiting()>0):
            
            try:        
        ##        if ser.inWaiting() >100:
        ##            ser.flushInput()
        ##
        ##        if ser.inWaiting() >20 and ser.inWaiting() <30:
        ##            
        ##            ser.flushInput()

                arduinoLine = ser.readline().decode().strip()

                arduinoLine_split = arduinoLine.split('|')
                if len(arduinoLine_split) == 7:
                    try:
                        arduinoLine_floats = [round(float(x),1) for x in arduinoLine.split('|')]
                        returnedList = arduinoLine_floats
                        ser.flushInput()
                    except ValueError:
                        pass
            except UnicodeDecodeError:
                pass
    except IOError:
        print("IO error")

    return returnedList


def calibrateIMU(ser):
    arduinoInitial = numpy.zeros(8)
    while True:
        arduinoOutput = getDataFromIMU(ser,arduinoInitial)
        arduinoInitial = arduinoOutput
        print("Calibration: Sys= " + str(arduinoOutput[0]) + " Gyro= " + str(arduinoOutput[1]) + " Accel= " + str(arduinoOutput[2]) + " Mag= " + str(arduinoOutput[3]))
        if int(arduinoOutput[0]) is 3 and int(arduinoOutput[1]) is 3 and int(arduinoOutput[2]) is 3 and int(arduinoOutput[3]) is 3:
            return 1

if __name__ == '__main__':
    portDictionary = selectSerialPort()
    serial_IMU = serial.Serial(portDictionary['3'],57600)
    oldData = []
    try:
        
        while True:
            start_time = timeit.default_timer()
            
            listData = getDataFromIMU(serial_IMU, oldData)
            oldData = listData
##            print(listData)
            elapsed = timeit.default_timer() - start_time
            print(elapsed)

    except KeyboardInterrupt:
        print("End")
