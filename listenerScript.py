import serial
from checkUSBs import selectSerialPort
from IOFunctions import waitForResponse
from mainProgram_beta_rplidar import mainAlgorithm_beta_rpLidar
from mainProgram_beta_hokuyo import mainAlgorithm_beta_hokuyo

print("Initializing IO serial port...")

portDictionary = selectSerialPort()

## Initialize serial for IO communication
serial_IO = serial.Serial(portDictionary['1'], 115200,timeout = 5)


print("Waiting for start command...")
try:
    while True:

        str_return = waitForResponse(serial_IO)
        if str_return == 'Start':
##            mainAlgorithm_beta_rpLidar(serial_IO)
            mainAlgorithm_beta_hokuyo(serial_IO)
            str_return = ''

except KeyboardInterrupt:
    print("Listener stopped")

serial_IO.close()
