import serial
import serial.tools.list_ports



def selectSerialPort():
    neededPorts={}
    for port in serial.tools.list_ports.comports():
        if port.description == 'FT231X USB UART':
            neededPorts['1'] = port.device

        if port.description == 'CP2102 USB to UART Bridge Controller':
            neededPorts['2'] = port.device

        if port.description == 'USB2.0-Serial':
            neededPorts['3'] = port.device

        if port.description == 'FT230X Basic UART':
            neededPorts['4'] = port.device

        if port.description == 'URG-Series USB Driver':
            neededPorts['5'] = port.device

    return neededPorts


if __name__ == '__main__':
    for port in serial.tools.list_ports.comports():
        print(port.description)
    
