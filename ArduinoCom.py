import struct
import smbus
import time
from helperCodes import wrapAngle
import traceback

class ArduinoCom:
    """
    This class serves as the interface between the Raspberry Pi and Arduino. It permits communication between the two devices
    over an I2C communication where the Ardunio is the slave device. Take care that the dataCommands list is the same between
    the Arduino and Pi
    """
    def __init__(self):
        #
        # smbus implements i2c on the RPi
        #
        self.bus = smbus.SMBus(1)

        #
        # this is the Slave address of the Arduino
        #
        self.address = 0x04

        #dictionary mapping data to their values
        self.data = {'psiD': 0, 'speedD': 50, 'gpsLat': 2, 'gpsLon': 3, 'gpsV': 4, 'gpsPsi': 5}

        #dictionary mapping data to send to arduino with respective commands
        #IMPORTANT IF YOU CHANGE THESE YOU MUST CHANGE THEM IN THE ARDUINO AS WELL
        #TODO: SYNC PI AND ARDUINO COMMANDS
        self.dataCommands = {'psiD': 0, 'speedD': 1, 'gpsLat': 2, 'gpsLon': 3, 'gpsV': 4, 'gpsPsi': 5}


    # region Setters and Getters
    def getpsiD(self):
        return self.data['psiD']

    def getSpeed(self):
        return self.data['speed']

    def setpsiD(self, heading):
        heading = wrapAngle(heading)
        self.data['psiD'] = heading

    def setSpeed(self, speed):
        if speed >= 0:
            self.data['speed'] = speed

    def setData(self, dataName, dataVal):
        """
        Set Data in the Pi by name
        :param dataName:
        :param dataVal:
        :return:
        """
        if dataName in self.data.keys():
            self.data[dataName] = dataVal

    def getDataByName(self, dataName):
        """
        Get data as currently stored in the Pi
        :param dataName: Name of data you wish to access
        :return: data
        """
        return self.data[dataName]
    # endregion

    def sendData(self, dataName):
        """
        Send data with a name to the Arduino
        :param dataName: name of data to send
        :return: None
        """
        if dataName in self.dataCommands.keys():
            dataStr = str(self.data[dataName])
            print dataStr
            dataArr = []
            for c in dataStr: #parse the data string into an array of chars
                dataArr.append(ord(c))
            try:
                self.bus.write_i2c_block_data(self.address, self.dataCommands[dataName], dataArr)
                print "sent " + str(dataArr)
            except Exception as e:
                print 'failed to write to Arduino'
                traceback.print_exc()
                print

    def getData(self, dataName):
        """
        Get data from Arduino Mega
        :param dataName: name of data variable that you wish to retrieve
        :return: value of requested data as stored in Arduino
        """
        try:
            data_received = self.bus.read_i2c_block_data(self.address, self.dataCommands[dataName],4)
            val = self.bytes_2_float(data_received, 0)
            self.data[dataName] = val
            print(dataName + " is: " + str(self.data[dataName]))
            print "com completed"
        except Exception as e:
            print 'failed to receive from Arduino'
            print e
            print

    def bytes_2_float(self, data, index):
        """
        convert a byte array to a float
        :param data: byte array
        :param index: index offset that the data starts at
        :return: a float from a byte array
        """
        bytes = data[4*index:(index+1)*4]
        return struct.unpack('f', "".join(map(chr,bytes)))[0]
