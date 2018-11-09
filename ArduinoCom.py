import struct
import smbus
import time
from helperCodes import wrapAngle
import traceback

class ArduinoCom:
    def __init__(self):
        #
        # smbus implements i2c on the RPi
        #
        self.bus = smbus.SMBus(1)

        #
        # this is the Slave address of the Arduino
        #
        self.address = 0x04
        # adapted from code at https://gist.github.com/gileri/5a9285d6a1cfde142260
        #
        # check out this information about the "command" parameter (second argument in block read / write)
        # https://raspberrypi.stackexchange.com/questions/8469/meaning-of-cmd-param-in-write-i2c-block-data
        #

        #dictionary mapping data to their values
        self.data = {'psiD': 0, 'speedD': 50, 'gpsLat': 2, 'gpsLon': 3, 'gpsV': 4, 'gpsPsi': 5}

        #dictionary mapping data to send to arduino with respective commands
        #IMPORTANT DO NOT CHANGE
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
        if dataName in self.data.keys():
            self.data[dataName] = dataVal

    def getDataByName(self, dataName):
        return self.data[dataName]
    # endregion

    def sendData(self, dataName):
        if dataName in self.dataCommands.keys():
            dataStr = str(self.data[dataName])
            dataArr = []
            for c in dataStr: #parse the data string into an array of chars
                dataArr.append(ord(c))
            try:
                self.bus.write_i2c_block_data(self.address, self.dataCommands[dataName], dataArr)
                #print "sent " + str(dataArr)
            except Exception as e:
                pass

    def getData(self, dataName):
        try:
            data_received = self.bus.read_i2c_block_data(self.address, self.dataCommands[dataName],4)
            val = self.bytes_2_float(data_received, 0)
            self.data[dataName] = val
            #print(dataName + " is: " + str(self.data[dataName]))
            #print "com completed\n"
        except Exception as e:
            pass

    def bytes_2_float(self, data, index):
        bytes = data[4*index:(index+1)*4]
        return struct.unpack('f', "".join(map(chr,bytes)))[0]

