import struct
import smbus
import time

class ArduinoCom:

    #
    # initialize dummy value of output from Pi (bytes only)
    #
    byteListDummyFromPi = [150, 220]

    #
    # initialize dummy values of inputs to Pi
    #
    dummyToPiFloats = [-3.1416, 6.2832]
    dummyToPiBytes = [2047, 50, 50]

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


    #
    # 1 = command byte to request first data block from Arduino
    # 8 = number of bytes (one 4-byte float + one 2-byte word)
    #
    def getFloatData(self,oldFloats):
        try:
            data_received = self.bus.read_i2c_block_data(self.address, 1, 8)
            newFloats = [self.bytes_2_float(data_received, 0)]
            newFloats.append(self.bytes_2_float(data_received, 1))
        except:
            print("error reading float data")
            newFloats = oldFloats;

        return newFloats

    #
    # 2 = command byte to request second data block from Arduino
    # 4 = number of bytes (one 2-byte word + two bytes)
    #
    def getByteData(self, oldBytes):
        try:
            data_received = self.bus.read_i2c_block_data(self.address, 2, 4)
            newBytes = [data_received[0] * 255 + data_received[1]]
            newBytes.append(data_received[2])
            newBytes.append(data_received[3])
        except:
            print("error reading byte data")
            newBytes = oldBytes

        return newBytes

    #
    # 255 = command byte to initiate writing to Arduino
    # (arbitrary--must be different than read)
    #
    def putByteList(self,byteList):
        try:
            self.bus.write_i2c_block_data(self.address, 255, byteList)
        except:
            print("error writing commands")
        return None

    #
    # crazy conversion of groups of 4 bytes in an array into a float
    # simple code assumes floats are at beginning of the array
    # "index" = float index, starting at 0
    #
    def bytes_2_float(self,data, index):
        bytes = data[4 * index:(index + 1) * 4]
        return struct.unpack('f', "".join(map(chr, bytes)))[0]

    ##########
    # main part of script starts here
    ##########

    #
    # now loop thru reading from and writing to Arduino
    #
    while True:
        time.sleep(0.1)
        dummyToPiFloats = getFloatData(dummyToPiFloats)
        dummyToPiBytes = getByteData(dummyToPiBytes)
        print(dummyToPiFloats, dummyToPiBytes)
        #
        #   send variable to Pi
        #
        putByteList(byteListDummyFromPi)
