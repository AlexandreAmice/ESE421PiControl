import struct
import smbus
import time

class I2C:
    dataFile = 'data.txt'
    heading = 255

    #
    # 2 = command byte to request second data block from Arduino
    # 4 = number of bytes (one 2-byte word + two bytes)
    #
    def getByteData(oldBytes):
        try:
            data_received = bus.read_i2c_block_data(address, 2, 4)
            newBytes = [data_received[0]*255 + data_received[1]]
            newBytes.append(data_received[2])
            newBytes.append(data_received[3])
            string = parse_serial(data_received)
            with open(dataFile, 'a') as f:
                f.write(string)
        
        
        except Exception as e:
            print e
            print("error reading byte data")
            newBytes = oldBytes;

        return newBytes


    #
    # crazy conversion of groups of 4 bytes in an array into a float
    # simple code assumes floats are at beginning of the array
    # "index" = float index, starting at 0
    #
    def bytes_2_float(data, index=0):
        bytes = data[4*index:(index+1)*4]
        return struct.unpack('f', "".join(map(chr, bytes)))[0]

    #
    # print parsed data into serial monitor
    #
    def parse_serial(data):
        string = "GPS Latitude: " + data[0] + ' GPS Longitude: ' + data[1] + ' GPS Velocity: ' + data[2] + ' GPS Psi: ' + data[3] + '\n' 
        return string
        

    #
    # 255 = command byte to initiate writing to Arduino
    # (arbitrary--must be different than read)
    #
    def putByteList(command, byteList):
        try:
            bus.write_i2c_block_data(address, command, byteList)
        except:
            pass
            #print("error writing commands")
        return None


    ##########
    # main part of script starts here
    ##########

    #
    # smbus implements i2c on the RPi
    #
    bus = smbus.SMBus(1)

    #
    # this is the Slave address of the Arduino
    #
    address = 0x04

    #
    # initialize dummy value of output from Pi (bytes only)
    #
    byteListDummyFromPi = [150, 220]

    #
    # initialize dummy values of inputs to Pi
    #
    dummyToPiFloats = [0, 6.2832]
    dummyToPiBytes = [2047, 50, 50]

    #
    # now loop thru reading from and writing to Arduino
    #
    while True:
        time.sleep(0.1)
        dummyToPiBytes = getByteData(dummyToPiBytes)
        #print(dummyToPiFloats, dummyToPiBytes)
    #
    #   send variable to Pi
    #
        putByteList(heading, byteListDummyFromPi)


