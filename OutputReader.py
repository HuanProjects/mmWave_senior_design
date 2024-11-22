import struct

# constants
FRAME_HEADER_LENGTH = 40

# function to convert a 4 bytes data into a 32 bit unsigned integer
# @param data : array of 4 * 8 bit integer in the form of little endian
# @return     : 32 bit unsigned integer
def unit32_converter(data):
    return (data[0] +
            data[1] * 256 + 
            data[2] * 65536 +
            data[3] * 16777216)

# function to conver a 2 bytes data into a 16 bit unsigned integer
# @param data : array of 2 * 8 bit integer in the form of little endian
# @return     : 16 bit unsigned integer
def unit16_converter(data):
    return (data[0] +
            data[1] * 256)

# function to check if the given data match the magic number pattern
# @param data : array of 8 integers
# @return     : if this array match the magic number pattern
def checkMagicNumbers(data):
    return (data[0] == 2 and data[1] == 1 and
            data[2] == 4 and data[3] == 3 and
            data[4] == 6 and data[5] == 5 and
            data[6] == 8 and data[7] == 7)

# function to read the frame header
# @param data              : array of one byte size integers of radar data.
# @param debug             : flag to print out values for debuging
# the output is a tuple of the following values
# @return foundMagicNumber : boolean value of whether the magic pattern is found
# @return totalPacketLength: Total packet length including frame header in bytes
# @return numDetectedObj   : Number of detected objects.
# @return numTlvs          : Number of TLVs (Type Length Value) in the packet.
# @return subframeNumber   : Index of the subframe within the packet (default: 0).
def frameHeaderExtractor(data, debug = False):
    foundMagicNumber = False
    totalPacketLength = 0
    numDetectedObj = 0
    numTlvs = 0
    subframeNumber = 0
    if checkMagicNumbers(data[0:8]):
        foundMagicNumber = True
        totalPacketLength = unit32_converter(data[12:16])
        numDetectedObj = unit32_converter(data[28:32])
        numTlvs = unit32_converter(data[32:36])
        subframeNumber = unit32_converter(data[36:40])
    if debug:
        print("foundMagicNumber  = %d" % (foundMagicNumber))
        print("totalPacketLength = %d" % (totalPacketLength))
        print("numDetectedObj    = %s" % (numDetectedObj)) 
        print("numTlvs           = %d" % (numTlvs)) 
        print("subframeNumber    = %d" % (subframeNumber))
    return (foundMagicNumber, totalPacketLength, 
            numDetectedObj, numTlvs, subframeNumber)

# function to parse data from the retrieved radar output
# @param data                    : array of one-byte integers containing radar packet data.
# @param debug                   : flag to print out intermediate values for debugging (default: False).
# @return xPointsLocations       : list of x-coordinate positions of detected objects in meters.
# @return yPointsLocations       : list of y-coordinate positions of detected objects in meters.
# @return zPointsLocations       : list of z-coordinate positions of detected objects in meters.
# @return radialDopplerVelocities: list of radial velocities of detected objects in meters per second (m/s).
def outputDataParser(data, debug=False):
    xPointsLocations = []
    yPointsLocations = []
    zPointsLocations = []
    radialDopplerVelocities = []
    goodData = False
    # extract frame header
    foundMagicNumber, totalPacketLength, numDetectedObj, _, _ = frameHeaderExtractor(data, debug)

    # proceed only if magic number is found
    if foundMagicNumber:
        currentByte = FRAME_HEADER_LENGTH

        # extract TLV header
        tlvType = unit32_converter(data[currentByte:currentByte + 4])
        tlvLength = unit32_converter(data[currentByte + 4:currentByte + 8])
        currentByte += 8  # Skip past TLV header

        # validate TLV type and length
        if tlvType == 1 and tlvLength < totalPacketLength:
            if numDetectedObj == 0:
                return [], [], [], []

            # parse each detected object
            for obj in range(numDetectedObj):
                try:
                    x = struct.unpack('<f', bytes(data[currentByte:currentByte + 4]))[0]
                    y = struct.unpack('<f', bytes(data[currentByte + 4:currentByte + 8]))[0]
                    z = struct.unpack('<f', bytes(data[currentByte + 8:currentByte + 12]))[0]
                    v = struct.unpack('<f', bytes(data[currentByte + 12:currentByte + 16]))[0]
                except struct.error as e:
                    raise ValueError(f"Error parsing object {obj}: {e}")

                xPointsLocations.append(x)
                yPointsLocations.append(y)
                zPointsLocations.append(z)
                radialDopplerVelocities.append(v)

                currentByte += 16  # move to the next object

                if debug:
                    print(f"Object {obj}: x={x}, y={y}, z={z}, v={v}")
            goodData = True
    return (goodData, numDetectedObj, xPointsLocations, yPointsLocations, zPointsLocations, radialDopplerVelocities)
