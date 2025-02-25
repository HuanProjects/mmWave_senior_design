import struct
import math
import pandas as pd
import numpy as np
# Frame Header Length
FRAME_HEADER_LENGTH = 40
# TLV Header Length
TLV_HEADER_LENGTH = 8
# Magic words
MAGIC_WORD = [2, 1, 4, 3, 6, 5, 8, 7]

# Function to convert a 4-byte little-endian array to a 32-bit unsigned integer
def uint32_converter(data):
    """
    Converts a 4-byte little-endian array into a 32-bit unsigned integer.

    :param data: List of 4 integers representing the bytes in little-endian format.
    :return: 32-bit unsigned integer.
    """
    return struct.unpack('<I', bytes(data))[0]

# Function to convert a 2-byte little-endian array to a signed 16-bit integer
def int16_converter(data):
    """
    Converts a 2-byte little-endian array into a signed 16-bit integer.

    :param data: List of 2 integers representing the bytes in little-endian format.
    :return: Signed 16-bit integer.
    """
    value = data[0] + (data[1] * 256)
    return value if value < 32768 else value - 65536

# function to convert a one byte unsigned integer to a one byte signed integer
def int8_converter(value):
    """
    Converts an 8-bit unsigned integer to a signed integer.
    
    :param value: Integer value between 0 and 255.
    :return: Signed integer between -128 and 127.
    """
    return value - 256 if value > 127 else value

# Function to verify if the given data matches the predefined magic number sequence
def checkMagicNumbers(data):
    """
    Checks if the first 8 bytes of the data match the magic number pattern.

    :param data: List of integers representing the packet data.
    :return: True if the first 8 bytes match the magic number sequence, otherwise False.
    """
    return (data[0] == 2 and data[1] == 1 and
            data[2] == 4 and data[3] == 3 and
            data[4] == 6 and data[5] == 5 and
            data[6] == 8 and data[7] == 7)

# Function to extract and parse the frame header from the radar packet
def frameHeaderExtractor(data, debug=False):
    """
    Extracts the frame header from radar packet data.

    :param data: List of integers representing radar packet data.
    :param debug: Boolean flag to enable debug logging.
    :return: Tuple containing:
        - foundMagicNumber: Boolean indicating if magic number is found.
        - totalPacketLength: Total packet length in bytes.
        - numDetectedObj: Number of detected objects.
        - numTlvs: Number of TLVs in the packet.
        - subframeNumber: Index of the subframe within the packet.
    """
    foundMagicNumber = False
    totalPacketLength = 0
    numDetectedObj = 0
    numTlvs = 0
    subframeNumber = 0
    if checkMagicNumbers(data[0:8]):
        foundMagicNumber = True
        totalPacketLength = uint32_converter(data[12:16])
        numDetectedObj = uint32_converter(data[28:32])
        numTlvs = uint32_converter(data[32:36])
        subframeNumber = uint32_converter(data[36:40])
    if debug:
        print("foundMagicNumber  = %d" % (foundMagicNumber))
        print("totalPacketLength = %d" % (totalPacketLength))
        print("numDetectedObj    = %s" % (numDetectedObj)) 
        print("numTlvs           = %d" % (numTlvs)) 
        print("subframeNumber    = %d" % (subframeNumber))
    return (foundMagicNumber, totalPacketLength, 
            numDetectedObj, numTlvs, subframeNumber)

# Function to parse point cloud data from a TLV
def pointCloud(data, packageLength):
    """
    Parses point cloud data from a TLV.

    :param data: List of integers representing the TLV data.
    :param packageLength: Length of the package in bytes.
    :return: Pandas DataFrame containing point cloud data with columns:
        - x: X-coordinate
        - y: Y-coordinate
        - z: Z-coordinate
        - doppler: Doppler velocity
        - snr: Signal-to-noise ratio
    """    
    xPointsLocation = []
    yPointsLocation = []
    zPointsLocation = []
    dopplerVelocities = []
    pointsSNR = []
    
    # Calculate number of points
    numOfPoints = int((packageLength - 20) / 8)

    # Unpack scaling units
    elevationUnit = struct.unpack('<f', bytes(data[0:4]))[0]
    azimuthUnit = struct.unpack('<f', bytes(data[4:8]))[0]
    dopplerUnit = struct.unpack('<f', bytes(data[8:12]))[0]
    rangeUnit = struct.unpack('<f', bytes(data[12:16]))[0]
    snrUnit = struct.unpack('<f', bytes(data[16:20]))[0]

    currentByte = 20  # Start reading points after the scaling units
    for _ in range(numOfPoints):
        elevation = int8_converter(data[currentByte]) * elevationUnit
        azimuth = int8_converter(data[currentByte + 1]) * azimuthUnit
        doppler = int16_converter(data[currentByte + 2: currentByte + 4]) * dopplerUnit
        rangeVal = int16_converter(data[currentByte + 4: currentByte + 6]) * rangeUnit
        snr = int16_converter(data[currentByte + 6: currentByte + 8]) * snrUnit
        
        # Convert spherical to Cartesian coordinates
        x = rangeVal * math.cos(elevation) * math.sin(azimuth)
        y = rangeVal * math.cos(elevation) * math.cos(azimuth)
        z = rangeVal * math.sin(elevation)

        # Save the points
        xPointsLocation.append(x)
        yPointsLocation.append(y)
        zPointsLocation.append(z)
        dopplerVelocities.append(doppler)
        pointsSNR.append(snr)
        
        currentByte += 8  # Move to the next point
    
    pointCloud = pd.DataFrame({
        'x': xPointsLocation,
        'y': yPointsLocation,
        'z': zPointsLocation,
        'doppler' : dopplerVelocities,
        'snr': pointsSNR
    })
    return pointCloud

# Function to parse target list data from a TLV
def targetList(data, packageLength):
    """
    Parses target list data from a TLV.

    :param data: List of integers representing the TLV data.
    :param packageLength: Length of the package in bytes.
    :return: Pandas DataFrame containing target data with columns:
        - tid: Target ID
        - posX, posY, posZ: Position coordinates
        - velX, velY, velZ: Velocity components
        - accX, accY, accZ: Acceleration components
        - g: Confidence metric
        - confidenceLevel: Confidence level
    """    
    num_target = packageLength // 112
    targetList = []
    currentByte = 0
    for _ in range(num_target):
        tid = uint32_converter(data[currentByte: currentByte + 4])
        posX = struct.unpack('<f', bytes(data[currentByte + 4: currentByte + 8]))[0]
        posY = struct.unpack('<f', bytes(data[currentByte + 8: currentByte + 12]))[0]
        posZ = struct.unpack('<f', bytes(data[currentByte + 12: currentByte + 16]))[0]
        velX = struct.unpack('<f', bytes(data[currentByte + 16: currentByte + 20]))[0]
        velY = struct.unpack('<f', bytes(data[currentByte + 20: currentByte + 24]))[0]
        velZ = struct.unpack('<f', bytes(data[currentByte + 24: currentByte + 28]))[0]
        accX = struct.unpack('<f', bytes(data[currentByte + 28: currentByte + 32]))[0]
        accY = struct.unpack('<f', bytes(data[currentByte + 32: currentByte + 36]))[0]
        accZ = struct.unpack('<f', bytes(data[currentByte + 36: currentByte + 40]))[0]
        currentByte += 40

        # skip past the error covariance matrix
        currentByte += 16*4
        g = struct.unpack('<f', bytes(data[currentByte: currentByte + 4]))[0]
        confidenceLevel = struct.unpack('<f', bytes(data[currentByte + 4: currentByte + 8]))[0]
        currentByte += 8
        
        target = {
            'tid': tid, 
            'posX': posX,
            'posY': posY, 
            'posZ': posZ, 
            'velX': velX, 
            'velY': velY,
            'velZ': velZ, 
            'accX': accX,
            'accY': accY,
            'accZ': accZ,
            'g': g,
            'confidenceLevel': confidenceLevel
        }
        targetList.append(target)
    return pd.DataFrame(targetList)
    
def record_frames(data_port, magic_word=bytearray(MAGIC_WORD)):
    """
    A generator that yields frames as they are recorded.

    Parameters:
    - data_port: The serial port to read data from.
    - magic_word: The sequence used to identify the start of a frame.

    Yields:
    - frame: The full frame data as a bytearray.
    - numTlvs: The number of TLVs in the frame.
    """
    buffer = bytearray()

    while True:
        # Read available bytes into the buffer
        new_data = data_port.read(data_port.in_waiting or 1)
        buffer.extend(new_data)

        while True:
            # Find the magic word in the buffer
            magic_index = buffer.find(magic_word)
            if magic_index == -1:
                # Magic word not found, keep only the last 8 bytes for overlap checking
                buffer = buffer[-len(magic_word):]
                break

            # Remove data before the magic word
            buffer = buffer[magic_index:]

            # Ensure the buffer contains at least the frame header
            if len(buffer) < FRAME_HEADER_LENGTH:
                break

            # Parse the frame header
            byte_array = np.frombuffer(buffer[:FRAME_HEADER_LENGTH], dtype='uint8')
            found_magic_number, total_packet_length, num_detected_obj, num_tlvs, subframe_number = frameHeaderExtractor(byte_array)

            if not found_magic_number:
                # Discard invalid header
                buffer = buffer[8:]  # Move past the first 8 bytes
                continue

            # Check if the buffer contains the full frame
            if len(buffer) < total_packet_length:
                break

            # Extract the full frame
            frame = buffer[:total_packet_length]
            yield frame, num_tlvs

            # Remove the processed frame from the buffer
            buffer = buffer[total_packet_length:]

def process_frame(frame, numTlvs):
    """
    Processes a single frame to extract and analyze TLV data.

    Parameters:
    - frame: The full frame data as a bytearray.
    - numTlvs: The number of TLVs in the frame.

    Returns:
    - A dictionary containing:
        - 'tlv_types': A list of TLV types detected in the frame.
        - 'targets': Target list (if any).
        - 'cloud': Point cloud data (if any).
        - 'target_ids': Target IDs (if any).
        - 'presence': Presence indication (if any).
    """
    currentLoc = 40  # Start after Frame Header
    frame_tlv_types = []  # TLV types for this frame
    targets = None
    cloud = None
    target_ids = None
    presence = None

    for _ in range(numTlvs):
        # Check if there are enough bytes left for TLV Header
        if currentLoc + 8 > len(frame):
            print(f"Insufficient data for TLV header at loc {currentLoc}. Frame length: {len(frame)}")
            break

        # Extract TLV Header
        tlvType = uint32_converter(frame[currentLoc:currentLoc+4])
        tlvLength = uint32_converter(frame[currentLoc+4:currentLoc+8])
        
        # Check if there are enough bytes left for the full TLV
        if currentLoc + 8 + tlvLength > len(frame):
            print(f"Insufficient data for TLV {tlvType} at loc {currentLoc}. TLV length: {tlvLength}. Frame length: {len(frame)}")
            break
        
        frame_tlv_types.append(tlvType)
        currentLoc += 8

        # Process based on TLV Type
        if tlvType == 1010:
            targets = targetList(frame[currentLoc : currentLoc + tlvLength], tlvLength)
            # print("\nTarget List")
            # print(len(targets))
            # print(targets)
        if tlvType == 1020:
            cloud = pointCloud(frame[currentLoc : currentLoc + tlvLength], tlvLength)
            # print("Point Cloud")
            # print(len(cloud))
            # print(cloud)
        elif tlvType == 1011:
            target_ids = frame[currentLoc:currentLoc + tlvLength]
            target_ids = np.array(target_ids, dtype=np.uint8)
            # print("Target ID")
            # print(len(target_ids))
            # print(target_ids)
        elif tlvType == 1021:
            presenceIndication = uint32_converter(frame[currentLoc: currentLoc + 4])
            presence = presenceIndication == 1
            # print("Presence")
            # print(presence)
        # Move to the next TLV
        currentLoc += tlvLength

    return {
        'tlv_types': frame_tlv_types,
        'targets': targets,
        'cloud': cloud,
        'target_ids': target_ids,
        'presence': presence
    }