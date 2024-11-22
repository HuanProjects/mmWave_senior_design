import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
from OutputReader import outputDataParser
import itertools

# constants
CONFIG_FILE = "config.cfg"
CLI_PORT = "COM5"
DATA_PORT = "COM3"
DEBUG = False
MAX_DATA_LENGTH = 2**15
c = 3e8

# function to configure the radar
# @param configFile: the directory of the current config file
# @return cliPort  : control port
# @return dataPort : data port
def configRadar(configFile):
    # read config files and strip of extra white spaces
    with open(configFile) as file:
        config = [line.strip() for line in file]

    # set up the config ports
    cliPort = serial.Serial(CLI_PORT, 115200)
    dataPort = serial.Serial(DATA_PORT, 921600)

    # write config files to the radar
    for line in config:
        cliPort.write((line + '\n').encode())
        time.sleep(0.01)
    return cliPort, dataPort

# function to process radar data received from the data port
# @param dataPort       : the serial port object connected to the radar's data interface
# @return goodData      : boolean indicating if valid data was processed
# @return detectedObjects : a dictionary containing detected objects' information
#                           keys: "numObj", "x", "y", "z", "doppler"
def processData(dataPort):
    # Flag to indicate if valid data was processed
    goodData = False
    # Initialize an empty dictionary to store detected objects
    detectedObjects = {
        "numObj": 0,  # Number of detected objects
        "x": [],      # X-coordinates of detected objects
        "y": [],      # Y-coordinates of detected objects
        "z": [],      # Z-coordinates of detected objects
        "doppler": [] # Doppler velocities of detected objects
    }

    try:
        # Read all available bytes from the data port
        readBuffer = dataPort.read(dataPort.in_waiting)
        # Convert the raw bytes to a NumPy array for processing
        byteArray = np.frombuffer(readBuffer, dtype='uint8')

        # Check if data was received
        if len(byteArray) > 0:
            # Parse the byte array to extract radar data
            goodData, numDetectedObj, xPointsLocations, \
            yPointsLocations, zPointsLocations, \
            radialDopplerVelocities = outputDataParser(byteArray, DEBUG)

        # If valid data was parsed, update the detectedObjects dictionary
        if goodData:
            detectedObjects.update({
                "numObj": numDetectedObj,        # Number of detected objects
                "x": xPointsLocations,          # X-coordinates
                "y": yPointsLocations,          # Y-coordinates
                "z": zPointsLocations,          # Z-coordinates
                "doppler": radialDopplerVelocities # Doppler velocities
            })

    except Exception as e:
        # Handle exceptions during data processing
        print(f"Error processing data: {e}")
        goodData = False  # Set flag to indicate failure

    # Debugging information
    if DEBUG:
        print(f"Read {len(byteArray)} bytes from data port.")
        if goodData:
            print(f"Detected {detectedObjects['numObj']} objects.")
        else:
            print("No valid data detected.")

    # Return the status flag and the detected objects dictionary
    return goodData, detectedObjects

# Function to parse a radar configuration file
# @param configFile     : the path to the radar configuration file
# @return params        : a dictionary containing configuration 
#                        parameters as key-value pairs
def parseConfigFile(configFile):
    params = {}
    with open(configFile, 'r') as f:
        for line in f:
            line = line.strip()
            # Skip empty lines and lines that don't start with '%'
            if not line or not line.startswith('%'):
                continue
            # Remove '%' and any leading/trailing whitespace
            line = line.lstrip('%').strip()
            # Split the line into key and value
            if ':' in line:
                key, value = line.split(':', 1)
                params[key.strip()] = value.strip()
    return params

# Main function to configure radar, visualize data, and manage updates
def main():
    if not os.path.exists(CONFIG_FILE):
        print(f"Error: Configuration file {CONFIG_FILE} not found.")
        return

    # Parse the config file to extract parameters
    radarParams = parseConfigFile(CONFIG_FILE)

    # Extract parameters and convert to appropriate types
    max_range = float(radarParams.get('Maximum unambiguous Range(m)'))
    max_velocity = float(radarParams.get('Maximum Radial Velocity(m/s)'))
    frequency = float(radarParams.get('Frequency')) * 1e9  # Convert GHz to Hz
    f_c = frequency  # Carrier frequency in Hz

    # Set axis limits based on extracted parameters
    x_lim = max_range
    y_lim = max_range
    z_lim = max_range  # Assuming similar limits for vertical axis

    # Calculate maximum Doppler shift
    max_doppler_shift = (2 * max_velocity * f_c) / c

    # Connect to the radar
    cliPort, dataPort = configRadar(CONFIG_FILE)

    # Create a figure with four subplots arranged in a 2x2 grid
    fig = plt.figure(figsize=(8, 7))

    # Plot 1: X vs. Y (Left/Right vs. Forward/Backward)
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title("Top down view (X vs. Y)", fontsize=8)
    ax1.set_xlabel("X-axis (Left/Right) [m]", fontsize=7)
    ax1.set_ylabel("Y-axis (Forward/Backward) [m]", fontsize=7)
    ax1.set_xlim(-x_lim, x_lim)
    ax1.set_ylim(0, y_lim)
    scatter_xy = ax1.scatter([], [], c=[], cmap='viridis')
    ax1.tick_params(axis='both', which='major', labelsize=7)

    # Plot 2: X vs. Z (Left/Right vs. Up/Down)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title("Vertical plane (X vs. Z)", fontsize=8)
    ax2.set_xlabel("X-axis (Left/Right) [m]", fontsize=7)
    ax2.set_ylabel("Z-axis (Up/Down) [m]", fontsize=7)
    ax2.set_xlim(-x_lim, x_lim)
    ax2.set_ylim(-z_lim, z_lim)
    scatter_xz = ax2.scatter([], [], c=[], cmap='viridis')
    ax2.tick_params(axis='both', which='major', labelsize=7)

    # Plot 3: Histogram of Doppler Shift
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title("Histogram of Doppler Shift", fontsize=8)
    ax3.set_xlabel("Doppler Shift (Hz)", fontsize=7)
    ax3.set_ylabel("Number of Detections", fontsize=7)
    ax3.set_xlim(-max_doppler_shift, max_doppler_shift)
    ax3.tick_params(axis='both', which='major', labelsize=7)
    hist_doppler, = ax3.plot([], [], lw=2)  # Empty plot for updating

    # Plot 4: 3D Plot of X, Y, Z with Doppler Shift
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    ax4.set_title("3D Radar Detections", fontsize=8)
    ax4.set_xlabel("X-axis (Left/Right) [m]", fontsize=7)
    ax4.set_ylabel("Y-axis (Forward/Backward) [m]", fontsize=7)
    ax4.set_zlabel("Z-axis (Up/Down) [m]", fontsize=7)
    ax4.set_xlim(-x_lim, x_lim)
    ax4.set_ylim(0, y_lim)
    ax4.set_zlim(-z_lim, z_lim)
    scatter_3d = ax4.scatter([], [], [], c=[], cmap='viridis')
    ax4.tick_params(axis='both', which='major', labelsize=7)

    def update(frame):
        dataOk, detObj = processData(dataPort)
        if dataOk:
             # Convert data to NumPy arrays
            x = np.array(detObj["x"])  # Left/Right
            y = np.array(detObj["y"])  # Forward/Backward
            z = np.array(detObj["z"])  # Up/Down
            v_d = np.array(detObj["doppler"])  # Doppler velocity in m/s

            # Calculate Doppler shift
            f_d = (2 * v_d * f_c) / c  # Doppler shift in Hz

            # Update Plot 1: X vs. Y
            scatter_xy.set_offsets(np.c_[x, y])
            scatter_xy.set_array(f_d)
            scatter_xy.set_clim(vmin=-max_doppler_shift, vmax=max_doppler_shift)

            # Update Plot 2: X vs. Z
            scatter_xz.set_offsets(np.c_[x, z])
            scatter_xz.set_array(f_d)
            scatter_xz.set_clim(vmin=-max_doppler_shift, vmax=max_doppler_shift)

            # Update Plot 3: Histogram of Doppler Shift
            ax3.cla()  # Clear the previous histogram
            ax3.set_title("Histogram of Doppler Shift", fontsize=8)
            ax3.set_xlabel("Doppler Shift (Hz)", fontsize=7)
            ax3.set_ylabel("Number of Detections", fontsize=7)
            ax3.set_xlim(-max_doppler_shift, max_doppler_shift)
            ax3.tick_params(axis='both', which='major', labelsize=7)
            bins = np.linspace(-max_doppler_shift, max_doppler_shift, 30)
            ax3.hist(f_d, bins=bins, color='blue', alpha=0.7)

            # Update Plot 4: 3D Plot
            ax4.cla()  # Clear the previous scatter
            ax4.set_title("3D Radar Detections", fontsize=8)
            ax4.set_xlabel("X-axis (Left/Right) [m]", fontsize=7)
            ax4.set_ylabel("Y-axis (Forward/Backward) [m]", fontsize=7)
            ax4.set_zlabel("Z-axis (Up/Down) [m]", fontsize=7)
            ax4.set_xlim(-x_lim, x_lim)
            ax4.set_ylim(0, y_lim)
            ax4.set_zlim(-z_lim, z_lim)
            scatter_3d = ax4.scatter(x, y, z, c=f_d, cmap='viridis')
            ax4.tick_params(axis='both', which='major', labelsize=7)

    ani = FuncAnimation(fig, update, frames=itertools.count(), interval=100, blit=False)

    plt.tight_layout()
    plt.show()

    # When the plot window is closed, stop the radar and close the ports
    print("Stopping radar and closing ports...")
    cliPort.write(b'sensorStop\n')
    cliPort.close()
    dataPort.close()

if __name__ == "__main__":
    main()
