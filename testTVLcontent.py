import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from OutputReader import record_frames, process_frame
from plot_helpers import setup_plot, update_plot  # Assuming these are saved in `plot_helpers.py`
import time 
import re
from collections import deque

# Configuration
CLI_PORT = 'COM5'
DATA_PORT = 'COM3'
configFileName = 'config2.cfg'

def setup_sensor(configFileName):
    """
    Set up the radar sensor with the given configuration.
    """
    import serial
    cli_port = serial.Serial(CLI_PORT, 115200)
    data_port = serial.Serial(DATA_PORT, 921600)

    with open(configFileName, 'r') as config_file:
        config = config_file.readlines()

    for command in config:
        cli_port.write((command.strip() + '\n').encode())
        time.sleep(0.01)

    return cli_port, data_port

def parse_xyz_limits(config_text):
    """
    Parses the boundaryBox values from the radar configuration text.

    Parameters:
    - config_text: The radar configuration text as a string.

    Returns:
    - A dictionary with x_limits, y_limits, and z_limits as tuples.
      Example: {"x_limits": (-10, 10), "y_limits": (-10, 10), "z_limits": (-2, 10)}
    """
    pattern = r"boundaryBox\s+(-?\d+(\.\d+)?(?:\s+-?\d+(\.\d+)?){5})"
    match = re.search(pattern, config_text)

    if match:
        values = list(map(float, match.group(1).split()))
        limits = {
            "x_limits": (values[0], values[1]),
            "y_limits": (values[2], values[3]),
            "z_limits": (values[4], values[5])
        }
        return limits
    else:
        return None  # Return None if boundaryBox is not found

def collect_data(data_port, frame_deque):
    for frame, numTlvs in record_frames(data_port):
        processed_data = process_frame(frame, numTlvs)
        frame_deque.append(processed_data)
        yield None  # Trigger FuncAnimation update

def main():
    # Set up sensor
    cli_port, data_port = setup_sensor(configFileName)

    # Parse limits from the configuration file
    with open(configFileName, 'r') as config_file:
        config_text = config_file.read()

    limits = parse_xyz_limits(config_text)
    if not limits:
        raise ValueError("Could not find 'boundaryBox' in the configuration file.")

    # Set up plots with parsed limits
    fig, ax1, ax2, norm, cmap = setup_plot(limits)

    frame_deque = deque(maxlen=2)
    # Initialize the deque with one empty frame
    frame_deque.append({
        'tlv_types': [],
        'targets': None,
        'cloud': None,
        'target_ids': None,
        'presence': None
    })


    try:
        # Initialize FuncAnimation
        ani = FuncAnimation(
            fig,
            update_plot,
            frames=collect_data(data_port, frame_deque),
            fargs=(frame_deque, ax1, ax2, limits, norm, cmap),
            blit=False,
            interval=20,
            cache_frame_data=False
        )
        plt.show()

    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Stopping sensor...")

    finally:
        cli_port.write(('sensorStop\n').encode())
        cli_port.close()
        data_port.close()

if __name__ == "__main__":
    main()
