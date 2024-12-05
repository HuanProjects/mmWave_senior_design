import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize, ListedColormap

GREY_RBG = [0.9, 0.9, 0.9, 1.0]

def setup_plot(limits):
    """
    Sets up the matplotlib figure and axes for the top-down and 3D scatter plots.

    Parameters:
    - limits: Dictionary containing x_limits, y_limits, and z_limits.

    Returns:
    - fig: The matplotlib figure.
    - ax1: 2D top-down view axis.
    - ax2: 3D scatter plot axis.
    - scatter_top_down: Scatter plot object for 2D plot.
    - norm: Normalization instance for consistent color mapping.
    - cmap: Custom colormap for consistent coloring.
    """
    fig = plt.figure(figsize=(12, 6))

    # Extract limits
    x_limits = limits["x_limits"]
    y_limits = limits["y_limits"]
    z_limits = limits["z_limits"]

    # Create a normalization instance for consistent color mapping
    norm = Normalize(vmin=0, vmax=249)  # Adjust vmax based on your target_ids range

    # Create a custom colormap that includes gray for invalid target_ids
    base_cmap = plt.cm.viridis
    colors = base_cmap(np.linspace(0, 1, 250))  # Assuming target_ids from 0 to 249
    gray_color = np.array(GREY_RBG)  # RGBA for gray
    colors = np.vstack([colors, gray_color])  # Add gray color at the end
    cmap = ListedColormap(colors)

    # Top-down X-Y plane subplot
    ax1 = fig.add_subplot(121)
    ax1.set_xlim(*x_limits)
    ax1.set_ylim(*y_limits)
    ax1.set_title("Top-Down View (X-Y Plane)")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.grid(True)

    # 3D scatter plot for X, Y, Z with Target IDs coloring
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_xlim(*x_limits)
    ax2.set_ylim(*y_limits)
    ax2.set_zlim(*z_limits)
    ax2.set_title("3D Scatter with Target IDs Coloring")
    ax2.set_xlabel("X")
    ax2.set_ylabel("Y")
    ax2.set_zlabel("Z")

    return fig, ax1, ax2, norm, cmap

def update_plot(frame, frame_deque, ax1, ax2, limits, norm, cmap):
    """
    Updates the plots with new data from the cloud_data array.

    Parameters:
    - frame: Current animation frame (required for FuncAnimation but not used here).
    - frame_deque: Queue containing processed frames.
    - ax1: Matplotlib 2D axis for 2D scatter plot.
    - ax2: Matplotlib 3D axis for 3D scatter plot.
    - limits: Dictionary containing x_limits, y_limits, and z_limits.
    - norm: Normalization instance for consistent color mapping.
    - cmap: Custom colormap for consistent coloring.
    """
    cloud_data = {
        'x': np.array([]),
        'y': np.array([]),
        'z': np.array([]),
    }
    target_ids = np.array([])

    if len(frame_deque) >= 1:
        # Retrieve the previous frame
        current_frame = frame_deque[-2]

        if current_frame['cloud'] is not None:
            cloud_df = current_frame['cloud']
            cloud_data['x'] = cloud_df['x'].values
            cloud_data['y'] = cloud_df['y'].values
            cloud_data['z'] = cloud_df['z'].values

        # Retrieve the latest frame for target_ids
        next_frame = frame_deque[-1]
        if 'target_ids' in next_frame and next_frame['target_ids'] is not None:
            target_ids = next_frame['target_ids']
        else:
            target_ids = np.full(len(cloud_data['x']), -1)  # Use -1 for no target_ids
    
    # Check if the number of target_ids matches the length of cloud_data["x"]
    if len(target_ids) != len(cloud_data["x"]):
        print(target_ids)
        print(len(target_ids))
        print(cloud_data['x'])
        print(len(cloud_data['x']))
        return
    
    x_limits = limits["x_limits"]
    y_limits = limits["y_limits"]
    z_limits = limits["z_limits"]

    # Flip x values across the midpoint 0
    if cloud_data['x'].size > 0:
        cloud_data['x'] = -cloud_data['x']  # Negate the x values to reflect across 0

    # Clear and reset the 3D scatter plot if there's new data
    if cloud_data['x'].size > 0:
        ax2.cla()
        ax2.set_xlim(*x_limits)
        ax2.set_ylim(*y_limits)
        ax2.set_zlim(*z_limits)
        ax2.set_title("3D Scatter with Target IDs Coloring")
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        ax2.set_zlabel("Z")

        colors = get_colors(target_ids, norm, cmap)
        ax2.scatter(cloud_data['x'], cloud_data['y'], cloud_data['z'], c=colors, s=10, edgecolors='none')

    # Clear and reset the top-down plot if there's new data
    if cloud_data['x'].size > 0 and cloud_data['y'].size > 0:
        ax1.cla()
        ax1.set_xlim(*x_limits)
        ax1.set_ylim(*y_limits)
        ax1.set_title("Top-Down View (X-Y Plane)")
        ax1.set_xlabel("X")
        ax1.set_ylabel("Y")

        colors = get_colors(target_ids, norm, cmap)
        ax1.scatter(cloud_data['x'], cloud_data['y'], c=colors, s=10, edgecolors='none')

def get_colors(target_ids, norm, cmap):
    """
    Assign colors to points based on target_ids.

    Parameters:
    - target_ids: Array of target IDs.
    - norm: Normalization instance for consistent color mapping.
    - cmap: Custom colormap.

    Returns:
    - Array of RGBA colors.
    """
    if len(target_ids) == 0:
        # No target_ids detected, return gray for all points
        colors = np.array(GREY_RBG)  # RGBA for gray
        return colors

    # Initialize colors array
    colors = np.empty((len(target_ids), 4))  # RGBA

    # Mask for valid target_ids within the range 0 to 249
    valid_mask = (target_ids >= 0) & (target_ids <= 249)

    # Normalize valid target_ids
    normalized_ids = norm(target_ids[valid_mask])

    # Map normalized target_ids to colors using the colormap
    colors[valid_mask] = cmap(normalized_ids)

    # Assign gray color to invalid target_ids
    colors[~valid_mask] = np.array(GREY_RBG)  # RGBA for gray

    return colors
