import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import csv

# Load the map image
map_image_path = '../map/base_map.pgm'
map_image = Image.open(map_image_path)
map_array = np.array(map_image)

# Map metadata from the YAML file
origin_x, origin_y = -6.977912, -3.423147  # Origin of the map
resolution = 0.025000  # Resolution of the map


# Function to convert image coordinates to map coordinates
def image_to_map_coordinates(x_img, y_img, origin_x, origin_y, resolution):
    x_map = x_img * resolution + origin_x
    y_map = (map_array.shape[0] - y_img) * resolution + origin_y
    return x_map, y_map


# Initialize lists to store clicked points
x_clicked = []
y_clicked = []
speeds = []  # Store speeds for each point
line = None
points = None
x_interpolated = []
y_interpolated = []
speeds_interpolated = []  # Store interpolated speeds
point_markers = []  # Store point markers for undo functionality
selected_point_idx = -1  # Index of currently selected point
selected_point_marker = None


def catmull_rom_spline(P0, P1, P2, P3, S0, S1, S2, S3, num_points=100):
    """
    Compute Catmull-Rom spline between P1 and P2 with speed interpolation.
    """
    # Convert points to numpy arrays
    P0, P1, P2, P3 = map(np.array, [P0, P1, P2, P3])
    
    # Create t points
    t = np.linspace(0, 1, num_points)
    
    # Calculate points
    points = []
    speeds = []
    for ti in t:
        point = 0.5 * ((2*P1) +
                       (-P0 + P2) * ti +
                       (2*P0 - 5*P1 + 4*P2 - P3) * ti**2 +
                       (-P0 + 3*P1 - 3*P2 + P3) * ti**3)
        
        # Linear interpolation for speed
        speed = S1 + (S2 - S1) * ti
        
        points.append(point)
        speeds.append(speed)
    
    return np.array(points), np.array(speeds)


# Function to generate and display spline
def generate_spline():
    global line, points, x_interpolated, y_interpolated, speeds_interpolated
    if len(x_clicked) >= 4:  # Need at least 4 points for Catmull-Rom
        if line is not None:
            line.remove()
        if points is not None:
            points.remove()
        
        x_sorted, y_sorted = x_clicked, y_clicked
        speeds_sorted = speeds
        
        # Create closed loop
        x_loop = x_sorted[-1:] + x_sorted + x_sorted[:2]
        y_loop = y_sorted[-1:] + y_sorted + y_sorted[:2]
        speeds_loop = speeds_sorted[-1:] + speeds_sorted + speeds_sorted[:2]
        
        x_spline = []
        y_spline = []
        speeds_spline = []
        
        for i in range(len(x_sorted)):
            points, segment_speeds = catmull_rom_spline(
                (x_loop[i], y_loop[i]),
                (x_loop[i+1], y_loop[i+1]),
                (x_loop[i+2], y_loop[i+2]),
                (x_loop[i+3], y_loop[i+3]),
                speeds_loop[i],
                speeds_loop[i+1], 
                speeds_loop[i+2],
                speeds_loop[i+3],
                num_points=10
            )
            
            x_spline.extend(points[:, 0])
            y_spline.extend(points[:, 1])
            speeds_spline.extend(segment_speeds)
        
        x_interpolated = np.array(x_spline)
        y_interpolated = np.array(y_spline)
        speeds_interpolated = np.array(speeds_spline)
        
        line, = ax.plot(x_spline, y_spline, 'b-', linewidth=2)
        points = ax.scatter(x_spline, y_spline, c=speeds_spline, cmap='plasma', s=10)
        
        # Update title to show selected point speed if one is selected
        if selected_point_idx >= 0:
            ax.set_title(f'Click to draw points, r to reset, z to undo\nLeft/Right arrows to select point, Up/Down to change speed\nSelected point speed: {speeds[selected_point_idx]:.1f}')
        else:
            ax.set_title('Click to draw points, r to reset, z to undo\nLeft/Right arrows to select point, Up/Down to change speed')
            
        fig.canvas.draw()


# Function to handle mouse click events
def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        x_clicked.append(event.xdata)
        y_clicked.append(event.ydata)
        speeds.append(1.0)  # Default speed
        point_marker, = ax.plot(event.xdata, event.ydata, 'ro')
        point_markers.append(point_marker)
        generate_spline()
        fig.canvas.draw()


# Function to reset the canvas
def reset_canvas():
    global x_clicked, y_clicked, speeds, line, points, x_interpolated, y_interpolated, speeds_interpolated, point_markers, selected_point_idx, selected_point_marker
    x_clicked = []
    y_clicked = []
    speeds = []
    x_interpolated = []
    y_interpolated = []
    speeds_interpolated = []
    selected_point_idx = -1
    if line is not None:
        line.remove()
        line = None
    if points is not None:
        points.remove()
        points = None
    if selected_point_marker is not None:
        selected_point_marker.remove()
        selected_point_marker = None
    for marker in point_markers:
        marker.remove()
    point_markers = []
    ax.clear()
    ax.imshow(map_array, cmap='gray')
    ax.set_title('Click to draw points, r to reset, z to undo\nLeft/Right arrows to select point, Up/Down to change speed')
    fig.canvas.draw()


# Function to handle key press events
def onkey(event):
    global line, points, x_interpolated, y_interpolated, selected_point_idx, selected_point_marker
    
    if event.key == 'r':
        reset_canvas()
    elif event.key == 'z' and x_clicked:
        x_clicked.pop()
        y_clicked.pop()
        speeds.pop()
        point_markers[-1].remove()
        point_markers.pop()
        if line is not None:
            line.remove()
            line = None
        if points is not None:
            points.remove()
            points = None
        generate_spline()
        fig.canvas.draw()
    elif event.key in ['left', 'right'] and x_clicked:
        if selected_point_marker is not None:
            selected_point_marker.remove()
        
        if event.key == 'left':
            selected_point_idx = (selected_point_idx - 1) % len(x_clicked)
        else:
            selected_point_idx = (selected_point_idx + 1) % len(x_clicked)
            
        selected_point_marker, = ax.plot(x_clicked[selected_point_idx], 
                                       y_clicked[selected_point_idx], 
                                       'yo', markersize=10)
        ax.set_title(f'Click to draw points, r to reset, z to undo\nLeft/Right arrows to select point, Up/Down to change speed\nSelected point speed: {speeds[selected_point_idx]:.1f}')
        fig.canvas.draw()
        
    elif event.key in ['up', 'down'] and selected_point_idx >= 0:
        speed_delta = 0.1 if event.key == 'up' else -0.1
        speeds[selected_point_idx] = max(0.1, speeds[selected_point_idx] + speed_delta)
        generate_spline()
        # Redraw selected point marker
        if selected_point_marker is not None:
            selected_point_marker.remove()
        selected_point_marker, = ax.plot(x_clicked[selected_point_idx], 
                                       y_clicked[selected_point_idx], 
                                       'yo', markersize=10)
        ax.set_title(f'Click to draw points, r to reset, z to undo\nLeft/Right arrows to select point, Up/Down to change speed\nSelected point speed: {speeds[selected_point_idx]:.1f}')
        fig.canvas.draw()


# Function to save the clicked points to a CSV file
def save_race_line(filename='race_line.csv'):
    if len(x_interpolated) == 0:
        print("No points to save - please create a spline first")
        return
        
    with open(filename, mode='w') as file:
        writer = csv.writer(file)
        for x_img, y_img, speed in zip(x_interpolated, y_interpolated, speeds_interpolated):
            x_map, y_map = image_to_map_coordinates(x_img, y_img, origin_x, origin_y, resolution)
            writer.writerow([x_map, y_map, speed])
    print("Race line saved to", filename)


# Set up the plot
fig, ax = plt.subplots()
ax.imshow(map_array, cmap='gray')
ax.set_title('Click to draw points, r to reset, z to undo\nLeft/Right arrows to select point, Up/Down to change speed')
fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', onkey)

plt.show()

# Save the race line after the plot window is closed
save_race_line('drawn_line.csv')
