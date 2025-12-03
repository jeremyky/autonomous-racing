import matplotlib.pyplot as plt
import numpy as np
import csv

# Global variables
points = []  # List of (x, y, speed) tuples
selected_idx = 0
selected_point_marker = None
line = None
scatter = None

def load_raceline(filename):
    """Load raceline from CSV file."""
    global points
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        points = [(float(row[0]), float(row[1]), float(row[2])) for row in reader]

def save_raceline(filename):
    """Save raceline to CSV file."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(points)
    print(f"Saved raceline to {filename}")

def update_visualization():
    """Update the plot with current points and speeds."""
    global line, scatter, selected_point_marker
    
    # Clear previous plots
    if line is not None:
        line.remove()
    if scatter is not None:
        scatter.remove()
    if selected_point_marker is not None:
        selected_point_marker.remove()

    # Extract coordinates and speeds
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    speeds = [p[2] for p in points]

    # Plot line and points
    line, = ax.plot(x, y, 'b-', linewidth=1)
    scatter = ax.scatter(x, y, c=speeds, cmap='plasma', s=30)
    
    # Highlight selected point
    selected_point_marker = ax.plot(points[selected_idx][0], 
                                  points[selected_idx][1], 
                                  'yo', markersize=15, 
                                  label=f'Speed: {points[selected_idx][2]:.2f}')[0]
    
    ax.legend()
    fig.canvas.draw()

def smooth_speeds(center_idx, delta):
    """Smooth speeds around the modified point using a gaussian window."""
    window_size = 10
    sigma = 2
    
    # Create Gaussian window weights
    x = np.arange(-window_size//2, window_size//2 + 1)
    gaussian = np.exp(-(x**2)/(2*sigma**2))
    
    # Apply speed change with gaussian falloff
    for i in range(max(0, center_idx - window_size//2), 
                  min(len(points), center_idx + window_size//2 + 1)):
        weight = gaussian[i - (center_idx - window_size//2)]
        points[i] = (points[i][0], points[i][1], 
                    max(0.1, points[i][2] + delta * weight))

def on_key(event):
    """Handle keyboard events."""
    global selected_idx
    
    if event.key == 'left':
        selected_idx = (selected_idx - 1) % len(points)
    elif event.key == 'right':
        selected_idx = (selected_idx + 1) % len(points)
    elif event.key == 'up':
        smooth_speeds(selected_idx, 0.1)  # Increase speed
    elif event.key == 'down':
        smooth_speeds(selected_idx, -0.1)  # Decrease speed
    elif event.key == 's':
        save_raceline('edited_raceline.csv')
    
    # Update title with current point info
    ax.set_title(f'Point {selected_idx}/{len(points)-1}\n'
                f'Speed: {points[selected_idx][2]:.2f}\n'
                f'Left/Right: Select point, Up/Down: Adjust speed, s: Save')
    
    update_visualization()

# Set up the plot
fig, ax = plt.subplots(figsize=(12, 8))
ax.set_aspect('equal')

# Load initial raceline
load_raceline('drawn_line.csv')

# Initial visualization
ax.set_title('Left/Right: Select point, Up/Down: Adjust speed, s: Save')
update_visualization()

# Connect keyboard handler
fig.canvas.mpl_connect('key_press_event', on_key)

plt.show()
