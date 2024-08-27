import numpy as np
import matplotlib.pyplot as plt

def interpolate_line(p1, p2, p3, t, strength=1.0):
    assert(strength >= 0.0 and strength <= 1.0)

    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)

    v1 = p2 - p1
    v2 = p3 - p2

    p1 = p1 + (1 - strength) * v1
    p3 = p2 + strength * v2

    v1 = p2 - p1
    v2 = p3 - p2

    a = p1 + t * v1
    b = p2 + t * v2

    v3 = b - a
    c = a + t * v3

    return c

def plot_interpolated_path(p1, p2, p3, num_frames=10, strength=1.0):
    """
    Plots the points representing the path of the blend between the start_line and end_line.
    
    Parameters:
    - start_line: A tuple of tuples containing the start and end points of the first line.
    - end_line: A tuple of tuples containing the start and end points of the second line.
    - num_frames: The number of interpolation points to generate.
    - strength: A float >= 0, where 1 means normal blend, >1 virtually extends the lines for a stronger blend.
    """

    plt.figure(figsize=(8, 6))
    
    # Arrays to store the interpolated path points
    path_points = []
    
    for i in range(num_frames + 1):
        t = i / num_frames
        point_on_path = interpolate_line(p1, p2, p3, t, strength=strength)
        path_points.append(point_on_path)
    
    # Convert the list to an array for easier plotting
    path_points = np.array(path_points)
    
    # Plot the interpolated path points
    plt.plot(path_points[:, 0], path_points[:, 1], 'bo-', label='Interpolated Path')
    
    # Plot the start and end lines for reference
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'ro-', label="Start Line")
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'mo-', label="End Line")
    
    plt.legend()
    plt.title(f'Interpolated Path Between Two Lines (Strength={strength})')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()

# Define the start and end lines
p1 = (0,0)
p2 = (4,4)
p3 = (6,0)

# Plot the interpolated path with different blend strengths
plot_interpolated_path(p1, p2, p3, num_frames=20, strength=0)  # Weaker blend
plot_interpolated_path(p1, p2, p3, num_frames=20, strength=0.1)  # Weaker blend
plot_interpolated_path(p1, p2, p3, num_frames=20, strength=0.2)  # Weaker blend
plot_interpolated_path(p1, p2, p3, num_frames=20, strength=0.5)  # Weaker blend
plot_interpolated_path(p1, p2, p3, num_frames=20, strength=1.0)  # Normal blend
