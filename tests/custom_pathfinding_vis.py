import ctypes
import numpy as np
import matplotlib.pyplot as plt
import sys

# Load the shared library
pathfinding_lib = ctypes.CDLL('../build/libs/libcustom_pathfinding_vis_test.so')

# Define the argument and return types for the find_path function
pathfinding_lib.find_path.argtypes = [
    ctypes.c_double, ctypes.c_double, ctypes.c_double,  # start_x, start_y, start_z
    ctypes.c_double, ctypes.c_double, ctypes.c_double,  # goal_x, goal_y, goal_z
    ctypes.POINTER(ctypes.c_int)
]
pathfinding_lib.find_path.restype = ctypes.POINTER(ctypes.c_double)

# Define the argument and return types for the find_path_random function
pathfinding_lib.find_path_random.argtypes = [ctypes.POINTER(ctypes.c_int)]
pathfinding_lib.find_path_random.restype = ctypes.POINTER(ctypes.c_double)

# Define the argument type for the free_memory function
pathfinding_lib.free_memory.argtypes = [ctypes.POINTER(ctypes.c_double)]

def find_path_with_params(start=None, goal=None):
    path_size = ctypes.c_int()

    if start is not None and goal is not None:
        # Call the C++ function with custom start and goal positions
        result_ptr = pathfinding_lib.find_path(
            start[0], start[1], start[2],
            goal[0], goal[1], goal[2],
            ctypes.byref(path_size)
        )
    else:
        # Call the C++ function to get a random path
        result_ptr = pathfinding_lib.find_path_random(ctypes.byref(path_size))

    if not result_ptr:
        print("No path found.")
        return None

    # Copy the data into Python's memory
    path_array = np.ctypeslib.as_array(result_ptr, shape=(path_size.value,))
    path_array = np.copy(path_array)

    # Free the memory allocated by the C++ function
    pathfinding_lib.free_memory(result_ptr)

    # Reshape to an Nx3 array for 3D points
    path_array = path_array.reshape(-1, 3)

    return path_array

def plot_path(path_array):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the path
    ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2], marker='o', color='blue')

    # Highlight the first point in green
    ax.scatter(path_array[0, 0], path_array[0, 1], path_array[0, 2], color='green', label='Start')

    # Highlight the last point in red
    ax.scatter(path_array[-1, 0], path_array[-1, 1], path_array[-1, 2], color='red', label='Goal')

    # Add labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show legend
    ax.legend()

    plt.show()

def parse_arguments():
    if len(sys.argv) == 7:
        # Parse start and goal positions from command line arguments
        try:
            start = list(map(float, sys.argv[1:4]))
            goal = list(map(float, sys.argv[4:7]))
            return start, goal
        except ValueError:
            print("Error: Invalid start or goal position format.")
            sys.exit(1)
    elif len(sys.argv) == 1:
        # No custom positions provided
        return None, None
    else:
        print("Usage: python script.py [start_x start_y start_z goal_x goal_y goal_z]")
        sys.exit(1)

if __name__ == "__main__":
    start_pos, goal_pos = parse_arguments()

    if start_pos is not None and goal_pos is not None:
        path_array = find_path_with_params(start_pos, goal_pos)
        if path_array is None:
            print("No path")
            print(path_array)
            exit(0)

        print(path_array)
        plot_path(path_array)
        exit(0)

    while True:
        user_input = input("Press Enter to generate a new path or type 'q' to quit: ")
        if user_input.lower() == 'q':
            break
        path_array = find_path_with_params(start_pos, goal_pos)
        if path_array is None:
            print("No path")
            print(path_array)

        print(path_array)
        plot_path(path_array)
