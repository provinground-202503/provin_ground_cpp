import numpy as np
import matplotlib.pyplot as plt

class Vehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, L=2.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L  # Wheelbase of the vehicle

    def update(self, dt, steer_angle):
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += (self.v / self.L) * np.tan(steer_angle) * dt
        self.yaw = self.normalize_angle(self.yaw)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

def pure_pursuit_steer_control(vehicle, path, look_ahead_distance):
    """
    Calculates the steering angle for pure pursuit.

    Args:
        vehicle (Vehicle): The current vehicle object.
        path (np.array): A 2D array of path points [[x1, y1], [x2, y2], ...].
        look_ahead_distance (float): The look-ahead distance.

    Returns:
        float: The calculated steering angle.
        int: The index of the target point on the path.
    """
    min_dist = float('inf')
    target_idx = -1

    # Find the closest point on the path to the vehicle's current position
    # This also helps in handling cases where the vehicle is slightly off the path
    for i, point in enumerate(path):
        dist = np.hypot(vehicle.x - point[0], vehicle.y - point[1])
        if dist < min_dist:
            min_dist = dist
            target_idx = i

    # Find the look-ahead point
    # Iterate from the closest point forward to find a point that is at least
    # the look_ahead_distance away from the vehicle.
    # We add a small epsilon to look_ahead_distance to avoid issues when the vehicle is exactly on a path point
    current_look_ahead_point_idx = target_idx
    for i in range(target_idx, len(path)):
        dist_from_vehicle_to_path_point = np.hypot(vehicle.x - path[i][0], vehicle.y - path[i][1])
        if dist_from_vehicle_to_path_point >= look_ahead_distance:
            current_look_ahead_point_idx = i
            break
    else:
        # If no point is found beyond look_ahead_distance, use the last point of the path
        # This is important for ensuring the vehicle reaches the end of the path
        current_look_ahead_point_idx = len(path) - 1

    target_point = path[current_look_ahead_point_idx]

    # Calculate the angle to the look-ahead point in the vehicle's frame
    alpha = np.arctan2(target_point[1] - vehicle.y, target_point[0] - vehicle.x) - vehicle.yaw

    # Pure pursuit steering angle formula
    steer_angle = np.arctan2(2.0 * vehicle.L * np.sin(alpha), look_ahead_distance)
    return steer_angle, current_look_ahead_point_idx

def generate_rectangle_path(width, height, center_x, center_y, num_points_per_side=50, start_corner="bottom_left"):
    """Generates a rectangular path."""
    half_width = width / 2.0
    half_height = height / 2.0

    # Define corners
    bl = [center_x - half_width, center_y - half_height] # Bottom-left
    br = [center_x + half_width, center_y - half_height] # Bottom-right
    tr = [center_x + half_width, center_y + half_height] # Top-right
    tl = [center_x - half_width, center_y + half_height] # Top-left

    path_segments = []

    # Order the corners based on start_corner
    corners = []
    if start_corner == "bottom_left":
        corners = [bl, br, tr, tl, bl] # Close the loop
    elif start_corner == "bottom_right":
        corners = [br, tr, tl, bl, br]
    elif start_corner == "top_right":
        corners = [tr, tl, bl, br, tr]
    elif start_corner == "top_left":
        corners = [tl, bl, br, tr, tl]
    else:
        raise ValueError("Invalid start_corner. Choose from 'bottom_left', 'bottom_right', 'top_right', 'top_left'.")

    for i in range(len(corners) - 1):
        p1 = corners[i]
        p2 = corners[i+1]

        # Generate points along the segment
        xs = np.linspace(p1[0], p2[0], num_points_per_side)
        ys = np.linspace(p1[1], p2[1], num_points_per_side)
        path_segments.append(np.array([xs, ys]).T)

    return np.vstack(path_segments)

def main():
    # --- Simulation Parameters ---
    dt = 0.1  # Time step [s]
    simulation_time = 200.0  # Total simulation time [s]
    vehicle_speed = 2.0  # Constant vehicle speed [m/s]
    look_ahead_distance = 2.1  # Look-ahead distance for pure pursuit [m]
    
    # --- Rectangular Path Parameters ---
    path_width = 20.0
    path_height = 10.0
    path_center_x = 0.0
    path_center_y = 0.0
    num_points_per_side = 50 # Density of points on each side
    
    # --- Vehicle Initialization ---
    # Start near a corner for easier visualization of path following
    initial_x = -path_width / 2.0 - 2.0 # Slightly outside bottom-left corner
    initial_y = -path_height / 2.0 - 2.0
    initial_yaw = np.deg2rad(90) # Pointing upwards
    vehicle = Vehicle(x=initial_x, y=initial_y, yaw=initial_yaw, v=vehicle_speed)

    # --- Generate Path ---
    path = generate_rectangle_path(path_width, path_height, path_center_x, path_center_y, num_points_per_side)

    # --- Simulation Loop ---
    time = 0.0
    history_x, history_y = [], []
    
    while time < simulation_time:
        steer_angle, target_idx = pure_pursuit_steer_control(vehicle, path, look_ahead_distance)
        vehicle.update(dt, steer_angle)

        history_x.append(vehicle.x)
        history_y.append(vehicle.y)

        time += dt

        # Optional: Plot in real-time (can slow down simulation)
        # plt.clf()
        # plt.plot(path[:, 0], path[:, 1], "r--", label="Target Path")
        # plt.plot(history_x, history_y, "b-", label="Vehicle Trajectory")
        # plt.plot(vehicle.x, vehicle.y, "go", markersize=8, label="Vehicle Current Pos")
        # if target_idx < len(path): # Ensure target_idx is valid for plotting
        #    plt.plot(path[target_idx, 0], path[target_idx, 1], "kx", markersize=10, label="Target Point")
        # plt.axis("equal")
        # plt.grid(True)
        # plt.legend()
        # plt.pause(0.01)

    # --- Plotting Results ---
    plt.figure(figsize=(10, 8))
    plt.plot(path[:, 0], path[:, 1], "r--", label="Target Path")
    plt.plot(history_x, history_y, "b-", label="Vehicle Trajectory")
    plt.plot(initial_x, initial_y, "go", markersize=8, label="Initial Position")
    plt.plot(history_x[-1], history_y[-1], "mo", markersize=8, label="Final Position")
    
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Pure Pursuit Path Following (Rectangular Path)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()