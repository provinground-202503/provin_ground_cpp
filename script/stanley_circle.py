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
        # Update based on kinematic model
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += (self.v / self.L) * np.tan(steer_angle) * dt
        self.yaw = self.normalize_angle(self.yaw)

    def normalize_angle(self, angle):
        """Normalizes an angle to be within the range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

def stanley_steer_control(vehicle, path, k=0.5, k_soft=0.1):
    """
    Calculates the steering angle using the Stanley method.

    Args:
        vehicle (Vehicle): The current vehicle object.
        path (np.array): A 2D array of path points [[x1, y1], [x2, y2], ...].
        k (float): Gain for cross-track error (k_e).
        k_soft (float): Softening constant for the denominator to prevent division by zero at low speeds.

    Returns:
        float: The calculated steering angle.
        int: The index of the nearest point on the path (for reference/plotting).
    """
    # 1. Find the nearest point on the path to the vehicle's front axle
    # Vehicle front axle position
    front_axle_x = vehicle.x + vehicle.L * np.cos(vehicle.yaw)
    front_axle_y = vehicle.y + vehicle.L * np.sin(vehicle.yaw)

    min_dist = float('inf')
    closest_point_idx = -1
    
    for i, point in enumerate(path):
        dist = np.hypot(front_axle_x - point[0], front_axle_y - point[1])
        if dist < min_dist:
            min_dist = dist
            closest_point_idx = i

    target_point = path[closest_point_idx]
    
    # Calculate the heading of the path segment at the closest point
    # For a circular path, we need to be careful with indexing for heading calculation.
    # We can use the next point for the heading, wrapping around if at the end.
    if closest_point_idx + 1 < len(path):
        path_heading = np.arctan2(path[closest_point_idx + 1][1] - path[closest_point_idx][1],
                                  path[closest_point_idx + 1][0] - path[closest_point_idx][0])
    else: 
        # If at the last point, calculate heading using the last and first point to close the circle
        path_heading = np.arctan2(path[0][1] - path[closest_point_idx][1],
                                  path[0][0] - path[closest_point_idx][0])

    # 2. Calculate Heading Error (e_psi)
    heading_error = vehicle.normalize_angle(path_heading - vehicle.yaw)

    # 3. Calculate Cross-track Error (e_ct)
    # Vector from path point to front axle
    vec_path_to_front_axle_x = front_axle_x - target_point[0]
    vec_path_to_front_axle_y = front_axle_y - target_point[1]
    
    # Calculate e_ct using the sign to determine which side of the path the vehicle is on
    # A positive cross-track error means the vehicle is to the left of the path (when looking along path_heading)
    e_ct = np.sin(path_heading) * vec_path_to_front_axle_x - np.cos(path_heading) * vec_path_to_front_axle_y

    # 4. Stanley Control Law
    # delta = heading_error + arctan(k * e_ct / (k_soft + v))
    steer_angle = heading_error + np.arctan2(k * e_ct, (k_soft + vehicle.v))

    # Clamp steering angle to reasonable limits (e.g., -max_steer to +max_steer)
    max_steer = np.deg2rad(30) # Example max steering angle 30 degrees
    steer_angle = np.clip(steer_angle, -max_steer, max_steer)
    
    return steer_angle, closest_point_idx

def generate_circular_path(radius, center_x, center_y, num_points=100):
    """Generates a circular path."""
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False) # endpoint=False to avoid duplicate start/end point
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)
    
    # Add the first point at the end to close the loop for continuous path following
    return np.array(list(zip(x, y)) + [[x[0], y[0]]])

def main():
    # --- Simulation Parameters ---
    dt = 0.1  # Time step [s]
    simulation_time = 100.0  # Total simulation time [s]
    vehicle_speed = 2.0  # Constant vehicle speed [m/s]
    
    # --- Stanley Controller Gains ---
    k = 0.8  # Gain for cross-track error
    k_soft = 0.1 # Softening constant

    # --- Circular Path Parameters ---
    path_radius = 10.0  # Radius of the circular path
    path_center_x = 0.0
    path_center_y = 10.0
    num_path_points = 200 # More points for smoother path representation

    # --- Vehicle Initialization ---
    # Start slightly outside the circle, pointing towards the tangent of the circle
    initial_x = path_center_x + path_radius + 5.0
    initial_y = path_center_y
    initial_yaw = np.deg2rad(90) # Pointing upwards (tangent to circle if starting on the right)
    vehicle = Vehicle(x=initial_x, y=initial_y, yaw=initial_yaw, v=vehicle_speed)

    # --- Generate Path ---
    path = generate_circular_path(path_radius, path_center_x, path_center_y, num_path_points)

    # --- Simulation Loop ---
    time = 0.0
    history_x, history_y, history_yaw = [], [], []
    
    while time < simulation_time:
        steer_angle, closest_point_idx = stanley_steer_control(vehicle, path, k, k_soft)
        vehicle.update(dt, steer_angle)

        history_x.append(vehicle.x)
        history_y.append(vehicle.y)
        history_yaw.append(vehicle.yaw)

        time += dt

        # Optional: Plot in real-time (can slow down simulation)
        # plt.clf()
        # plt.plot(path[:, 0], path[:, 1], "r--", label="Target Path")
        # plt.plot(history_x, history_y, "b-", label="Vehicle Trajectory")
        # plt.plot(vehicle.x, vehicle.y, "go", markersize=8, label="Vehicle Current Pos")
        # # Plot vehicle orientation
        # vehicle_front_x = vehicle.x + vehicle.L * np.cos(vehicle.yaw)
        # vehicle_front_y = vehicle.y + vehicle.L * np.sin(vehicle.yaw)
        # plt.plot([vehicle.x, vehicle_front_x], [vehicle.y, vehicle_front_y], "g-", linewidth=2, label="Vehicle Heading")
        #
        # # Plot the closest point on path (which is what Stanley uses for e_ct calculation)
        # if closest_point_idx < len(path): # Ensure closest_point_idx is valid for plotting
        #    plt.plot(path[closest_point_idx, 0], path[closest_point_idx, 1], "kx", markersize=10, label="Closest Path Point")
        #
        # plt.xlabel("X Position (m)")
        # plt.ylabel("Y Position (m)")
        # plt.title(f"Stanley Path Following (Time: {time:.1f}s)")
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
    plt.title("Stanley Path Following (Circular Path)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()