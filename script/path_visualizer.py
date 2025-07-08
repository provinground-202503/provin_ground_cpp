#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion

class PathVisualizer:
    def __init__(self):
        """
        Initializes the ROS node, subscribers, and matplotlib plot.
        """
        rospy.init_node('path_visualizer_node', anonymous=True)

        self.x_offset = rospy.get_param('~x_offset', -360777.923575)
        self.y_offset = rospy.get_param('~y_offset', -4065980.612646)
        rospy.loginfo(f"Using offsets: x={self.x_offset}, y={self.y_offset}")

        # Vehicle parameters (adjust if necessary)
        self.wheelbase = 2.0  # Vehicle wheelbase in meters

        # Data storage
        self.utm_x_history = []
        self.utm_y_history = []
        # === MODIFICATION START ===
        # History for predicted points (to accumulate red dots)
        self.pred_x_history = []
        self.pred_y_history = []
        # === MODIFICATION END ===

        self.current_pose = None
        self.current_ackermann_cmd = None

        # ROS Subscribers
        rospy.Subscriber("/utm", PoseStamped, self.utm_callback)
        rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_callback)

        # Setup matplotlib for live plotting
        plt.ion() 
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Vehicle Path Visualization")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        
        # Plot elements
        self.path_line, = self.ax.plot([], [], 'b-', label="Actual Path")
        self.current_pos_dot, = self.ax.plot([], [], 'bo', markersize=8, label="Current Position")
        # === MODIFICATION START ===
        # Changed style from 'r.-' to 'r.' to show only dots
        self.predicted_path_line, = self.ax.plot([], [], 'r.', markersize=4, label="Accumulated Predictions")
        # === MODIFICATION END ===

        self.ax.legend()

    def utm_callback(self, msg):
        """
        Callback function for the /utm topic. Applies offsets and stores the position.
        """
        msg.pose.position.x += self.x_offset
        msg.pose.position.y += self.y_offset
        
        self.current_pose = msg
        self.utm_x_history.append(msg.pose.position.x)
        self.utm_y_history.append(msg.pose.position.y)

    def ackermann_callback(self, msg):
        """
        Callback function for the /ackermann_cmd topic. Stores the current command.
        """
        self.current_ackermann_cmd = msg
    
    def get_yaw_from_quaternion(self, quaternion):
        """
        Converts a ROS Quaternion message to a yaw angle in radians.
        """
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def predict_path(self):
        """
        Calculates a short future trajectory. This function uses the offset-corrected
        current_pose, so no changes are needed here.
        """
        if not self.current_pose or not self.current_ackermann_cmd:
            return [], []

        x_pred = []
        y_pred = []

        x_current = self.current_pose.pose.position.x
        y_current = self.current_pose.pose.position.y
        yaw_current = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        
        speed = self.current_ackermann_cmd.drive.speed
        steering_angle = self.current_ackermann_cmd.drive.steering_angle
        
        # === MODIFICATION START ===
        dt = 0.1
        prediction_horizon = 2 # Predict just one step ahead
        # === MODIFICATION END ===

        # Simple Ackermann kinematic model for prediction
        for i in range(prediction_horizon):
            x_next = x_current + speed * np.cos(yaw_current) * dt
            y_next = y_current + speed * np.sin(yaw_current) * dt
            yaw_next = yaw_current + (speed / self.wheelbase) * np.tan(steering_angle) * dt
            
            x_pred.append(x_next)
            y_pred.append(y_next)
            
            x_current, y_current, yaw_current = x_next, y_next, yaw_next
            
        return x_pred, y_pred

    def run_visualizer(self):
        """
        Main loop to update and draw the plot.
        """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.current_pose:
                # Update actual path (blue line)
                self.path_line.set_data(self.utm_x_history, self.utm_y_history)

                # Update current position (blue dot)
                current_x = self.current_pose.pose.position.x
                current_y = self.current_pose.pose.position.y
                self.current_pos_dot.set_data([current_x], [current_y])
                
                # === MODIFICATION START ===
                # Get new prediction and add it to the history
                pred_x, pred_y = self.predict_path()
                if pred_x: # Ensure prediction is not empty
                    self.pred_x_history.extend(pred_x)
                    self.pred_y_history.extend(pred_y)
                
                # Update the accumulated predicted path (red dots)
                self.predicted_path_line.set_data(self.pred_x_history, self.pred_y_history)
                # === MODIFICATION END ===

                # Adjust plot limits and redraw
                self.ax.relim()
                self.ax.autoscale_view()
                
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = PathVisualizer()
        visualizer.run_visualizer()
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.ioff()
        plt.show()