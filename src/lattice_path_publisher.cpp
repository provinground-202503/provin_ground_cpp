#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h> // For quaternion conversions
#include <cmath>   // For M_PI, atan2, etc.
#include <vector>
#include <numeric> // For std::iota
#include <limits>  // For std::numeric_limits

// --- Constants ---
const double MAX_STEERING_ANGLE_DEG = 28.0; // degrees
const double MAX_STEERING_ANGLE_RAD = MAX_STEERING_ANGLE_DEG * M_PI / 180.0; // radians
const double MAX_VELOCITY_KMH = 10.0;      // km/h
const double MAX_VELOCITY_MS = MAX_VELOCITY_KMH / 3.6; // m/s
const double OBSTACLE_RADIUS = 0.3; // meters, as per request
const double OBSTACLE_INFLATION_RADIUS = 0.5; // 장애물 회피를 위한 여유 공간 (로봇 폭 고려)
const double PLANNING_HORIZON_METERS = 10.0; // 경로 계획을 고려할 전방 거리 (미터)

// --- Global Variables to store latest subscribed data ---
nav_msgs::Path current_local_path;
ackermann_msgs::AckermannDriveStamped current_vehicle_status;
geometry_msgs::Point current_utm_point;
visualization_msgs::MarkerArray current_obstacles;

// --- Callbacks for Subscribers ---
void localPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    current_local_path = *msg;
    // ROS_INFO_THROTTLE(1.0, "Received local path with %zu points.", current_local_path.poses.size());
}

void vehicleStatusCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    current_vehicle_status = *msg;
    // ROS_INFO_THROTTLE(1.0, "Received vehicle status: speed=%.2f m/s, steering=%.2f rad.",
    //          current_vehicle_status.drive.speed, current_vehicle_status.drive.steering_angle);
}

void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::Point utm_point = msg->pose.position;
    current_utm_point = utm_point;
    // ROS_INFO_THROTTLE(1.0, "Received UTM point: x=%.2f, y=%.2f.", current_utm_point.x, current_utm_point.y);
}

void obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    current_obstacles = *msg;
    // ROS_INFO_THROTTLE(1.0, "Received %zu obstacle markers.", current_obstacles.markers.size());
}

// --- Helper function to calculate distance between two points ---
double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// --- Main Lattice Planner Logic ---
// This function generates an avoidance path based on local path, current status, and obstacles.
nav_msgs::Path generateLatticePaths(
    const nav_msgs::Path& local_path,
    const ackermann_msgs::AckermannDriveStamped& vehicle_status,
    const geometry_msgs::Point& utm_point,
    const visualization_msgs::MarkerArray& obstacles
) {
    nav_msgs::Path avoid_path;
    avoid_path.header.frame_id = "map"; // Assuming UTM points are in 'map' frame
    avoid_path.header.stamp = ros::Time::now();

    // --- 1. Get Current Robot Pose (Position from UTM, Orientation from Local Path) ---
    geometry_msgs::Pose current_robot_pose;
    current_robot_pose.position = utm_point; // Robot's current position

    double robot_yaw = 0.0; // Default yaw
    if (local_path.poses.size() >= 2) {
        // Use the first two points of local_path to estimate current robot yaw.
        // Assuming the first point is closest to the robot's current UTM position.
        const geometry_msgs::Point& p1 = local_path.poses[0].pose.position;
        const geometry_msgs::Point& p2 = local_path.poses[1].pose.position;

        robot_yaw = atan2(p2.y - p1.y, p2.x - p1.x);
        // ROS_INFO_THROTTLE(1.0, "Estimated robot yaw from local path segment: %.2f rad (%.2f deg)", robot_yaw, robot_yaw * 180.0 / M_PI);
    } else if (!local_path.poses.empty()) {
        ROS_WARN_THROTTLE(1.0, "Local path has less than 2 points. Cannot accurately estimate yaw from path segment. Using default 0.0.");
    } else {
        ROS_WARN_THROTTLE(1.0, "Local path is empty. Cannot estimate robot yaw. Returning empty path.");
        return avoid_path; // Return empty path if no path to follow
    }
    current_robot_pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw);

    // Add current robot pose as the start of the avoid path
    geometry_msgs::PoseStamped start_pose_stamped;
    start_pose_stamped.header = avoid_path.header;
    start_pose_stamped.pose = current_robot_pose;
    avoid_path.poses.push_back(start_pose_stamped);

    // --- 2. Determine Planning Goal from Local Path ---
    // Find a goal point along the local path within a planning horizon.
    // This makes the planner reactive to closer obstacles but still aims for the original path.
    geometry_msgs::Point goal_point = local_path.poses.back().pose.position; // Default to end of local path

    // Find a point on the local path within PLANNING_HORIZON_METERS
    for (const auto& pose_stamped : local_path.poses) {
        if (calculateDistance(utm_point, pose_stamped.pose.position) < PLANNING_HORIZON_METERS) {
            goal_point = pose_stamped.pose.position;
        } else {
            // Once we pass the horizon, the last point found is our goal for this iteration
            break;
        }
    }
    
    // ROS_INFO_THROTTLE(1.0, "Planning goal set to (%.2f, %.2f)", goal_point.x, goal_point.y);


    // --- 3. Obstacle Detection and Avoidance Strategy (Simplified Lattice Planner Logic) ---
    bool obstacle_in_planning_horizon = false;
    geometry_msgs::Point closest_obstacle_pos;
    double min_dist_to_obstacle = std::numeric_limits<double>::max();

    for (const auto& marker : obstacles.markers) {
        if (marker.points.empty()) continue; // Skip empty markers

        geometry_msgs::Point obstacle_center = marker.points[0]; // Assuming single-point markers
        double dist_to_obstacle = calculateDistance(utm_point, obstacle_center);

        // Check if the obstacle is within the planning horizon and in front
        // A more robust check uses transforms to confirm "in front"
        double dx = obstacle_center.x - utm_point.x;
        double dy = obstacle_center.y - utm_point.y;
        double relative_angle = atan2(dy, dx) - robot_yaw;
        // Normalize angle to -PI to PI
        relative_angle = atan2(sin(relative_angle), cos(relative_angle));

        if (dist_to_obstacle < PLANNING_HORIZON_METERS && std::abs(relative_angle) < M_PI / 2.0) { // Obstacle is in front cone
            if (dist_to_obstacle - (OBSTACLE_RADIUS + OBSTACLE_INFLATION_RADIUS) < min_dist_to_obstacle) {
                min_dist_to_obstacle = dist_to_obstacle - (OBSTACLE_RADIUS + OBSTACLE_INFLATION_RADIUS);
                closest_obstacle_pos = obstacle_center;
                obstacle_in_planning_horizon = true;
            }
        }
    }

    if (obstacle_in_planning_horizon && min_dist_to_obstacle < 2.0) { // If closest obstacle is less than 2m away (after inflation)
        ROS_WARN_THROTTLE(0.5, "Close obstacle detected at (%.2f, %.2f), planning avoidance path. Distance to boundary: %.2f m",
                          closest_obstacle_pos.x, closest_obstacle_pos.y, min_dist_to_obstacle);

        // --- Simplified Lattice Planner: Generate a few candidate trajectories ---
        // In a full Lattice Planner, this would involve:
        // 1. Defining a set of motion primitives (e.g., straight, gentle left/right curves, sharp left/right curves).
        //    These primitives are pre-generated or calculated on-the-fly respecting max_steering_angle.
        // 2. Sampling a few "target end states" (x, y, yaw, speed) around the goal point.
        // 3. Generating a trajectory from current state to each target end state using motion primitives.
        // 4. Evaluating each trajectory using a cost function (collision, path deviation, smoothness, etc.).
        // 5. Selecting the lowest cost trajectory.

        // For this example, let's create a "steer left" and "steer right" path.
        // And compare them (conceptually) or just pick one.
        
        // Simple fixed-step path generation for demonstration
        int num_path_points = 20;
        double segment_length = 0.5; // meters per path segment
        
        // Try to avoid by steering slightly perpendicular to the obstacle
        // Determine which side of the local path the obstacle is on
        // This requires projecting the obstacle onto the path, or knowing path curvature.
        // For simplicity, let's just try to go to the "side" that is further from the obstacle initially
        // or just apply a fixed small steering angle
        
        double current_x = utm_point.x;
        double current_y = utm_point.y;
        double current_yaw_for_path = robot_yaw; // Use the estimated robot yaw
        double target_angular_velocity = 0.0; // rad/s, for steering

        // A very simple heuristic: if obstacle is relatively 'right' of current heading, steer left, else steer right.
        // (This needs better obstacle relative positioning logic)
        double obstacle_relative_y = sin(robot_yaw) * (closest_obstacle_pos.x - utm_point.x) - cos(robot_yaw) * (closest_obstacle_pos.y - utm_point.y);

        if (obstacle_relative_y > 0) { // Obstacle is to the "left" of current heading
            target_angular_velocity = -MAX_STEERING_ANGLE_RAD / 5.0; // Try to steer right (negative yaw change)
        } else { // Obstacle is to the "right" or directly in front
            target_angular_velocity = MAX_STEERING_ANGLE_RAD / 5.0; // Try to steer left (positive yaw change)
        }
        
        // Generate points for the avoidance path
        for (int i = 0; i < num_path_points; ++i) {
            geometry_msgs::PoseStamped next_pose_stamped;
            next_pose_stamped.header = avoid_path.header;

            // Simple integration for new pose based on current velocity and steering
            // This is a basic kinematic model, not a dynamic one
            double dt = segment_length / MAX_VELOCITY_MS; // Time to travel one segment
            
            // Limit steering angle to ensure it respects MAX_STEERING_ANGLE_RAD
            double steer_angle_for_segment = target_angular_velocity * dt;
            if (std::abs(steer_angle_for_segment) > MAX_STEERING_ANGLE_RAD) {
                steer_angle_for_segment = std::copysign(MAX_STEERING_ANGLE_RAD, steer_angle_for_segment);
            }

            current_yaw_for_path += steer_angle_for_segment; // Update yaw for next point

            next_pose_stamped.pose.position.x = current_x + segment_length * cos(current_yaw_for_path);
            next_pose_stamped.pose.position.y = current_y + segment_length * sin(current_yaw_for_path);
            next_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw_for_path);

            // Update current_x, current_y for next iteration
            current_x = next_pose_stamped.pose.position.x;
            current_y = next_pose_stamped.pose.position.y;

            avoid_path.poses.push_back(next_pose_stamped);

            // Add basic collision check for the generated point
            // This is a simplified check: check distance to all obstacles
            bool collision_with_this_point = false;
            for (const auto& marker : obstacles.markers) {
                if (marker.points.empty()) continue;
                geometry_msgs::Point obs_center = marker.points[0];
                if (calculateDistance(next_pose_stamped.pose.position, obs_center) < (OBSTACLE_RADIUS + OBSTACLE_INFLATION_RADIUS)) {
                    collision_with_this_point = true;
                    // ROS_WARN_THROTTLE(0.1, "Collision detected for generated path point! (%.2f, %.2f)", next_pose_stamped.pose.position.x, next_pose_stamped.pose.position.y);
                    break;
                }
            }
            if (collision_with_this_point) {
                // If collision, this path segment is bad.
                // In a real planner, you'd discard this trajectory and try another.
                // For this example, we just stop extending this path.
                ROS_WARN_THROTTLE(0.5, "Generated path segment led to collision. Shortening path.");
                break; 
            }

            // If we've reached near the goal point from local path, stop extending
            if (calculateDistance(next_pose_stamped.pose.position, goal_point) < 1.0) {
                // ROS_INFO_THROTTLE(1.0, "Reached near local path goal, stopping path generation.");
                break;
            }
        }
        // Ensure the path ends at the goal if possible, and it's not too short
        if (avoid_path.poses.empty() || calculateDistance(avoid_path.poses.back().pose.position, goal_point) > 0.5) {
            geometry_msgs::PoseStamped final_goal_pose_stamped;
            final_goal_pose_stamped.header = avoid_path.header;
            final_goal_pose_stamped.pose.position = goal_point;
            // Orient the final pose towards the goal from the last generated point
            if (!avoid_path.poses.empty()) {
                double dx_goal = goal_point.x - avoid_path.poses.back().pose.position.x;
                double dy_goal = goal_point.y - avoid_path.poses.back().pose.position.y;
                final_goal_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy_goal, dx_goal));
            } else { // If path somehow became empty
                 final_goal_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw);
            }
            avoid_path.poses.push_back(final_goal_pose_stamped);
        }

    } else {
        // No close obstacles or obstacles not in front. Follow the local path.
        // We'll simply copy the local_path to avoid_path.
        // This implicitly assumes local_path respects vehicle dynamics if no obstacles.
        ROS_INFO_THROTTLE(1.0, "No critical obstacles detected. Using local path as avoid_path.");
        avoid_path = local_path; // Directly assign if no avoidance needed
        
        // Ensure that the local path starts from the current robot's pose for seamless transition
        // Find the closest point in local_path to current_utm_point
        double min_dist_to_path_point = std::numeric_limits<double>::max();
        int closest_point_idx = -1;
        for (size_t i = 0; i < local_path.poses.size(); ++i) {
            double dist = calculateDistance(utm_point, local_path.poses[i].pose.position);
            if (dist < min_dist_to_path_point) {
                min_dist_to_path_point = dist;
                closest_point_idx = i;
            }
        }

        if (closest_point_idx != -1) {
            // Create a new path starting from current robot pose and appending the rest of local_path
            nav_msgs::Path aligned_local_path;
            aligned_local_path.header = local_path.header;
            aligned_local_path.poses.push_back(start_pose_stamped); // Add current robot pose

            // Append points from the closest_point_idx onwards
            for (size_t i = closest_point_idx; i < local_path.poses.size(); ++i) {
                aligned_local_path.poses.push_back(local_path.poses[i]);
            }
            avoid_path = aligned_local_path;
        } else {
            // Fallback if no closest point found (e.g., empty local_path)
            ROS_WARN_THROTTLE(1.0, "Could not find closest point on local path for alignment. Using original local_path.");
            avoid_path = local_path;
        }
    }

    return avoid_path;
}

// --- Main Function ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planner_node");
    ros::NodeHandle nh;

    ROS_INFO("Lattice Planner Node Started.");

    // --- Subscribers ---
    ros::Subscriber local_path_sub = nh.subscribe("/local_path", 1, localPathCallback);
    ros::Subscriber vehicle_status_sub = nh.subscribe("/vehicle_status", 1, vehicleStatusCallback);
    ros::Subscriber utm_sub = nh.subscribe("/utm", 1, utmCallback);
    ros::Subscriber obstacles_sub = nh.subscribe("/filtered_obstacles", 1, obstaclesCallback);

    // --- Publishers ---
    ros::Publisher avoid_path_pub = nh.advertise<nav_msgs::Path>("/avoid_path", 1);

    ros::Rate loop_rate(10); // Publish at 10 Hz

    while (ros::ok()) {
        // Only generate path if we have essential data
        if (!current_local_path.poses.empty() && current_obstacles.markers.empty() || // If local path exists, and no obstacles, or...
            !current_local_path.poses.empty() && !current_obstacles.markers.empty()) { // ...local path and obstacles exist
            
            nav_msgs::Path generated_avoid_path = generateLatticePaths(
                current_local_path,
                current_vehicle_status,
                current_utm_point,
                current_obstacles
            );

            // Publish the generated path
            avoid_path_pub.publish(generated_avoid_path);
        } else {
            ROS_WARN_THROTTLE(5.0, "Waiting for initial data (local_path and/or obstacles) to start planning.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}