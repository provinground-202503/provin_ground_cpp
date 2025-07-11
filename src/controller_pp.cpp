#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <yolo/YoloDetectionArray.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2::doTransform if needed, or just for clarity

#include <vector>
#include <cmath>
#include <string>
#include <fstream> // Not strictly needed here if path is subscribed
#include <sstream> // Not strictly needed here if path is subscribed
#include <iomanip>
#include <limits>
#include <algorithm>

// Waypoint structure definition (needs to be consistent or included)
struct waypoint {
    double x;
    double y;
};

class PurePursuitController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber utm_sub_;
    ros::Subscriber erp_status_sub_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber global_path_sub_; // New subscriber for global path

    ros::Subscriber start_signal_sub_;
    ros::Publisher erp_cmd_pub_;
    ros::Publisher ack_pub_;
    ros::Timer control_timer_;

    // Path and vehicle state
    std::vector<waypoint> waypoint_vector_; // Will be populated by global_path_sub_
    int target_index_;
    double curr_x_, curr_y_, yaw_;
    double prev_x_, prev_y_; // Variables to store previous position for yaw calculation
    
    // Pure Pursuit parameters
    double car_length_;
    double look_ahead_distance_;

    // ERP status parameters
    double current_speed_erp_, current_speed_mps_, current_speed_kph_;
    double current_steer_erp_, current_steer_rad_, current_steer_deg_;
    double erp_status_dt_; // Time difference between status updates
    double prev_error_mps_, error_integral_;
    double yolo_speed_mps_, MAX_SPEED_MPS_;

    uint8_t emergency_brake_;
    bool START_SIGNAL_;

    /**
     * @brief Callback for receiving the global path.
     */
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoint_vector_.clear();
        for (const auto& pose_stamped : msg->poses) {
            waypoint wp;
            wp.x = pose_stamped.pose.position.x;
            wp.y = pose_stamped.pose.position.y;
            waypoint_vector_.push_back(wp);
        }
        ROS_INFO_ONCE("Global path received with %zu waypoints.", waypoint_vector_.size());
    }

    /**
     * @brief Subscribes to UTM coordinates, converts to local and calculates yaw.
     */
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Assuming /utm provides already offset local coordinates or handles offset internally if global_path_publisher also used offsets.
        // If /utm provides global coordinates, you would need to apply the same offset here as in global_path_publisher.
        curr_x_ = msg->pose.position.x;
        curr_y_ = msg->pose.position.y;
        
        // Calculate delta for yaw calculation
        double delta_x = curr_x_ - prev_x_;
        double delta_y = curr_y_ - prev_y_;

        // Update yaw only if vehicle has moved a certain distance (e.g., 1 cm)
        if (std::sqrt(delta_x * delta_x + delta_y * delta_y) > 0.01) {
            yaw_ = std::atan2(delta_y, delta_x);
        }

        // Store current position for next iteration
        prev_x_ = curr_x_;
        prev_y_ = curr_y_;
    }

    void erpStatusCallback(const erp_driver::erpStatusMsg::ConstPtr& msg) {
        current_speed_erp_ = msg->speed;
        current_steer_erp_ = msg->steer;

        current_speed_kph_ = current_speed_erp_ / 10.0;
        current_speed_mps_ = current_speed_kph_ / 3.6;

        current_steer_deg_ = static_cast<double>(current_steer_erp_) / 71.0;
        current_steer_rad_ = current_steer_deg_ * M_PI / 180.0;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw_imu; // Use yaw_imu to avoid conflict with class member yaw_
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_imu);
        
        // Set emergency brake based on pitch angle
        // if(pitch >= std::tan(0.05)) { // Adjust threshold as needed for incline detection
        //     emergency_brake_ = 20; // Aggressive brake
        // } else {
        //     emergency_brake_ = 1; // Minimal brake
        // }
    }

    void yoloCallback(const yolo::YoloDetectionArray::ConstPtr& msg) {
        bool redlight, yellowlight, greenlight;
        redlight =false;
        yellowlight=false;
        greenlight=true;
        for(auto& detection : msg->detections){
            if(detection.class_name=="redlight"){
                redlight=true;
                break;
            }
            else if(detection.class_name=="yellowlight"){
                yellowlight=true;
                break;
            }
            else if(detection.class_name=="greenlight"){
                greenlight=true;
            }
        }
        if(redlight || yellowlight){
            yolo_speed_mps_=0.0;
            return;
        }
        else{
            yolo_speed_mps_=MAX_SPEED_MPS_;
            return;
        }
    }
    
    void startSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
        START_SIGNAL_ = msg->data;
    }
    
    /**
     * @brief Control loop executed periodically.
     */
    void controlLoop(const ros::TimerEvent&) {
        if (waypoint_vector_.empty() || std::isnan(curr_x_) || std::isnan(curr_y_)) {
            ROS_WARN_THROTTLE(1.0, "Path is empty or current position is unknown.");
            publishCommands(0.0, 0.0); // Stop the vehicle if no path or position
            return;
        }

        // 1. Find the closest waypoint to the vehicle
        int closest_index = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < waypoint_vector_.size(); ++i) {
            double dx = curr_x_ - waypoint_vector_[i].x;
            double dy = curr_y_ - waypoint_vector_[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }

        // Adjust look-ahead distance based on path segment (example for S-course)
        if(closest_index != -1 && closest_index >= 920 && closest_index <= 1060) {
            look_ahead_distance_ = 1.05; // Shorter look-ahead for tight turns
        } else {
            look_ahead_distance_ = 3.0; // Default look-ahead
        }

        // 2. Search for the target point satisfying the look-ahead distance, starting from the closest point
        target_index_ = -1;
        for (size_t i = closest_index; i < waypoint_vector_.size(); ++i) {
            double dx_target = curr_x_ - waypoint_vector_[i].x;
            double dy_target = curr_y_ - waypoint_vector_[i].y;
            double dist_to_vehicle = std::sqrt(dx_target * dx_target + dy_target * dy_target);

            if (dist_to_vehicle >= look_ahead_distance_) {
                target_index_ = i;
                break;
            }
        }
        
        // If no target point found (end of path), stop the vehicle
        if (target_index_ == -1) {
            ROS_INFO("Reached end of path. Stopping vehicle.");
            publishCommands(0.0, 0.0);
            return;
        }

        // 3. Transform the target point to the vehicle's local coordinate frame (base_link)
        waypoint target_waypoint = waypoint_vector_[target_index_];
        double target_x_local = (target_waypoint.x - curr_x_) * std::cos(yaw_) + (target_waypoint.y - curr_y_) * std::sin(yaw_);
        double target_y_local = -(target_waypoint.x - curr_x_) * std::sin(yaw_) + (target_waypoint.y - curr_y_) * std::cos(yaw_);

        // 4. Calculate steering angle using the transformed coordinates
        double ld_sq = target_x_local * target_x_local + target_y_local * target_y_local;
        double steering_angle = std::atan2(2.0 * car_length_ * target_y_local, ld_sq);
        
        double target_speed_mps = yolo_speed_mps_; // Desired speed
        publishCommands(target_speed_mps, steering_angle);
        
        ROS_INFO("Target Index: %d, Steering Angle (rad): %.2f", target_index_, steering_angle);
    }
    
    /**
     * @brief Publishes Ackermann and ERP42 control messages.
     */
    void publishCommands(double target_speed_mps, double steer_angle) {
        if(!START_SIGNAL_) {
            std::cout<<"start signal is false\n";
            return;
        }
        // PID controller for longitudinal speed
        double p_gain = 0.7;
        double i_gain = 0.0;
        double d_gain = 0.005;
        double error_speed = target_speed_mps - current_speed_mps_;
        std::cout<<"error_speed: "<<error_speed<<std::endl;
        double output_mps_ = 0;
        if(error_speed <=0){
            target_speed_mps = 0;
            output_mps_ = 0;
            emergency_brake_ = static_cast<uint8_t>(32); // 0.05m/s margin
        }
        else {
            double p_term = p_gain * error_speed;
            error_integral_ += error_speed * erp_status_dt_;
            double i_term = i_gain * error_integral_;
            double d_term = d_gain * (error_speed - prev_error_mps_) / erp_status_dt_;
            prev_error_mps_ = error_speed;
            output_mps_ = p_term + i_term + d_term;
            emergency_brake_ = static_cast<uint8_t>(1);
        }

        // Publish AckermannDriveStamped message (for visualization or other nodes)
        ackermann_msgs::AckermannDriveStamped ack_msg;
        ack_msg.header.stamp = ros::Time::now();
        ack_msg.header.frame_id = "base_link";
        ack_msg.drive.steering_angle = steer_angle;
        ack_msg.drive.speed = output_mps_; // Use PID output for speed
        ack_pub_.publish(ack_msg);

        // Publish ERP42 control command
        erp_driver::erpCmdMsg cmd_msg;
        cmd_msg.e_stop = false;
        // cmd_msg.gear = (target_speed_mps == 0) ? 1 : 0; // 0 for forward, 1 for stop (neutral/brake)
        cmd_msg.gear = 0;
        // if (emergency_brake_ >1){
        //     cmd_msg.speed = 0;
        // }
        // else if (current_speed_mps_ > target_speed_mps && target_speed_mps > 0.1) { // If overshooting speed, reduce speed to target
        //     cmd_msg.speed = static_cast<uint8_t>(target_speed_mps * 3.6 * 10.0);
        // } else {
        //     cmd_msg.speed = static_cast<uint8_t>(output_mps_ * 3.6 * 10.0); // Convert m/s to ERP42 speed unit (0-200)
        // }
        
        // // Apply brake based on target speed and emergency brake status
        // cmd_msg.brake = (target_speed_mps == 0) ? 20 : emergency_brake_; // Full brake if target speed is 0, else emergency brake status

        if(current_speed_mps_>target_speed_mps){cmd_msg.speed=0;}
        else cmd_msg.speed = static_cast<uint8_t>(output_mps_ * 3.6 * 10.0);
        cmd_msg.brake = emergency_brake_;

        double steering_angle_degree = steer_angle * 180.0 / M_PI;
        // ERP42 steer command is usually inverted and scaled
        cmd_msg.steer = -static_cast<int32_t>(steering_angle_degree * 71.0); 

        // Clamp steer command to valid range
        cmd_msg.steer = std::max(-2000, std::min(2000, (int)cmd_msg.steer));
        erp_cmd_pub_.publish(cmd_msg);
        std::cout << "Steering Angle (rad): " << steer_angle << ", ERP Angle: " << cmd_msg.steer << std::endl;
    }

    /**
     * @brief Class initialization.
     */
    void initialize() {
        target_index_ = 0;
        car_length_ = 1.04; // Wheelbase of ERP42
        look_ahead_distance_ = 3.0; // Initial look-ahead distance
        
        // Initialize current and previous positions and yaw
        curr_x_ = 0.0;
        curr_y_ = 0.0;
        prev_x_ = 0.0;
        prev_y_ = 0.0;
        yaw_ = 0.0;
        
        erp_status_dt_ = 0.025; // Assuming 40Hz ERP status updates (1/40 = 0.025)
        error_integral_ = 0.0;
        prev_error_mps_ = 0.0;
        emergency_brake_ = 1; // Default brake value (minimal)

        MAX_SPEED_MPS_ = 2.0;

        yolo_speed_mps_ = MAX_SPEED_MPS_;
        START_SIGNAL_ = false;
    }

public:
    PurePursuitController() : nh_("~") {
        // Subscribers
        utm_sub_ = nh_.subscribe("/utm_fix", 1, &PurePursuitController::utmCallback, this);
        erp_status_sub_ = nh_.subscribe("/erp42_status", 1, &PurePursuitController::erpStatusCallback, this);
        yolo_sub_ = nh_.subscribe("/yolo_detections",1,&PurePursuitController::yoloCallback,this);
        imu_sub_ = nh_.subscribe("/imu_fix", 1, &PurePursuitController::imuCallback, this);
        global_path_sub_ = nh_.subscribe("/avoid_path", 1, &PurePursuitController::globalPathCallback, this); // Subscribe to the new global path topic
        start_signal_sub_ = nh_.subscribe("/erp42_start", 1, &PurePursuitController::startSignalCallback, this);


        // Publishers
        erp_cmd_pub_ = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd", 1);
        ack_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
        
        // Control loop timer (20 Hz)
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &PurePursuitController::controlLoop, this);
        
        initialize();
    }
    ~PurePursuitController() = default;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    PurePursuitController ppc;
    ros::spin();
    return 0;
}