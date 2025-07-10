#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <limits> // For std::numeric_limits

// Waypoint structure definition
struct waypoint {
    double x;
    double y;
};

class GlobalPathPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    std::vector<waypoint> waypoint_vector_;

    // UTM coordinate offsets (can be loaded from parameter server if needed)
    double utm_offset_x_;
    double utm_offset_y_;

    /**
     * @brief Loads waypoints from a CSV file and applies UTM offsets.
     */
    void loadWaypoints() {
        std::string file_path = "/root/erp42_ws/src/provin_ground_cpp/path/pathmaker_path_xy.csv";
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_FATAL("Failed to open path file: %s", file_path.c_str());
            ros::shutdown();
            return;
        }

        waypoint_vector_.clear();
        std::string line;
        std::getline(file, line); // Skip header line

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            waypoint guide;

            std::getline(ss, token, ',');
            guide.x = std::stod(token) - utm_offset_x_;
            std::getline(ss, token, ',');
            guide.y = std::stod(token) - utm_offset_y_;
            
            waypoint_vector_.push_back(guide);
        }
        file.close();
        ROS_INFO("%zu waypoints loaded relative to local origin.", waypoint_vector_.size());
    }

    /**
     * @brief Publishes the loaded path for visualization in Rviz.
     */
    void publishPath() {
        if (waypoint_vector_.empty()) return;

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        for (const auto& wp : waypoint_vector_) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);
    }

public:
    GlobalPathPublisher() : nh_("~") {
        // Initialize UTM offsets (can be set to 0 if path is already local)
        // utm_offset_x_ = 360777.923575;
        // utm_offset_y_ = 4065980.612646;
        utm_offset_x_ = 0.0;
        utm_offset_y_ = 0.0;

        loadWaypoints();
        path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1, true); // Advertise as /global_path
        publishPath(); // Publish once at startup, or regularly if path can change
        
        // You might want to publish the path periodically if it's dynamic
        // Or just once if it's a static path. For a static path, publishing once is usually enough.
        // ros::Timer publish_timer_ = nh_.createTimer(ros::Duration(1.0), &GlobalPathPublisher::publishPath, this);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_path_publisher");
    GlobalPathPublisher gpp;
    ros::spin();
    return 0;
}