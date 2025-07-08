#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>

// 웨이포인트 구조체 정의
struct waypoint {
    double x;
    double y;
};

class GlobalPathPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Timer publish_timer_;

    std::vector<waypoint> waypoint_vector_;
    double utm_offset_x_;
    double utm_offset_y_;

    /**
     * @brief CSV 파일에서 웨이포인트를 로드하고 UTM 오프셋을 적용합니다.
     */
    void loadWaypoints() {
        std::string file_path = "/root/erp42_ws/src/provin_ground_cpp/path/path_morai_xy.csv";
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_FATAL("cannot open file: %s", file_path.c_str());
            ros::shutdown();
            return;
        }

        waypoint_vector_.clear();
        std::string line;
        std::getline(file, line); // 헤더 라인 건너뛰기

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
        ROS_INFO("%zu waypoints loaded from local origin.", waypoint_vector_.size());
    }

    /**
     * @brief 로드된 경로를 Rviz 등에서 시각화하기 위해 발행합니다.
     */
    void publishPath(const ros::TimerEvent&) {
        if (waypoint_vector_.empty()) {
            ROS_WARN_THROTTLE(1.0, "no waypoints loaded.");
            return;
        }

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map"; // Global path in the map frame

        for (const auto& waypoint : waypoint_vector_) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = waypoint.x;
            pose.pose.position.y = waypoint.y;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);
        ROS_INFO_THROTTLE(5.0, "global path publishing.");
    }

public:
    GlobalPathPublisher() : nh_("~") {
        utm_offset_x_ = 0; // Set your UTM offsets here if needed
        utm_offset_y_ = 0;

        loadWaypoints();
        path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1, true); // Latched topic
        publish_timer_ = nh_.createTimer(ros::Duration(1.0), &GlobalPathPublisher::publishPath, this); // Publish periodically
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_path_publisher");
    GlobalPathPublisher gpp;
    ros::spin();
    return 0;
}