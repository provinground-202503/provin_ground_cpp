#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <fstream>
#include <vector>
#include <string>

// CSV 파일에서 경로를 읽는 함수
bool loadPathFromCSV(const std::string& file_path, nav_msgs::Path& path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open file: %s", file_path.c_str());
        return false;
    }

    std::string line;
    // 헤더 라인 스킵 (필요 시)
    // std::getline(file, line); 

    path.poses.clear();
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> values;
        while (std::getline(ss, value, ',')) {
            try {
                values.push_back(std::stod(value));
            } catch (const std::invalid_argument& e) {
                ROS_WARN("Invalid number format in CSV: %s", value.c_str());
                continue;
            }
        }

        if (values.size() >= 2) { // x, y 좌표는 최소한 있어야 함
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "gps";
            pose.pose.position.x = values[0];
            pose.pose.position.y = values[1];
            // z 좌표가 있으면 사용, 없으면 0
            pose.pose.position.z = (values.size() >= 3) ? values[2] : 0.0;
            pose.pose.orientation.w = 1.0; // 오리엔테이션은 고정
            path.poses.push_back(pose);
        }
    }
    path.header.frame_id = "gps";
    path.header.stamp = ros::Time::now();
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_global_path_node");
    ros::NodeHandle nh;

    // 패키지 경로 가져오기
    std::string package_path = ros::package::getPath("provin_ground");
    if (package_path.empty()) {
        ROS_FATAL("Failed to get path for package 'my_package'.");
        return -1;
    }

    const int NUM_PATHS = 7;
    std::vector<ros::Publisher> path_pubs;
    std::vector<nav_msgs::Path> global_paths(NUM_PATHS);

    for (int i = 0; i < NUM_PATHS; ++i) {
        // 토픽 이름: /global_path/path_0, /global_path/path_1, ...
        std::string topic_name = "/global_path/path_" + std::to_string(i);
        path_pubs.push_back(nh.advertise<nav_msgs::Path>(topic_name, 1, true)); // latch=true

        // 파일 경로 설정
        std::string file_path = package_path + "/path/utm_fix_" + std::to_string(i + 1) + ".csv";
        
        ROS_INFO("Loading global path from: %s", file_path.c_str());
        if (loadPathFromCSV(file_path, global_paths[i])) {
            ROS_INFO("Successfully loaded path %d with %zu poses.", i, global_paths[i].poses.size());
        } else {
            ROS_ERROR("Failed to load path %d.", i);
        }
    }

    ros::Rate rate(1); // 1 Hz
    while(ros::ok()) {
        for(int i = 0; i < NUM_PATHS; ++i) {
            if (!global_paths[i].poses.empty()) {
                global_paths[i].header.stamp = ros::Time::now();
                path_pubs[i].publish(global_paths[i]);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}