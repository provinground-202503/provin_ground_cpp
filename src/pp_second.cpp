#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h> // Assuming lidar_clusters provide points directly
#include <sensor_msgs/PointCloud2.h> // Or other suitable message type for clusters

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>
#include <numeric> // For std::accumulate

// 웨이포인트 구조체 정의 (공통 헤더 파일로 분리하는 것이 이상적)
struct waypoint {
    double x;
    double y;
};

class LocalPathCalculator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber utm_sub_;
    ros::Subscriber clustered_sub_; // Lidar clusters subscriber
    ros::Publisher local_path_pub_; // Publisher for the calculated local path

    std::vector<waypoint> global_waypoint_vector_;
    std::vector<geometry_msgs::Point> current_clusters_; // Current obstacle clusters
    
    double curr_x_, curr_y_, yaw_;
    bool global_path_received_ = false;
    bool current_pose_received_ = false;

    // Local Path Calculation Parameters
    double local_path_lookahead_dist_; // How far to look along the global path for local path generation
    int num_waypoints_for_local_path_; // Number of waypoints to include in the local path
    double obstacle_avoidance_distance_; // Minimum distance to maintain from obstacles
    double safety_margin_; // Additional safety margin around obstacles
    double angle_increment_; // For DWA-like trajectory generation (angular resolution)
    double max_steering_angle_rad_; // Maximum steer angle for path generation

    /**
     * @brief 글로벌 경로를 수신합니다.
     */
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        global_waypoint_vector_.clear();
        for (const auto& pose_stamped : msg->poses) {
            global_waypoint_vector_.push_back({pose_stamped.pose.position.x, pose_stamped.pose.position.y});
        }
        ROS_INFO_ONCE("global waypoint subscribed. wapoints: %zu", global_waypoint_vector_.size());
        global_path_received_ = true;
    }

    /**
     * @brief UTM 좌표를 수신하여 로컬 좌표로 변환하고, 이전 위치와 비교하여 yaw를 계산합니다.
     * 이 노드에서는 현재 위치와 yaw만 업데이트합니다.
     */
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_x_ = msg->pose.position.x;
        curr_y_ = msg->pose.position.y;
        
        // Use orientation directly from PoseStamped if available and reliable,
        // otherwise, infer from position change (as in original code).
        // For accurate heading, IMU or direct pose estimation is better.
        // Assuming msg->pose.orientation contains reliable yaw.
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        double roll, pitch, yaw_from_quat;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_from_quat);
        yaw_ = yaw_from_quat;
        current_pose_received_ = true;
    }

    /**
     * @brief Lidar 클러스터 데이터를 수신합니다.
     * 이 데이터는 base_link 좌표계라고 가정합니다.
     */
    void clusteredCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        current_clusters_.clear();
        // Assuming PointCloud2 for clusters and iterating through its points
        // NOTE: This is a simplified conversion. Actual PointCloud2 parsing requires more effort.
        // For demonstration, let's assume it's just a flat array of X, Y, Z.
        // If your clusters are in a different message type (e.g., geometry_msgs::Point[] or custom),
        // adjust this part accordingly.
        
        // Example for PointCloud2 if it's dense and contains only x,y,z
        if (msg->data.empty()) return;

        // Simplified parsing for demonstration (assuming float32 x,y,z)
        // This is highly dependent on your PointCloud2 data structure (fields, endianness, etc.)
        // A robust way would involve sensor_msgs::readPointCloud() or pcl::fromROSMsg
        for (size_t i = 0; i < msg->data.size(); i += msg->point_step) {
            geometry_msgs::Point p;
            // Assuming x,y,z are float32 at the beginning of each point's data
            // You might need to adjust offset and data type based on msg->fields
            p.x = *reinterpret_cast<const float*>(&msg->data[i + msg->fields[0].offset]);
            p.y = *reinterpret_cast<const float*>(&msg->data[i + msg->fields[1].offset]);
            p.z = *reinterpret_cast<const float*>(&msg->data[i + msg->fields[2].offset]);
            
            // Transform obstacle point from base_link to map frame
            // Assuming current_clusters_ stores points in the map frame
            double obs_x_map = curr_x_ + p.x * std::cos(yaw_) - p.y * std::sin(yaw_);
            double obs_y_map = curr_y_ + p.x * std::sin(yaw_) + p.y * std::cos(yaw_);
            geometry_msgs::Point obs_point_map;
            obs_point_map.x = obs_x_map;   
            obs_point_map.y = obs_y_map;
            obs_point_map.z = p.z;
            current_clusters_.push_back(obs_point_map); // z is likely not used for 2D path planning
        }
        // ROS_INFO_THROTTLE(1.0, "장애물 클러스터 수신: %zu", current_clusters_.size());
    }

    /**
     * @brief 현재 위치를 기반으로 글로벌 경로에서 로컬 경로를 추출하고 장애물 회피를 적용합니다.
     */
    void calculateAndPublishLocalPath(const ros::TimerEvent&) {
        if (!global_path_received_ || !current_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "wait for global path or current pose.");
            return;
        }

        // 1. 현재 차량 위치에서 가장 가까운 글로벌 경로 웨이포인트 찾기
        int closest_global_index = -1;
        double min_dist_to_global = std::numeric_limits<double>::max();

        for (size_t i = 0; i < global_waypoint_vector_.size(); ++i) {
            double dx = curr_x_ - global_waypoint_vector_[i].x;
            double dy = curr_y_ - global_waypoint_vector_[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist_to_global) {
                min_dist_to_global = dist;
                closest_global_index = i;
            }
        }

        if (closest_global_index == -1) {
            ROS_ERROR("cannot find closest waypoint in global path.");
            return;
        }

        // 2. Look-ahead distance에 기반하여 로컬 경로 추출
        std::vector<waypoint> candidate_local_path;
        double accumulated_dist = 0.0;
        int current_local_path_points = 0;

        for (size_t i = closest_global_index; i < global_waypoint_vector_.size(); ++i) {
            if (i > closest_global_index) {
                accumulated_dist += std::sqrt(std::pow(global_waypoint_vector_[i].x - global_waypoint_vector_[i-1].x, 2) + 
                                               std::pow(global_waypoint_vector_[i].y - global_waypoint_vector_[i-1].y, 2));
            }
            candidate_local_path.push_back(global_waypoint_vector_[i]);
            current_local_path_points++;

            if (accumulated_dist >= local_path_lookahead_dist_ && current_local_path_points >= num_waypoints_for_local_path_) {
                break;
            }
        }

        if (candidate_local_path.empty()) {
            ROS_WARN("waypoint more needed for local path calculation.");
            // Stop condition for the vehicle if near end of global path
            nav_msgs::Path local_path_msg; // Publish empty path to signal stop
            local_path_msg.header.stamp = ros::Time::now();
            local_path_msg.header.frame_id = "map";
            local_path_pub_.publish(local_path_msg);
            return;
        }

        // 3. 장애물 회피 로직 (DWA-like & Safety Map)
        // 이 부분은 간소화된 DWA의 개념을 사용합니다.
        // 목표: candidate_local_path를 기반으로 장애물을 회피하는 최적의 경로를 찾습니다.
        // 여러 가능한 궤적을 시뮬레이션하고 평가하여 가장 좋은 것을 선택합니다.
        // 여기서는 기존 경로를 장애물로부터 밀어내는 방식을 사용하겠습니다.

        std::vector<waypoint> optimized_local_path = candidate_local_path;

        for (size_t i = 0; i < optimized_local_path.size(); ++i) {
            double current_waypoint_x = optimized_local_path[i].x;
            double current_waypoint_y = optimized_local_path[i].y;

            for (const auto& cluster_point : current_clusters_) {
                double dist_to_obstacle = std::sqrt(std::pow(current_waypoint_x - cluster_point.x, 2) +
                                                    std::pow(current_waypoint_y - cluster_point.y, 2));

                if (dist_to_obstacle < obstacle_avoidance_distance_ + safety_margin_) {
                    // 장애물로부터 너무 가깝습니다. 웨이포인트를 밀어냅니다.
                    // 간단한 회피 전략: 장애물로부터 멀어지는 방향으로 웨이포인트를 이동
                    double angle_to_obstacle = std::atan2(cluster_point.y - current_waypoint_y, cluster_point.x - current_waypoint_x);
                    double overlap = (obstacle_avoidance_distance_ + safety_margin_) - dist_to_obstacle;

                    // 회피 방향: 장애물에서 멀어지는 반대 방향으로 이동
                    double avoidance_angle = angle_to_obstacle + M_PI; // 180 degrees away
                    
                    // 회피 거리: 겹치는 만큼 + 작은 상수
                    double shift_x = overlap * std::cos(avoidance_angle);
                    double shift_y = overlap * std::sin(avoidance_angle);

                    optimized_local_path[i].x += shift_x;
                    optimized_local_path[i].y += shift_y;

                    // 이동 후, 차량의 최대 조향각 제약 (곡률)을 고려하여 경로가 너무 급격하게 변하지 않도록 조정할 수 있습니다.
                    // 이는 DWA의 '실현 가능한 궤적' 부분과 유사합니다.
                    // 여기서는 간단히 너무 큰 조향각이 발생하지 않도록 조향각 제한을 추가할 수 있습니다.
                    // 예를 들어, 새로운 웨이포인트가 이전 웨이포인트와 이루는 각도가 너무 크면 클램핑합니다.
                    if (i > 0) {
                        double dx_prev = optimized_local_path[i].x - optimized_local_path[i-1].x;
                        double dy_prev = optimized_local_path[i].y - optimized_local_path[i-1].y;
                        double path_segment_angle = std::atan2(dy_prev, dx_prev);

                        // 차량 현재 yaw와 경로 세그먼트 각도 차이
                        double angle_diff = path_segment_angle - yaw_;
                        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff)); // Normalize to -pi to pi

                        // 최대 조향각을 기반으로 경로의 곡률을 제한
                        // (Pure Pursuit에서 계산되는 steering_angle과 유사)
                        // Ld = segment length, L = car_length
                        // steering_angle = atan2(2*L*y, Ld*Ld)
                        // simplified: steering_angle ~ 2 * L * y / Ld^2
                        // max_y_local = max_steering_angle * Ld^2 / (2*L)
                        
                        double segment_length = std::sqrt(dx_prev*dx_prev + dy_prev*dy_prev);
                        if (segment_length > 0.0) {
                            double required_steer_rad = std::atan2(2.0 * 1.04 * (optimized_local_path[i].y - optimized_local_path[i-1].y), segment_length * segment_length);
                            if (std::abs(required_steer_rad) > max_steering_angle_rad_) {
                                // Clamp the waypoint to respect max steering angle
                                double clamped_steer_rad = std::copysign(max_steering_angle_rad_, required_steer_rad);
                                // This requires re-calculating the point, which is non-trivial.
                                // For simplicity, this is a conceptual placeholder.
                                // A full DWA would explore such trajectories.
                                // Instead, we can simply say if it requires too much steering,
                                // we might need to slow down or find an alternative path.
                                // For now, we just indicate it's a constraint to be aware of.
                            }
                        }
                    }
                }
            }
        }
        
        // 4. 로컬 경로 발행
        nav_msgs::Path local_path_msg;
        local_path_msg.header.stamp = ros::Time::now();
        local_path_msg.header.frame_id = "map"; // 로컬 경로도 map 프레임

        for (const auto& wp : optimized_local_path) {
            geometry_msgs::PoseStamped pose;
            pose.header = local_path_msg.header;
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = 0;
            local_path_msg.poses.push_back(pose);
        }
        local_path_pub_.publish(local_path_msg);
        ROS_INFO_THROTTLE(1.0, "local path published. waypoints: %zu", optimized_local_path.size());
    }

public:
    LocalPathCalculator() : nh_("~") {
        global_path_sub_ = nh_.subscribe("/global_path", 1, &LocalPathCalculator::globalPathCallback, this);
        utm_sub_ = nh_.subscribe("/utm_fix", 1, &LocalPathCalculator::utmCallback, this);
        clustered_sub_ = nh_.subscribe("/lidar_clusters", 1, &LocalPathCalculator::clusteredCallback, this);
        
        local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path_dwa", 1);
        
        // Parameters
        nh_.param("local_path_lookahead_dist", local_path_lookahead_dist_, 5.0); // 5 meters ahead
        nh_.param("num_waypoints_for_local_path", num_waypoints_for_local_path_, 50); // Up to 50 waypoints
        nh_.param("obstacle_avoidance_distance", obstacle_avoidance_distance_, 1.0); // Keep 1m from obstacles
        nh_.param("safety_margin", safety_margin_, 0.2); // Add 0.2m safety buffer
        nh_.param("angle_increment", angle_increment_, 0.1); // For DWA-like angle sampling (not fully used here)
        nh_.param("max_steering_angle_deg", max_steering_angle_rad_, 25.0); // Max steer in degrees
        max_steering_angle_rad_ = max_steering_angle_rad_ * M_PI / 180.0; // Convert to radians

        // Timer to trigger path calculation
        nh_.createTimer(ros::Duration(1.0 / 10.0), &LocalPathCalculator::calculateAndPublishLocalPath, this); // 10 Hz
    }
    ~LocalPathCalculator() = default;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_path_calculate");
    LocalPathCalculator lpc;
    ros::spin();
    return 0;
}