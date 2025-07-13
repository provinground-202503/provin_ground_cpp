#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <yolo/YoloDetectionArray.h> // [변경] YOLO 메시지 헤더

#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

class NetworkYoloPathNode {
public:
    NetworkYoloPathNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // === 파라미터 로드 ===
        private_nh.param("num_paths", num_paths_, 7);
        private_nh.param("local_path_size", local_path_size_, 20);
        private_nh.param("obstacle_safety_dist", safety_dist_, 1.0);
        private_nh.param("camera_image_width", camera_image_width_, 640); // [추가] YOLO가 사용하는 카메라 이미지 너비 (픽셀)

        // 비용 함수 가중치
        private_nh.param("w_obs", w_obs_, 1000.0);
        private_nh.param("w_cen", w_cen_, 0.5);
        private_nh.param("w_trans", w_trans_, 1.0);
        private_nh.param("w_lat", w_lat_, 2.0); // [추가] YOLO 측면 회피 비용 가중치

        local_paths_.resize(num_paths_);
        object_horizontal_position_ = 0.0; // [추가] 객체 위치 없음(0)으로 초기화
        
        // === 구독자 (Subscribers) ===
        obstacle_sub_ = nh.subscribe("/filtered_obstacles", 1, &NetworkYoloPathNode::obstacleCallback, this);
        yolo_detections_sub_ = nh.subscribe("/yolo_detections", 1, &NetworkYoloPathNode::yoloDetectionsCallback, this); // [변경]

        // 7개의 로컬 경로 구독
        for (int i = 0; i < num_paths_; ++i) {
            std::string sub_topic = "/local_path/path_" + std::to_string(i);
            local_path_subs_.push_back(nh.subscribe<nav_msgs::Path>(sub_topic, 1, 
                [this, i](const nav_msgs::Path::ConstPtr& msg){ this->localPathCallback(msg, i); }));
        }

        // === 발행자 (Publisher) ===
        final_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);
    }

    void run() {
        ros::Rate rate(10); // 10Hz
        while(ros::ok()) {
            bool all_paths_received = true;
            for(const auto& path : local_paths_) {
                if(path.poses.empty()) {
                    all_paths_received = false;
                    break;
                }
            }
            if (all_paths_received) {
                generateAndPublishFinalPath();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // [변경] YOLO 탐지 결과 콜백 함수
    void yoloDetectionsCallback(const yolo::YoloDetectionArray::ConstPtr& msg) {
        object_horizontal_position_ = 0.0; // 매번 초기화

        for (const auto& det : msg->detections) {
            if (det.class_name == "barrel" || det.class_name == "person") {
                // 객체의 중심 x 픽셀 좌표 계산
                double center_x = static_cast<double>(det.x) + static_cast<double>(det.width) / 2.0;
                
                // 중심 x좌표를 -1.0 (왼쪽) ~ +1.0 (오른쪽) 범위로 정규화
                object_horizontal_position_ = (center_x / camera_image_width_ - 0.5) * 2.0;

                // 가장 먼저 탐지된 주요 객체 하나만 사용
                break; 
            }
        }
    }

    void localPathCallback(const nav_msgs::Path::ConstPtr& msg, int path_idx) {
        local_paths_[path_idx] = *msg;
    }

    void obstacleCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        obstacles_.clear();
        for (const auto& marker : msg->markers) {
            obstacles_.push_back(marker.pose.position);
        }
    }
    
    double distSq(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return dx * dx + dy * dy;
    }

    void generateAndPublishFinalPath() {
        nav_msgs::Path final_path;
        final_path.header.stamp = ros::Time::now();
        final_path.header.frame_id = "gps";
        
        geometry_msgs::PoseStamped prev_selected_pose;

        for (int i = 0; i < local_path_size_; ++i) {
            std::vector<geometry_msgs::Point> candidate_points;
            bool all_paths_long_enough = true;
            for(int j=0; j<num_paths_; ++j) {
                if (local_paths_[j].poses.size() > i) {
                    candidate_points.push_back(local_paths_[j].poses[i].pose.position);
                } else {
                    all_paths_long_enough = false;
                    break;
                }
            }
            if (!all_paths_long_enough) break; 

            double min_cost = std::numeric_limits<double>::max();
            int best_candidate_idx = -1;

            geometry_msgs::Point median_point;
            std::vector<double> x_coords, y_coords;
            for(const auto& pt : candidate_points) {
                x_coords.push_back(pt.x);
                y_coords.push_back(pt.y);
            }
            std::sort(x_coords.begin(), x_coords.end());
            std::sort(y_coords.begin(), y_coords.end());
            median_point.x = x_coords[x_coords.size() / 2];
            median_point.y = y_coords[y_coords.size() / 2];

            for (int j = 0; j < candidate_points.size(); ++j) {
                const auto& pt = candidate_points[j];
                
                // 1. 장애물 비용 (Marker 기반)
                double obs_cost = 0.0;
                for (const auto& obs_pt : obstacles_) {
                    if (distSq(pt, obs_pt) < safety_dist_ * safety_dist_) {
                        obs_cost = std::numeric_limits<double>::max();
                        break;
                    }
                }
                if (obs_cost == std::numeric_limits<double>::max()) continue;

                // 2. 중앙 유지 비용
                double cen_cost = std::sqrt(distSq(pt, median_point));

                // 3. 전환 비용 (경로 부드러움)
                double trans_cost = (i > 0) ? std::sqrt(distSq(pt, prev_selected_pose.pose.position)) : 0.0;
                
                // 4. YOLO 기반 측면 회피 비용 [핵심 로직]
                double lateral_avoidance_cost = 0.0;
                if (object_horizontal_position_ != 0.0) {
                     // 경로 인덱스 j를 -1 (좌) ~ +1 (우) 범위로 정규화
                     double path_norm_pos = (static_cast<double>(j) / (num_paths_ - 1) - 0.5) * 2.0;
                     
                     // 객체가 있는 방향으로 가는 경로에 페널티(양의 비용)를 부여
                     // 반대 방향으로 가는 경로에는 보상(음의 비용)을 부여
                     lateral_avoidance_cost = path_norm_pos * object_horizontal_position_;
                }

                // --- 총 비용 계산 ---
                double total_cost = w_obs_ * obs_cost + w_cen_ * cen_cost + w_trans_ * trans_cost + w_lat_ * lateral_avoidance_cost;

                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_candidate_idx = j;
                }
            }

            if (best_candidate_idx != -1) {
                geometry_msgs::PoseStamped new_pose;
                new_pose.header = final_path.header;
                new_pose.pose.position = candidate_points[best_candidate_idx];
                new_pose.pose.orientation.w = 1.0;
                
                final_path.poses.push_back(new_pose);
                prev_selected_pose = new_pose;
            } else {
                ROS_WARN("Could not find a safe path at index %d. Stopping path generation.", i);
                break;
            }
        }
        
        if (!final_path.poses.empty()) {
            final_path_pub_.publish(final_path);
        }
    }

    // ROS 핸들
    ros::Subscriber obstacle_sub_;
    ros::Subscriber yolo_detections_sub_; // [변경]
    std::vector<ros::Subscriber> local_path_subs_;
    ros::Publisher final_path_pub_;

    // 데이터 저장 변수
    std::vector<nav_msgs::Path> local_paths_;
    std::vector<geometry_msgs::Point> obstacles_;
    double object_horizontal_position_; // [변경] 객체의 정규화된 수평 위치 (-1.0 ~ 1.0)

    // 파라미터
    int num_paths_;
    int local_path_size_;
    int camera_image_width_; // [추가]
    double safety_dist_;
    double w_obs_, w_cen_, w_trans_, w_lat_; // [변경]
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "network_yolo_path_node");
    NetworkYoloPathNode node;
    node.run();
    return 0;
}