#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

class NetworkAvoidPathNode {
public:
    NetworkAvoidPathNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // 파라미터 로드
        private_nh.param("num_paths", num_paths_, 7);
        private_nh.param("local_path_size", local_path_size_, 20);
        private_nh.param("obstacle_safety_dist", safety_dist_, 1.0);
        // 비용 함수 가중치
        private_nh.param("w_obs", w_obs_, 1000.0);
        private_nh.param("w_cen", w_cen_, 0.5);
        private_nh.param("w_trans", w_trans_, 1.0);

        local_paths_.resize(num_paths_);
        
        // 구독자
        obstacle_sub_ = nh.subscribe("/filtered_obstacles", 1, &NetworkAvoidPathNode::obstacleCallback, this);
        for (int i = 0; i < num_paths_; ++i) {
            std::string sub_topic = "/local_path/path_" + std::to_string(i);
            local_path_subs_.push_back(nh.subscribe<nav_msgs::Path>(sub_topic, 1, 
                [this, i](const nav_msgs::Path::ConstPtr& msg){ this->localPathCallback(msg, i); }));
        }

        // 발행자
        final_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);
    }

    void run() {
        ros::Rate rate(10); // 10Hz
        while(ros::ok()) {
            // 모든 로컬 경로가 한번 이상 수신되었는지 확인
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
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg, int path_idx) {
        local_paths_[path_idx] = *msg;
    }

    void obstacleCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        obstacles_.clear();
        for (const auto& marker : msg->markers) {
            obstacles_.push_back(marker.pose.position);
        }
    }
    
    // 유클리드 거리 제곱 계산
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

        for (int i = 0; i < local_path_size_; ++i) { // 각 인덱스(깊이)에 대해
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

            if (!all_paths_long_enough) break; // 하나의 경로라도 끝나면 탐색 중지

            double min_cost = std::numeric_limits<double>::max();
            int best_candidate_idx = -1;

            // 중앙값 계산을 위한 중간 지점 찾기
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

            for (int j = 0; j < candidate_points.size(); ++j) { // 7개 후보 지점
                const auto& pt = candidate_points[j];
                
                // 1. 장애물 비용
                double obs_cost = 0.0;
                for (const auto& obs_pt : obstacles_) {
                    double d_sq = distSq(pt, obs_pt);
                    if (d_sq < safety_dist_ * safety_dist_) {
                        obs_cost = std::numeric_limits<double>::max(); // 안전거리 침범 시 비용 무한대
                        break;
                    }
                }
                if (obs_cost == std::numeric_limits<double>::max()) { // 이미 비용이 무한대면 더 계산할 필요 없음
                    continue;
                }

                // 2. 중앙 유지 비용
                double cen_cost = std::sqrt(distSq(pt, median_point));

                // 3. 전환 비용 (이전 선택 지점과의 거리)
                double trans_cost = 0.0;
                if (i > 0) {
                    trans_cost = std::sqrt(distSq(pt, prev_selected_pose.pose.position));
                }

                // 총 비용 계산
                double total_cost = w_obs_ * obs_cost + w_cen_ * cen_cost + w_trans_ * trans_cost;

                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_candidate_idx = j;
                }
            }

            if (best_candidate_idx != -1) {
                geometry_msgs::PoseStamped new_pose;
                new_pose.header.frame_id = "gps";
                new_pose.header.stamp = ros::Time::now();
                new_pose.pose.position = candidate_points[best_candidate_idx];
                new_pose.pose.orientation.w = 1.0;
                
                final_path.poses.push_back(new_pose);
                prev_selected_pose = new_pose;
            } else {
                // 안전한 경로를 찾지 못함
                ROS_WARN("Could not find a safe path at index %d. Stopping path generation.", i);
                break;
            }
        }
        
        if (!final_path.poses.empty()) {
            final_path_pub_.publish(final_path);
        }
    }

    ros::Subscriber obstacle_sub_;
    std::vector<ros::Subscriber> local_path_subs_;
    ros::Publisher final_path_pub_;

    std::vector<nav_msgs::Path> local_paths_;
    std::vector<geometry_msgs::Point> obstacles_;
    
    int num_paths_;
    int local_path_size_;
    double safety_dist_;
    double w_obs_, w_cen_, w_trans_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "network_avoid_path_node");
    NetworkAvoidPathNode node;
    node.run();
    return 0;
}