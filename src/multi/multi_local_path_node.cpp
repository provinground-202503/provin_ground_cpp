#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <limits>

class MultiLocalPathNode {
public:
    MultiLocalPathNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // 파라미터 로드
        private_nh.param("num_paths", num_paths_, 7);
        private_nh.param("local_path_size", local_path_size_, 20);

        // 구독자 및 발행자 설정
        pose_sub_ = nh.subscribe("/utm_fix", 1, &MultiLocalPathNode::poseCallback, this);
        
        local_path_pubs_.resize(num_paths_);
        global_paths_.resize(num_paths_);
        global_path_subs_.resize(num_paths_);

        for (int i = 0; i < num_paths_; ++i) {
            std::string sub_topic = "/global_path/path_" + std::to_string(i);
            // bind와 lambda를 사용하여 콜백에 인덱스 전달
            global_path_subs_[i] = nh.subscribe<nav_msgs::Path>(sub_topic, 1, 
                [this, i](const nav_msgs::Path::ConstPtr& msg){ this->globalPathCallback(msg, i); });

            std::string pub_topic = "/local_path/path_" + std::to_string(i);
            local_path_pubs_[i] = nh.advertise<nav_msgs::Path>(pub_topic, 1);
        }
    }

    void run() {
        ros::spin();
    }

private:
    // 전역 경로 콜백
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg, int path_idx) {
        global_paths_[path_idx] = *msg;
        ROS_INFO("Received global path %d with %zu poses.", path_idx, msg->poses.size());
    }

    // 위치 콜백
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        for (int i = 0; i < num_paths_; ++i) {
            if (global_paths_[i].poses.empty()) {
                continue;
            }

            int closest_idx = findClosestWaypoint(msg->pose.position, global_paths_[i]);
            if (closest_idx == -1) continue;

            nav_msgs::Path local_path;
            local_path.header.stamp = ros::Time::now();
            local_path.header.frame_id = "gps";

            // 가장 가까운 지점부터 local_path_size_ 만큼 점을 추출
            for (int j = 0; j < local_path_size_; ++j) {
                int target_idx = closest_idx + j;
                if (target_idx < global_paths_[i].poses.size()) {
                    local_path.poses.push_back(global_paths_[i].poses[target_idx]);
                } else {
                    break; // 경로 끝에 도달
                }
            }
            
            if (!local_path.poses.empty()) {
                local_path_pubs_[i].publish(local_path);
            }
        }
    }

    // 가장 가까운 웨이포인트 인덱스 찾기
    int findClosestWaypoint(const geometry_msgs::Point& current_pos, const nav_msgs::Path& path) {
        double min_dist_sq = std::numeric_limits<double>::max();
        int closest_idx = -1;

        for (int i = 0; i < path.poses.size(); ++i) {
            double dx = current_pos.x - path.poses[i].pose.position.x;
            double dy = current_pos.y - path.poses[i].pose.position.y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    ros::Subscriber pose_sub_;
    std::vector<ros::Subscriber> global_path_subs_;
    std::vector<ros::Publisher> local_path_pubs_;
    std::vector<nav_msgs::Path> global_paths_;
    
    int num_paths_;
    int local_path_size_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_local_path_node");
    MultiLocalPathNode node;
    node.run();
    return 0;
}