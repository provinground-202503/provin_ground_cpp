#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

// TF2 (좌표 변환)를 위한 헤더 추가
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>

//=========================================================
// RRT* 알고리즘을 위한 파라미터
//=========================================================
const int MAX_ITERATIONS = 1000;
const double STEP_SIZE = 0.3;
const double OBSTACLE_RADIUS = 0.4;
const double NEIGHBOR_RADIUS = 0.5;
const double GOAL_RADIUS = 0.2;
const double SMOOTH_WEIGHT_DATA = 0.5;
const double SMOOTH_WEIGHT_SMOOTH = 0.2;
const double SMOOTH_TOLERANCE = 0.01;


// RRT* 트리의 노드를 표현하는 구조체
struct Node {
    double x, y;
    int parent_id;
    double cost;

    Node(double _x, double _y, int _pid = -1, double _cost = 0.0)
        : x(_x), y(_y), parent_id(_pid), cost(_cost) {}
};

class ObstacleAvoidancePath {
public:
    ObstacleAvoidancePath() : tf_listener_(tf_buffer_) {
        ros::NodeHandle nh("~");

        // 서브스크라이버 설정
        local_path_sub_ = nh.subscribe("/local_path", 1, &ObstacleAvoidancePath::localPathCallback, this);
        utm_sub_ = nh.subscribe("/utm_fix", 1, &ObstacleAvoidancePath::utmCallback, this);
        clustered_sub_ = nh.subscribe("/lidar_clusters", 1, &ObstacleAvoidancePath::clusteredCallback, this);

        // 퍼블리셔 설정
        avoid_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);
        local_path_base_link_pub_ = nh.advertise<nav_msgs::Path>("/local_path_base_link", 1);
        avoid_path_base_link_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path_base_link", 1);

        // 타이머 설정
        path_publish_timer_ = nh.createTimer(ros::Duration(0.05), &ObstacleAvoidancePath::pathPublishCallback, this);
        
        rng_ = std::mt19937(rd_());
        ROS_INFO("RRT* Obstacle Avoidance Path node has been initialized.");
    }

private:
    // ROS 관련 멤버
    ros::Subscriber local_path_sub_;
    ros::Subscriber utm_sub_;
    ros::Subscriber clustered_sub_;
    ros::Publisher avoid_path_pub_;
    ros::Publisher local_path_base_link_pub_;
    ros::Publisher avoid_path_base_link_pub_;
    ros::Timer path_publish_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 데이터 저장용 멤버 변수
    nav_msgs::Path local_path_;
    geometry_msgs::PoseStamped current_utm_;
    std::vector<geometry_msgs::Point> global_obstacles_;

    // 난수 생성기
    std::random_device rd_;
    std::mt19937 rng_;

    // 콜백 함수들
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        local_path_ = *msg;
    }

    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_utm_ = *msg;
    }

    void clusteredCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        global_obstacles_.clear();
        std::string target_frame = "map"; // RRT*가 실행될 목표 좌표계

        for (const auto& marker : msg->markers) {
            geometry_msgs::PoseStamped marker_pose_stamped;
            marker_pose_stamped.header = marker.header;
            marker_pose_stamped.pose = marker.pose;

            try {
                geometry_msgs::PoseStamped transformed_pose = tf_buffer_.transform(marker_pose_stamped, target_frame, ros::Duration(0.1));
                global_obstacles_.push_back(transformed_pose.pose.position);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("TF2 transform failed: %s", ex.what());
                continue;
            }
        }
    }

    void pathPublishCallback(const ros::TimerEvent&) {
        if (local_path_.poses.empty() || current_utm_.header.stamp.isZero()) {
            ROS_WARN_THROTTLE(1.0, "Waiting for local_path or utm message...");
            return;
        }

        nav_msgs::Path avoid_path;
        avoid_path.header.stamp = ros::Time::now();
        avoid_path.header.frame_id = "map";

        if (global_obstacles_.empty()) {
            avoid_path.poses = local_path_.poses;
        } else {
            Node start_node(current_utm_.pose.position.x, current_utm_.pose.position.y);
            Node goal_node(local_path_.poses.back().pose.position.x, local_path_.poses.back().pose.position.y);
            std::vector<Node> path_nodes = planRRTStar(start_node, goal_node);

            if (!path_nodes.empty()) {
                std::reverse(path_nodes.begin(), path_nodes.end());
                std::vector<Node> smoothed_path = smoothPath(path_nodes);
                for (const auto& node : smoothed_path) {
                    geometry_msgs::PoseStamped pose;
                    pose.header = avoid_path.header;
                    pose.pose.position.x = node.x;
                    pose.pose.position.y = node.y;
                    pose.pose.position.z = current_utm_.pose.position.z;
                    pose.pose.orientation.w = 1.0;
                    avoid_path.poses.push_back(pose);
                }
            }
        }

        avoid_path_pub_.publish(avoid_path);

        // 경로들을 base_link(os_sensor) 기준으로 변환하여 발행
        std::string target_frame = "base_link";
        geometry_msgs::TransformStamped map_to_base_transform;
        try {
            map_to_base_transform = tf_buffer_.lookupTransform(target_frame, "map", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not get transform from 'map' to 'base_link': %s", ex.what());
            return;
        }

        nav_msgs::Path local_path_in_base_link = transformPath(local_path_, map_to_base_transform, target_frame);
        local_path_base_link_pub_.publish(local_path_in_base_link);

        if(!avoid_path.poses.empty()){
             nav_msgs::Path avoid_path_in_base_link = transformPath(avoid_path, map_to_base_transform, target_frame);
             avoid_path_base_link_pub_.publish(avoid_path_in_base_link);
        }
    }
    
    // --- RRT* 헬퍼 함수들 ---

    nav_msgs::Path transformPath(const nav_msgs::Path& input_path, const geometry_msgs::TransformStamped& transform, const std::string& target_frame) {
        nav_msgs::Path transformed_path;
        transformed_path.header.stamp = ros::Time::now();
        transformed_path.header.frame_id = target_frame;
        for (const auto& original_pose : input_path.poses) {
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(original_pose, transformed_pose, transform);
            transformed_path.poses.push_back(transformed_pose);
        }
        return transformed_path;
    }

    double getDistance(double x1, double y1, double x2, double y2) {
        return std::hypot(x1 - x2, y1 - y2);
    }
    
    bool isCollision(const Node& a, const Node& b) {
        for (const auto& obs : global_obstacles_) {
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            if (dx == 0 && dy == 0) continue;
            double t = ((obs.x - a.x) * dx + (obs.y - a.y) * dy) / (dx*dx + dy*dy);
            t = std::max(0.0, std::min(1.0, t));
            double closest_x = a.x + t * dx;
            double closest_y = a.y + t * dy;
            if (getDistance(obs.x, obs.y, closest_x, closest_y) < OBSTACLE_RADIUS) {
                return true;
            }
        }
        return false;
    }

    int findNearestNode(const std::vector<Node>& tree, const Node& sample) {
        int nearest_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < tree.size(); ++i) {
            double dist = getDistance(tree[i].x, tree[i].y, sample.x, sample.y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_id = i;
            }
        }
        return nearest_id;
    }

    Node steer(const Node& from, const Node& to) {
        double dist = getDistance(from.x, from.y, to.x, to.y);
        if (dist < STEP_SIZE) {
            return to;
        }
        double ratio = STEP_SIZE / dist;
        return Node(from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio);
    }
    
    std::vector<Node> planRRTStar(const Node& start, const Node& goal) {
        std::vector<Node> tree;
        tree.push_back(start);

        double min_x = start.x, max_x = start.x, min_y = start.y, max_y = start.y;
        for(const auto& pose : local_path_.poses){
            min_x = std::min(min_x, pose.pose.position.x);
            max_x = std::max(max_x, pose.pose.position.x);
            min_y = std::min(min_y, pose.pose.position.y);
            max_y = std::max(max_y, pose.pose.position.y);
        }
        min_x -= 5.0; max_x += 5.0; min_y -= 5.0; max_y += 5.0;

        std::uniform_real_distribution<double> distX(min_x, max_x);
        std::uniform_real_distribution<double> distY(min_y, max_y);

        for (int i = 0; i < MAX_ITERATIONS; ++i) {
            Node random_sample = (rand() % 100 > 90) ? goal : Node(distX(rng_), distY(rng_));
            int nearest_id = findNearestNode(tree, random_sample);
            Node nearest_node = tree[nearest_id];
            Node new_node = steer(nearest_node, random_sample);

            if (isCollision(nearest_node, new_node)) {
                continue;
            }

            int best_parent_id = nearest_id;
            double min_cost = nearest_node.cost + getDistance(nearest_node.x, nearest_node.y, new_node.x, new_node.y);
            
            std::vector<int> neighbor_ids;
            for(size_t j=0; j<tree.size(); ++j){
                if(getDistance(tree[j].x, tree[j].y, new_node.x, new_node.y) < NEIGHBOR_RADIUS){
                    neighbor_ids.push_back(j);
                }
            }

            for (int neighbor_id : neighbor_ids) {
                const Node& neighbor_node = tree[neighbor_id];
                if (!isCollision(neighbor_node, new_node)) {
                    double cost = neighbor_node.cost + getDistance(neighbor_node.x, neighbor_node.y, new_node.x, new_node.y);
                    if (cost < min_cost) {
                        min_cost = cost;
                        best_parent_id = neighbor_id;
                    }
                }
            }
            new_node.parent_id = best_parent_id;
            new_node.cost = min_cost;
            tree.push_back(new_node);
            int new_node_id = tree.size() - 1;

            for (int neighbor_id : neighbor_ids) {
                if(neighbor_id == best_parent_id) continue;
                
                Node& neighbor_node = tree[neighbor_id];
                double new_neighbor_cost = new_node.cost + getDistance(new_node.x, new_node.y, neighbor_node.x, neighbor_node.y);

                if (new_neighbor_cost < neighbor_node.cost && !isCollision(new_node, neighbor_node)) {
                    neighbor_node.parent_id = new_node_id;
                    neighbor_node.cost = new_neighbor_cost;
                }
            }

            if (getDistance(new_node.x, new_node.y, goal.x, goal.y) < GOAL_RADIUS) {
                 ROS_INFO("RRT* path found to goal!");
                 break;
            }
        }

        std::vector<Node> path;
        int last_node_id = -1;
        double min_goal_dist = std::numeric_limits<double>::max();

        for(size_t i=0; i<tree.size(); ++i){
            double dist = getDistance(tree[i].x, tree[i].y, goal.x, goal.y);
            if(dist < min_goal_dist){
                min_goal_dist = dist;
                last_node_id = i;
            }
        }

        if(last_node_id != -1 && min_goal_dist < GOAL_RADIUS * 2) {
            int current_id = last_node_id;
            while(current_id != -1){
                path.push_back(tree[current_id]);
                current_id = tree[current_id].parent_id;
            }
        }
        return path;
    }

    std::vector<Node> smoothPath(const std::vector<Node>& path) {
        if (path.size() < 3) return path;
        std::vector<Node> smoothed_path = path;
        double change = SMOOTH_TOLERANCE;

        while (change >= SMOOTH_TOLERANCE) {
            change = 0.0;
            for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
                double aux_x = smoothed_path[i].x;
                double aux_y = smoothed_path[i].y;

                smoothed_path[i].x += SMOOTH_WEIGHT_DATA * (path[i].x - smoothed_path[i].x);
                smoothed_path[i].y += SMOOTH_WEIGHT_DATA * (path[i].y - smoothed_path[i].y);
                
                smoothed_path[i].x += SMOOTH_WEIGHT_SMOOTH * (smoothed_path[i-1].x + smoothed_path[i+1].x - (2.0 * smoothed_path[i].x));
                smoothed_path[i].y += SMOOTH_WEIGHT_SMOOTH * (smoothed_path[i-1].y + smoothed_path[i+1].y - (2.0 * smoothed_path[i].y));

                change += std::abs(aux_x - smoothed_path[i].x);
                change += std::abs(aux_y - smoothed_path[i].y);
            }
        }
        return smoothed_path;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_star_obstacle_avoidance_node");
    ObstacleAvoidancePath rrt_avoider;
    ros::spin();
    return 0;
}