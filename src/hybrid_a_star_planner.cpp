#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <algorithm> // for std::reverse

// ===================================================================================
// 데이터 구조체 정의 (원래 헤더 파일에 있던 내용)
// ===================================================================================

// 3D 상태 공간(x, y, theta)의 노드를 표현하는 구조체
struct Node3D {
    int grid_x, grid_y, grid_theta; // 그리드 인덱스
    double x, y, theta;             // 실제 좌표
    double g_cost;                  // 시작점으로부터의 실제 비용
    double h_cost;                  // 도착점까지의 예상 비용 (Heuristic)
    std::shared_ptr<Node3D> parent; // 부모 노드를 가리키는 포인터

    Node3D(int gx, int gy, int gth, double real_x, double real_y, double real_th)
        : grid_x(gx), grid_y(gy), grid_theta(gth), x(real_x), y(real_y), theta(real_th),
          g_cost(std::numeric_limits<double>::max()), h_cost(0.0), parent(nullptr) {}

    double getFCost() const { return g_cost + h_cost; }
};

// 우선순위 큐에서 f_cost를 비교하기 위한 비교 구조체
struct CompareNode {
    bool operator()(const std::shared_ptr<Node3D>& a, const std::shared_ptr<Node3D>& b) const {
        return a->getFCost() > b->getFCost();
    }
};

// ===================================================================================
// HybridAStarPlanner 클래스 정의
// ===================================================================================

class HybridAStarPlanner {
public:
    // 생성자
    HybridAStarPlanner(ros::NodeHandle& nh);

private:
    // ROS 관련 멤버
    ros::NodeHandle nh_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_local_path_;
    ros::Subscriber sub_obstacles_;
    ros::Publisher pub_avoid_path_;
    ros::Publisher pub_costmap_; // 디버깅용 코스트맵 발행
    ros::Timer timer_;

    // 입력 데이터
    geometry_msgs::PoseStamped current_pose_;
    nav_msgs::Path local_path_;
    visualization_msgs::MarkerArray obstacles_;
    bool pose_received_ = false;
    bool path_received_ = false;

    // 차량 파라미터
    double vehicle_width_;
    double vehicle_length_;
    double vehicle_base_to_front_;
    double max_steer_angle_; // 라디안 단위

    // 알고리즘 파라미터
    double grid_resolution_;
    int theta_resolution_;
    double motion_primitive_dist_;
    double goal_tolerance_dist_;
    double goal_tolerance_angle_;

    // 내부 데이터
    nav_msgs::OccupancyGrid costmap_;

    // 콜백 함수
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    
    // 메인 플래닝 함수
    void plan(const ros::TimerEvent& event);

    // 보조 함수
    void createCostmapFromObstacles();
    std::shared_ptr<Node3D> poseToNode(const geometry_msgs::Pose& pose);
    bool isColliding(double x, double y, double theta);
    double normalizeAngle(double angle);
    std::vector<std::shared_ptr<Node3D>> getMotionPrimitives(const std::shared_ptr<Node3D>& current_node);
    double calculateReedsSheppPathCost(const std::shared_ptr<Node3D>& from, const std::shared_ptr<Node3D>& to);
    nav_msgs::Path reconstructPath(const std::shared_ptr<Node3D>& goal_node);
};


// ===================================================================================
// 클래스 멤버 함수 구현 (원래 cpp 파일에 있던 내용)
// ===================================================================================

// 생성자: 파라미터 로드 및 ROS 인터페이스 초기화
HybridAStarPlanner::HybridAStarPlanner(ros::NodeHandle& nh) : nh_(nh) {
    // ROS 파라미터 서버에서 파라미터 로드
    nh_.param("vehicle_width", vehicle_width_, 1.4);
    nh_.param("vehicle_length", vehicle_length_, 1.6);
    nh_.param("vehicle_base_to_front", vehicle_base_to_front_, 1.3);
    nh_.param("max_steer_deg", max_steer_angle_, 28.0);
    nh_.param("grid_resolution", grid_resolution_, 0.2);
    nh_.param("theta_resolution", theta_resolution_, 72);
    nh_.param("motion_primitive_dist", motion_primitive_dist_, 1.0);
    nh_.param("goal_tolerance_dist", goal_tolerance_dist_, 0.5);
    nh_.param("goal_tolerance_angle", goal_tolerance_angle_, 10.0 * M_PI / 180.0);

    max_steer_angle_ = max_steer_angle_ * M_PI / 180.0; // 라디안으로 변환

    // Subscriber 및 Publisher 초기화
    sub_pose_ = nh_.subscribe("/utm_fix", 1, &HybridAStarPlanner::poseCallback, this);
    sub_local_path_ = nh_.subscribe("/local_path", 1, &HybridAStarPlanner::localPathCallback, this);
    sub_obstacles_ = nh_.subscribe("/filtered_obstacles", 1, &HybridAStarPlanner::obstaclesCallback, this);
    pub_avoid_path_ = nh_.advertise<nav_msgs::Path>("/avoid_path", 1);
    pub_costmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("/debug_costmap", 1);

    timer_ = nh_.createTimer(ros::Duration(0.2), &HybridAStarPlanner::plan, this);
    ROS_INFO("Hybrid A* Planner (Single File) Node Initialized.");
}

void HybridAStarPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    pose_received_ = true;
}

void HybridAStarPlanner::localPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    local_path_ = *msg;
    path_received_ = true;
}

void HybridAStarPlanner::obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    obstacles_ = *msg;
}

void HybridAStarPlanner::plan(const ros::TimerEvent& event) {
    if (!pose_received_ || !path_received_ || local_path_.poses.empty()) {
        ROS_WARN_THROTTLE(2.0, "Waiting for initial pose and local path...");
        return;
    }

    ROS_INFO("Start planning...");

    createCostmapFromObstacles();
    std::shared_ptr<Node3D> start_node = poseToNode(current_pose_.pose);
    std::shared_ptr<Node3D> goal_node = poseToNode(local_path_.poses.back().pose);

    start_node->g_cost = 0;
    start_node->h_cost = calculateReedsSheppPathCost(start_node, goal_node);

    std::priority_queue<std::shared_ptr<Node3D>, std::vector<std::shared_ptr<Node3D>>, CompareNode> open_list;
    std::unordered_map<std::string, std::shared_ptr<Node3D>> closed_list;

    open_list.push(start_node);

    while (!open_list.empty()) {
        std::shared_ptr<Node3D> current_node = open_list.top();
        open_list.pop();

        std::string current_key = std::to_string(current_node->grid_x) + "," + std::to_string(current_node->grid_y) + "," + std::to_string(current_node->grid_theta);
        if (closed_list.count(current_key)) {
            continue;
        }
        closed_list[current_key] = current_node;

        double dist_to_goal = std::hypot(current_node->x - goal_node->x, current_node->y - goal_node->y);
        double angle_diff = std::abs(normalizeAngle(current_node->theta - goal_node->theta));
        if (dist_to_goal < goal_tolerance_dist_ && angle_diff < goal_tolerance_angle_) {
            ROS_INFO("Goal Reached! Reconstructing path...");
            nav_msgs::Path avoid_path = reconstructPath(current_node);
            pub_avoid_path_.publish(avoid_path);
            return;
        }

        std::vector<std::shared_ptr<Node3D>> next_nodes = getMotionPrimitives(current_node);

        for (auto& next_node : next_nodes) {
            if (isColliding(next_node->x, next_node->y, next_node->theta)) {
                continue;
            }

            double new_g_cost = current_node->g_cost + motion_primitive_dist_;
            
            std::string next_key = std::to_string(next_node->grid_x) + "," + std::to_string(next_node->grid_y) + "," + std::to_string(next_node->grid_theta);
            if(closed_list.count(next_key)) continue;

            // Open 리스트에 이미 있는지 확인하는 로직은 복잡하므로, g_cost를 직접 비교하는 대신 그냥 push.
            // 우선순위 큐가 알아서 정렬하므로, 더 나은 경로가 나중에 탐색됨.
            next_node->g_cost = new_g_cost;
            next_node->h_cost = calculateReedsSheppPathCost(next_node, goal_node);
            next_node->parent = current_node;
            open_list.push(next_node);
        }
    }
    ROS_WARN("Could not find a valid path to the goal.");
}

void HybridAStarPlanner::createCostmapFromObstacles() {
    costmap_.header.frame_id = "gps";
    costmap_.info.resolution = grid_resolution_;
    costmap_.info.width = 400;
    costmap_.info.height = 400;
    costmap_.info.origin.position.x = current_pose_.pose.position.x - (costmap_.info.width / 2.0 * grid_resolution_);
    costmap_.info.origin.position.y = current_pose_.pose.position.y - (costmap_.info.height / 2.0 * grid_resolution_);
    costmap_.data.assign(costmap_.info.width * costmap_.info.height, 0);

    double inflation_radius = std::hypot(vehicle_length_, vehicle_width_) / 2.0;
    int inflation_cells = std::ceil(inflation_radius / grid_resolution_);

    for (const auto& marker : obstacles_.markers) {
        int cx = (marker.pose.position.x - costmap_.info.origin.position.x) / grid_resolution_;
        int cy = (marker.pose.position.y - costmap_.info.origin.position.y) / grid_resolution_;
        
        for (int i = -inflation_cells; i <= inflation_cells; ++i) {
            for (int j = -inflation_cells; j <= inflation_cells; ++j) {
                if (std::hypot(i, j) <= inflation_cells) {
                    int map_x = cx + i;
                    int map_y = cy + j;
                    if (map_x >= 0 && map_x < costmap_.info.width && map_y >= 0 && map_y < costmap_.info.height) {
                        costmap_.data[map_y * costmap_.info.width + map_x] = 100;
                    }
                }
            }
        }
    }
    pub_costmap_.publish(costmap_);
}

bool HybridAStarPlanner::isColliding(double x, double y, double theta) {
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    double half_w = vehicle_width_ / 2.0;
    double front = vehicle_base_to_front_;
    double rear = vehicle_length_ - vehicle_base_to_front_;

    std::vector<std::pair<double, double>> corners;
    corners.emplace_back(x + front * cos_th - half_w * sin_th, y + front * sin_th + half_w * cos_th);
    corners.emplace_back(x + front * cos_th + half_w * sin_th, y + front * sin_th - half_w * cos_th);
    corners.emplace_back(x - rear * cos_th - half_w * sin_th, y - rear * sin_th + half_w * cos_th);
    corners.emplace_back(x - rear * cos_th + half_w * sin_th, y - rear * sin_th - half_w * cos_th);

    for (const auto& corner : corners) {
        int map_x = (corner.first - costmap_.info.origin.position.x) / grid_resolution_;
        int map_y = (corner.second - costmap_.info.origin.position.y) / grid_resolution_;
        if (map_x < 0 || map_x >= costmap_.info.width || map_y < 0 || map_y >= costmap_.info.height) return true;
        if (costmap_.data[map_y * costmap_.info.width + map_x] > 50) return true;
    }
    return false;
}

std::vector<std::shared_ptr<Node3D>> HybridAStarPlanner::getMotionPrimitives(const std::shared_ptr<Node3D>& current_node) {
    std::vector<std::shared_ptr<Node3D>> next_nodes;
    double L = vehicle_length_;
    std::vector<double> steer_angles = {-max_steer_angle_, 0.0, max_steer_angle_};
    
    for (double steer : steer_angles) {
        double next_x, next_y, next_theta;
        if (std::abs(steer) < 1e-5) {
            next_x = current_node->x + motion_primitive_dist_ * cos(current_node->theta);
            next_y = current_node->y + motion_primitive_dist_ * sin(current_node->theta);
            next_theta = current_node->theta;
        } else {
            double turn_radius = L / tan(steer);
            double delta_theta = motion_primitive_dist_ / turn_radius;
            next_x = current_node->x + turn_radius * (sin(current_node->theta + delta_theta) - sin(current_node->theta));
            next_y = current_node->y - turn_radius * (cos(current_node->theta + delta_theta) - cos(current_node->theta));
            next_theta = normalizeAngle(current_node->theta + delta_theta);
        }
        
        auto next_pose = geometry_msgs::Pose();
        next_pose.position.x = next_x;
        next_pose.position.y = next_y;
        auto next_node = poseToNode(next_pose);
        next_node->theta = next_theta;
        next_nodes.push_back(next_node);
    }
    return next_nodes;
}

double HybridAStarPlanner::calculateReedsSheppPathCost(const std::shared_ptr<Node3D>& from, const std::shared_ptr<Node3D>& to) {
    return std::hypot(from->x - to->x, from->y - to->y);
}

nav_msgs::Path HybridAStarPlanner::reconstructPath(const std::shared_ptr<Node3D>& goal_node) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "gps";
    std::shared_ptr<Node3D> current_node = goal_node;
    while (current_node != nullptr) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = current_node->x;
        pose.pose.position.y = current_node->y;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_node->theta);
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
        current_node = current_node->parent;
    }
    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

std::shared_ptr<Node3D> HybridAStarPlanner::poseToNode(const geometry_msgs::Pose& pose) {
    double x = pose.position.x;
    double y = pose.position.y;
    double theta = normalizeAngle(tf2::getYaw(pose.orientation));
    int gx = static_cast<int>(x / grid_resolution_);
    int gy = static_cast<int>(y / grid_resolution_);
    int gth = static_cast<int>(theta / (2 * M_PI / theta_resolution_));
    return std::make_shared<Node3D>(gx, gy, gth, x, y, theta);
}

double HybridAStarPlanner::normalizeAngle(double angle) {
    return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}


// ===================================================================================
// 메인 함수
// ===================================================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "hybrid_a_star_planner_node");
    ros::NodeHandle nh("~");
    HybridAStarPlanner planner(nh);
    ros::spin();
    return 0;
}