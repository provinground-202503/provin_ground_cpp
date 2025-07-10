#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <random>

//=========================================================
// RRT* 알고리즘을 위한 파라미터
//=========================================================
// RRT* 탐색 최대 반복 횟수
const int MAX_ITERATIONS = 1500;
// 트리 성장 시 한 스텝의 크기 (미터)
const double STEP_SIZE = 0.5;
// 장애물 충돌 반경 (미터)
const double OBSTACLE_RADIUS = 0.1;
// RRT* 재연결(Rewire)을 위한 이웃 노드 탐색 반경 (미터)
const double NEIGHBOR_RADIUS = 0.5;
// 목표점에 도달했다고 판단하는 반경 (미터)
const double GOAL_RADIUS = 0.1;
// 경로 평탄화를 위한 가중치
const double SMOOTH_WEIGHT_DATA = 0.7;
const double SMOOTH_WEIGHT_SMOOTH = 0.1;
const double SMOOTH_TOLERANCE = 0.01;


// RRT* 트리의 노드를 표현하는 구조체
struct Node {
    double x, y;      // 노드의 좌표
    int parent_id;    // 부모 노드의 인덱스 (-1이면 시작 노드)
    double cost;      // 시작점으로부터의 총 비용 (g)

    // 생성자
    Node(double _x, double _y, int _pid = -1, double _cost = 0.0)
        : x(_x), y(_y), parent_id(_pid), cost(_cost) {}
};

class ObstacleAvoidePath {
public:
    ObstacleAvoidePath() {
        ros::NodeHandle nh("~");
        // 서브스크라이버와 퍼블리셔는 기존 코드와 동일하게 설정
        local_path_sub_ = nh.subscribe("/local_path", 1, &ObstacleAvoidePath::localPathCallback, this);
        utm_sub_ = nh.subscribe("/utm", 1, &ObstacleAvoidePath::utmCallback, this);
        clustered_sub_ = nh.subscribe("/filtered_clusters", 1, &ObstacleAvoidePath::clusteredCallback, this);
        avoid_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);

        // 10Hz (0.1초마다) 주기로 RRT* 경로 계획 실행
        path_publish_timer_ = nh.createTimer(ros::Duration(0.05), &ObstacleAvoidePath::pathPublishCallback, this);

        current_vehicle_yaw_ = 0.0;
        previous_utm_.header.stamp.fromSec(0);

        // 난수 생성을 위한 초기화 (RRT* 샘플링에 사용)
        rng_ = std::mt19937(rd_());

        ROS_INFO("RRT* based Obstacle Avoidance Path node has been initialized.");
    }

private:
    // ROS 핸들러
    ros::Subscriber local_path_sub_;
    ros::Subscriber utm_sub_;
    ros::Subscriber clustered_sub_;
    ros::Publisher avoid_path_pub_;
    ros::Timer path_publish_timer_;

    // 데이터 저장용 멤버 변수
    nav_msgs::Path local_path_;
    geometry_msgs::PoseStamped current_utm_;
    geometry_msgs::PoseStamped previous_utm_;
    std::vector<geometry_msgs::Point> global_obstacles_; // 변환된 장애물 좌표
    double current_vehicle_yaw_;

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
        // 이 콜백에서는 장애물 좌표를 로컬에서 글로벌로 바로 변환하여 저장
        if (current_utm_.header.stamp.isZero()) return; // 차량 위치 모르면 변환 불가

        // Yaw 계산 (이전 위치와 현재 위치 기반)
        if (!previous_utm_.header.stamp.isZero()) {
            double delta_x = current_utm_.pose.position.x - previous_utm_.pose.position.x;
            double delta_y = current_utm_.pose.position.y - previous_utm_.pose.position.y;
            if (std::hypot(delta_x, delta_y) > 0.01) {
                current_vehicle_yaw_ = std::atan2(delta_y, delta_x);
            }
        }
        previous_utm_ = current_utm_;

        global_obstacles_.clear();
        double vehicle_x = current_utm_.pose.position.x;
        double vehicle_y = current_utm_.pose.position.y;

        for (const auto& marker : msg->markers) {
            double obs_local_x = marker.pose.position.x;
            double obs_local_y = marker.pose.position.y;

            geometry_msgs::Point global_obs_point;
            global_obs_point.x = vehicle_x + obs_local_x * std::cos(current_vehicle_yaw_) - obs_local_y * std::sin(current_vehicle_yaw_);
            global_obs_point.y = vehicle_y + obs_local_x * std::sin(current_vehicle_yaw_) + obs_local_y * std::cos(current_vehicle_yaw_);
            global_obs_point.z = current_utm_.pose.position.z;
            global_obstacles_.push_back(global_obs_point);
        }
    }


    /**
 * @brief RRT* 경로 계획 또는 직선 경로를 생성하고 결과를 발행하는 메인 타이머 콜백
 */
    void pathPublishCallback(const ros::TimerEvent&) {
        if (local_path_.poses.empty() || current_utm_.header.stamp.isZero()) {
            ROS_WARN_THROTTLE(1.0, "Waiting for local_path or utm message...");
            return;
        }

        // 최종적으로 발행될 경로 메시지
        nav_msgs::Path avoid_path;
        avoid_path.header.stamp = ros::Time::now();
        avoid_path.header.frame_id = "map";

        // 💡 핵심 수정 부분: 장애물 유무에 따라 로직 분기
        if (global_obstacles_.empty()) {
            // --- 1. 장애물이 없을 때: 직선 경로 생성 ---
            ROS_INFO_THROTTLE(1.0, "No obstacles detected. Copying local path.");
            avoid_path.poses = local_path_.poses;

        } else {
            // --- 2. 장애물이 있을 때: RRT* 경로 계획 수행 ---
            ROS_INFO_THROTTLE(1.0, "Obstacles detected. Planning with RRT*.");

            // 시작점과 목표점 설정
            Node start_node(current_utm_.pose.position.x, current_utm_.pose.position.y);
            Node goal_node(local_path_.poses.back().pose.position.x, local_path_.poses.back().pose.position.y);

            // RRT* 알고리즘으로 경로 계획
            std::vector<Node> path_nodes = planRRTStar(start_node, goal_node);

            if (path_nodes.empty()) {
                ROS_WARN("RRT* failed to find a path. Publishing an empty path.");
            } else {
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

        // 최종 경로 발행
        avoid_path_pub_.publish(avoid_path);
    }
    // --- RRT* 헬퍼 함수들 ---

    double getDistance(double x1, double y1, double x2, double y2) {
        return std::hypot(x1 - x2, y1 - y2);
    }
    
    /**
     * @brief 주어진 두 점 사이에 장애물이 있는지 확인
     */
    bool isCollision(const Node& a, const Node& b) {
        for (const auto& obs : global_obstacles_) {
            // 선분과 점 사이의 최단 거리 계산
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            if (dx == 0 && dy == 0) continue;

            double t = ((obs.x - a.x) * dx + (obs.y - a.y) * dy) / (dx*dx + dy*dy);
            t = std::max(0.0, std::min(1.0, t));

            double closest_x = a.x + t * dx;
            double closest_y = a.y + t * dy;

            if (getDistance(obs.x, obs.y, closest_x, closest_y) < OBSTACLE_RADIUS) {
                return true; // 충돌
            }
        }
        return false; // 충돌 없음
    }

    /**
     * @brief 트리에서 샘플 지점과 가장 가까운 노드를 찾음
     */
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

    /**
     * @brief from 노드에서 to 노드 방향으로 STEP_SIZE 만큼 떨어진 새 노드를 생성
     */
    Node steer(const Node& from, const Node& to) {
        double dist = getDistance(from.x, from.y, to.x, to.y);
        if (dist < STEP_SIZE) {
            return to;
        }
        double ratio = STEP_SIZE / dist;
        return Node(from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio);
    }
    
    /**
     * @brief RRT* 알고리즘의 메인 함수
     */
    std::vector<Node> planRRTStar(const Node& start, const Node& goal) {
        std::vector<Node> tree;
        tree.push_back(start);

        // 샘플링 영역(Bounding Box)을 동적으로 설정
        double min_x = start.x, max_x = start.x;
        double min_y = start.y, max_y = start.y;
        for(const auto& pose : local_path_.poses){
            min_x = std::min(min_x, pose.pose.position.x);
            max_x = std::max(max_x, pose.pose.position.x);
            min_y = std::min(min_y, pose.pose.position.y);
            max_y = std::max(max_y, pose.pose.position.y);
        }
        // 약간의 여유 공간 추가
        min_x -= 5.0; max_x += 5.0;
        min_y -= 5.0; max_y += 5.0;

        std::uniform_real_distribution<double> distX(min_x, max_x);
        std::uniform_real_distribution<double> distY(min_y, max_y);


        for (int i = 0; i < MAX_ITERATIONS; ++i) {
            // 1. 랜덤 샘플 생성 (가끔 목표점을 샘플링하여 수렴 속도 향상)
            Node random_sample = (rand() % 100 > 90) ? goal : Node(distX(rng_), distY(rng_));

            // 2. 가장 가까운 노드 찾기
            int nearest_id = findNearestNode(tree, random_sample);
            Node nearest_node = tree[nearest_id];
            
            // 3. 새 노드(new_node)로 트리 확장(Steer)
            Node new_node = steer(nearest_node, random_sample);

            // 4. 충돌 체크
            if (isCollision(nearest_node, new_node)) {
                continue;
            }

            // 5. RRT* : 최적의 부모 노드 찾기 (Choose Parent)
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

            // 6. RRT* : 트리 재연결 (Rewire)
            for (int neighbor_id : neighbor_ids) {
                if(neighbor_id == best_parent_id) continue;
                
                Node& neighbor_node = tree[neighbor_id];
                double new_neighbor_cost = new_node.cost + getDistance(new_node.x, new_node.y, neighbor_node.x, neighbor_node.y);

                if (new_neighbor_cost < neighbor_node.cost && !isCollision(new_node, neighbor_node)) {
                    neighbor_node.parent_id = new_node_id;
                    neighbor_node.cost = new_neighbor_cost;
                }
            }

            // 7. 목표점 도달 확인
            if (getDistance(new_node.x, new_node.y, goal.x, goal.y) < GOAL_RADIUS) {
                 // 목표점에 도달했어도, 더 나은 경로를 위해 계속 탐색할 수 있음. 여기서는 즉시 경로 반환
                 ROS_INFO("RRT* path found to goal!");
                 break;
            }
        }

        // 경로 재구성: 목표점에서 가장 가까운 노드에서부터 시작점까지 역추적
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

        if(last_node_id != -1 && min_goal_dist < GOAL_RADIUS * 2) { // 목표 반경 근처에 노드가 있다면
            int current_id = last_node_id;
            while(current_id != -1){
                path.push_back(tree[current_id]);
                current_id = tree[current_id].parent_id;
            }
        }
        return path; // 경로는 목표->시작 순서
    }

    /**
     * @brief 경로를 부드럽게 만드는 함수 (단순 구현)
     */
    std::vector<Node> smoothPath(const std::vector<Node>& path) {
        if (path.size() < 3) return path;

        std::vector<Node> smoothed_path = path;
        double change = SMOOTH_TOLERANCE;

        while (change >= SMOOTH_TOLERANCE) {
            change = 0.0;
            for (size_t i = 1; i < path.size() - 1; ++i) {
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
    ros::init(argc, argv, "rrt_star_obstacle_avoid_node");
    ObstacleAvoidePath rrt_avoider;
    ros::spin();
    return 0;
}