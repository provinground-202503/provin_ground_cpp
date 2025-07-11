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
#include <algorithm> // std::reverse 사용

//=========================================================
// RRT* 알고리즘을 위한 파라미터
//=========================================================
const int MAX_ITERATIONS = 1000;      // RRT* 탐색 최대 반복 횟수
const double STEP_SIZE = 0.3;         // 트리 성장 시 한 스텝의 크기 (미터)
const double OBSTACLE_RADIUS = 0.4;   // 장애물 충돌 반경 (미터). 라이다 노이즈 및 차량 크기를 고려하여 약간 크게 설정
const double NEIGHBOR_RADIUS = 0.5;   // RRT* 재연결(Rewire)을 위한 이웃 노드 탐색 반경 (미터)
const double GOAL_RADIUS = 0.2;       // 목표점에 도달했다고 판단하는 반경 (미터)
const double SMOOTH_WEIGHT_DATA = 0.5;      // 경로 평탄화 가중치 (원본 경로 유지)
const double SMOOTH_WEIGHT_SMOOTH = 0.2;    // 경로 평탄화 가중치 (부드러움)
const double SMOOTH_TOLERANCE = 0.01; // 평탄화 종료 조건


// RRT* 트리의 노드를 표현하는 구조체
struct Node {
    double x, y;
    int parent_id;
    double cost;

    Node(double _x, double _y, int _pid = -1, double _cost = 0.0)
        : x(_x), y(_y), parent_id(_pid), cost(_cost) {}
};

class ObstacleAvoidancePath { // 클래스 이름 수정
public:
    // 생성자: 노드 초기화, 퍼블리셔/서브스크라이버 및 타이머 설정
    ObstacleAvoidancePath() : tf_listener_(tf_buffer_) { // tf_listener_ 초기화
        ros::NodeHandle nh("~");

        // 서브스크라이버 설정
        // local_path: 상위 계획기(전역 계획)로부터 받은 추천 경로
        local_path_sub_ = nh.subscribe("/local_path", 1, &ObstacleAvoidancePath::localPathCallback, this);
        // utm: 차량의 현재 위치 (UTM 좌표계)
        utm_sub_ = nh.subscribe("/utm_fix", 1, &ObstacleAvoidancePath::utmCallback, this);
        // lidar_clusters: 라이다 클러스터링 노드로부터 받은 장애물 정보 (토픽 이름 수정)
        clustered_sub_ = nh.subscribe("/filtered_clusters", 1, &ObstacleAvoidancePath::clusteredCallback, this);

        // 퍼블리셔 설정
        // avoid_path: RRT*로 생성된 최종 회피 경로
        avoid_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);

        // 20Hz (0.05초마다) 주기로 경로 계획 실행
        path_publish_timer_ = nh.createTimer(ros::Duration(0.05), &ObstacleAvoidancePath::pathPublishCallback, this);

        // 난수 생성을 위한 초기화 (RRT* 샘플링에 사용)
        rng_ = std::mt19937(rd_());

        ROS_INFO("RRT* Obstacle Avoidance Path node has been initialized.");
    }

private:
    // ROS 관련 멤버
    ros::Subscriber local_path_sub_;
    ros::Subscriber utm_sub_;
    ros::Subscriber clustered_sub_;
    ros::Publisher avoid_path_pub_;
    ros::Timer path_publish_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 데이터 저장용 멤버 변수
    nav_msgs::Path local_path_;
    geometry_msgs::PoseStamped current_utm_;
    std::vector<geometry_msgs::Point> global_obstacles_; // 'map' 좌표계 기준으로 변환된 장애물 좌표

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

    /**
     * @brief [핵심 수정] 라이다 클러스터링 결과를 받아 장애물 위치를 저장하는 콜백
     * 수동으로 좌표 변환하는 대신, tf2 라이브러리를 사용하여 안정적으로 변환합니다.
     */
    void clusteredCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        global_obstacles_.clear();
        std::string target_frame = "map"; // RRT*가 실행될 목표 좌표계 (UTM과 동일하다고 가정)

        for (const auto& marker : msg->markers) {
            // 들어온 마커의 좌표계(frame_id)를 목표 좌표계(target_frame)로 변환해야 함
            geometry_msgs::PoseStamped marker_pose_stamped;
            marker_pose_stamped.header = marker.header;
            marker_pose_stamped.pose = marker.pose;

            try {
                // tf_buffer_를 통해 좌표 변환 실행
                geometry_msgs::PoseStamped transformed_pose = tf_buffer_.transform(marker_pose_stamped, target_frame, ros::Duration(0.1));
                
                // 변환된 좌표를 장애물 목록에 추가
                global_obstacles_.push_back(transformed_pose.pose.position);

            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                continue; // 변환 실패 시 해당 장애물은 무시
            }
        }
    }

    // RRT* 경로 계획 또는 직선 경로를 생성하고 결과를 발행하는 메인 타이머 콜백
    void pathPublishCallback(const ros::TimerEvent&) {
        if (local_path_.poses.empty() || current_utm_.header.stamp.isZero()) {
            ROS_WARN_THROTTLE(1.0, "Waiting for local_path or utm message...");
            return;
        }

        nav_msgs::Path avoid_path;
        avoid_path.header.stamp = ros::Time::now();
        avoid_path.header.frame_id = "map";

        // 장애물 유무에 따라 로직 분기
        if (global_obstacles_.empty()) {
            // 장애물이 없을 때는 원래의 local_path를 그대로 발행
            ROS_INFO_THROTTLE(1.0, "No obstacles. Following local path.");
            avoid_path.poses = local_path_.poses;
        } else {
            // 장애물이 있을 때는 RRT* 알고리즘으로 회피 경로 계획
            ROS_INFO_THROTTLE(1.0, "Obstacles detected. Planning with RRT*.");

            Node start_node(current_utm_.pose.position.x, current_utm_.pose.position.y);
            Node goal_node(local_path_.poses.back().pose.position.x, local_path_.poses.back().pose.position.y);

            std::vector<Node> path_nodes = planRRTStar(start_node, goal_node);

            if (path_nodes.empty()) {
                ROS_WARN("RRT* failed to find a path. Publishing empty path.");
            } else {
                std::reverse(path_nodes.begin(), path_nodes.end()); // 경로를 시작->목표 순으로 뒤집기
                std::vector<Node> smoothed_path = smoothPath(path_nodes); // 경로 부드럽게 만들기

                // 최종 경로를 nav_msgs::Path 형식으로 변환
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
    }

    // --- RRT* 헬퍼 함수들 (주석 추가) ---

    // 두 점 사이의 유클리드 거리 계산
    double getDistance(double x1, double y1, double x2, double y2) {
        return std::hypot(x1 - x2, y1 - y2);
    }

    // 두 노드를 잇는 경로가 장애물과 충돌하는지 확인
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
                return true; // 충돌
            }
        }
        return false; // 충돌 없음
    }

    // 트리에서 샘플 지점과 가장 가까운 노드 찾기
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

    // from 노드에서 to 노드 방향으로 STEP_SIZE 만큼 떨어진 새 노드 생성
    Node steer(const Node& from, const Node& to) {
        double dist = getDistance(from.x, from.y, to.x, to.y);
        if (dist < STEP_SIZE) {
            return to;
        }
        double ratio = STEP_SIZE / dist;
        return Node(from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio);
    }
    
    // RRT* 알고리즘의 메인 함수
    std::vector<Node> planRRTStar(const Node& start, const Node& goal) {
        std::vector<Node> tree;
        tree.push_back(start);

        // 샘플링 영역을 local_path 주변으로 한정하여 효율성 증대 (좋은 아이디어!)
        double min_x = start.x, max_x = start.x, min_y = start.y, max_y = start.y;
        for(const auto& pose : local_path_.poses){
            min_x = std::min(min_x, pose.pose.position.x);
            max_x = std::max(max_x, pose.pose.position.x);
            min_y = std::min(min_y, pose.pose.position.y);
            max_y = std::max(max_y, pose.pose.position.y);
        }
        min_x -= 5.0; max_x += 5.0; min_y -= 5.0; max_y += 5.0; // 여유 공간 추가

        std::uniform_real_distribution<double> distX(min_x, max_x);
        std::uniform_real_distribution<double> distY(min_y, max_y);

        for (int i = 0; i < MAX_ITERATIONS; ++i) {
            // 1. 랜덤 샘플 생성 (10% 확률로 목표점을 샘플링하여 수렴 속도 향상)
            Node random_sample = (rand() % 100 > 90) ? goal : Node(distX(rng_), distY(rng_));

            // 2. 가장 가까운 노드(nearest_node) 찾기
            int nearest_id = findNearestNode(tree, random_sample);
            Node nearest_node = tree[nearest_id];
            
            // 3. nearest_node에서 random_sample 방향으로 한 스텝 전진한 새 노드(new_node) 생성
            Node new_node = steer(nearest_node, random_sample);

            // 4. 새 노드로 가는 경로가 장애물과 충돌하는지 확인
            if (isCollision(nearest_node, new_node)) {
                continue;
            }

            // 5. RRT* : 최적의 부모 노드 찾기 (Choose Parent)
            // NEIGHBOR_RADIUS 반경 내의 이웃 노드들 중, new_node까지의 총 비용(cost)이 가장 낮은 노드를 부모로 선택
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
            // new_node를 경유했을 때, 다른 이웃 노드들의 비용이 더 저렴해진다면 부모를 new_node로 변경
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

        if(last_node_id != -1 && min_goal_dist < GOAL_RADIUS * 2) {
            int current_id = last_node_id;
            while(current_id != -1){
                path.push_back(tree[current_id]);
                current_id = tree[current_id].parent_id;
            }
        }
        return path; // 경로는 목표->시작 순서
    }

    // 경로를 부드럽게 만드는 함수
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

                if (!isCollision(smoothed_path[i-1], smoothed_path[i+1])) {
                     // 충돌하지 않을 때만 평탄화 적용 (옵션)
                }

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