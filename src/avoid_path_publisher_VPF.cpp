#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h> // tf2::getYaw 사용을 위해 추가
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm> // std::min, std::max 사용
#include <limits>    // std::numeric_limits 사용

// --- 포텐셜 필드 파라미터 설정 ---
// 장애물의 충돌 반경 (미터) - 로봇 반경으로 사용
const double ROBOT_RADIUS = 0.3; 
// 장애물로부터 척력이 시작되는 거리 (미터). 이 거리보다 가까워야 척력이 발생합니다.
const double REPULSIVE_FIELD_RADIUS = 1.0; 
// 목표 지점으로부터 인력이 작용하는 거리. (그리드 맵 크기에 따라 조절)
const double ATTRACTIVE_FIELD_RADIUS = 5.0; // 예시 값, 로컬 경로 길이 고려

// 포텐셜 필드 가중치
const double ALPHA_ATTRACTIVE = 0.5; // 인력 계수
const double BETA_REPULSIVE = 10.0;  // 척력 계수 (척력은 보통 더 강하게 작용)

// 경로 조정 강도 (계산된 힘을 웨이포인트 이동에 얼마나 반영할지)
const double PATH_ADJUST_STRENGTH = 0.1; // 너무 크면 경로가 불안정해질 수 있음

// ---

class PotentialFieldPathGenerator{
public:
    PotentialFieldPathGenerator(){
        ros::NodeHandle nh("~");
        local_path_sub_ = nh.subscribe("/local_path", 1, &PotentialFieldPathGenerator::localPathCallback, this);
        utm_sub_ = nh.subscribe("/utm", 1, &PotentialFieldPathGenerator::utmCallback, this);
        clustered_sub_ = nh.subscribe("/lidar_clusters", 1, &PotentialFieldPathGenerator::clusteredCallback, this);
        avoid_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);
        
        // 20Hz (0.05초마다) 주기로 pathGenerationCallback 함수를 실행하여 회피 경로를 꾸준히 발행합니다.
        path_generation_timer_ = nh.createTimer(ros::Duration(0.05), &PotentialFieldPathGenerator::pathGenerationCallback, this);

        // 멤버 변수 초기화
        current_utm_.header.stamp = ros::Time(0); 
        previous_utm_.header.stamp = ros::Time(0); 
        current_vehicle_yaw_ = 0.0;

        ROS_INFO("Potential Field Path Generator node has been initialized.");
    }

private:
    // ROS 통신 핸들러
    ros::Subscriber local_path_sub_;
    ros::Subscriber utm_sub_;
    ros::Subscriber clustered_sub_;
    ros::Publisher avoid_path_pub_;
    ros::Timer path_generation_timer_;

    // 최신 데이터를 저장하기 위한 멤버 변수
    nav_msgs::Path local_path_;
    geometry_msgs::PoseStamped current_utm_;
    geometry_msgs::PoseStamped previous_utm_; // 이전 UTM 위치 저장을 위한 변수 (Yaw 계산용)
    visualization_msgs::MarkerArray obstacles_; // base_link 기준 장애물 마커들
    double current_vehicle_yaw_; // 계산된 현재 차량의 yaw 값

    // 콜백 함수: 수신한 데이터를 해당 멤버 변수에 저장하는 역할만 수행
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg){
        local_path_ = *msg;
        // ROS_INFO_THROTTLE(1.0, "Local path subscribed. Waypoints: %zu", local_path_.poses.size());
    }

    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_utm_ = *msg;
        // ROS_INFO_THROTTLE(1.0, "UTM subscribed. Position: (%.2f, %.2f)", current_utm_.pose.position.x, current_utm_.pose.position.y);
    }

    void clusteredCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
        obstacles_ = *msg; // 장애물은 로봇의 base_link 기준 좌표로 들어올 것으로 가정합니다.
        // ROS_INFO_THROTTLE(1.0, "Clustered obstacles subscribed. Count: %zu", obstacles_.markers.size());
    }

    /**
     * @brief 인력 계산 함수
     * @param current_point 현재 웨이포인트의 위치 (map 프레임)
     * @param target_point 목표 웨이포인트의 위치 (map 프레임)
     * @return 계산된 인력 벡터
     */
    geometry_msgs::Point calculateAttractiveForce(const geometry_msgs::Point& current_point, 
                                                 const geometry_msgs::Point& target_point) {
        geometry_msgs::Point force;
        double dx = target_point.x - current_point.x;
        double dy = target_point.y - current_point.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist > ATTRACTIVE_FIELD_RADIUS) {
            // 영향 범위를 벗어나면, 최대 인력을 적용하거나 (또는 0으로)
            // 여기서는 목표 방향으로 단위 벡터 * 최대 인력
            force.x = ALPHA_ATTRACTIVE * dx / dist;
            force.y = ALPHA_ATTRACTIVE * dy / dist;
        } else {
            // 영향 범위 내에서는 거리에 비례하여 인력 적용
            force.x = ALPHA_ATTRACTIVE * dx;
            force.y = ALPHA_ATTRACTIVE * dy;
        }
        force.z = 0.0; // 2D 평면이므로 Z는 0
        return force;
    }

    /**
     * @brief 척력 계산 함수
     * @param current_point 현재 웨이포인트의 위치 (map 프레임)
     * @param obstacle_point 장애물 위치 (map 프레임)
     * @return 계산된 척력 벡터
     */
    geometry_msgs::Point calculateRepulsiveForce(const geometry_msgs::Point& current_point, 
                                                const geometry_msgs::Point& obstacle_point) {
        geometry_msgs::Point force;
        double dx = current_point.x - obstacle_point.x;
        double dy = current_point.y - obstacle_point.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // 척력은 장애물과의 거리가 REPULSIVE_FIELD_RADIUS보다 작을 때만 발생
        if (dist < REPULSIVE_FIELD_RADIUS && dist > 0.001) { // 0으로 나누는 것 방지
            // 거리가 가까울수록 척력이 급격히 증가 (1/dist^2 형태)
            double magnitude = BETA_REPULSIVE * (1.0/dist - 1.0/REPULSIVE_FIELD_RADIUS) * (1.0/(dist*dist));
            force.x = magnitude * dx / dist;
            force.y = magnitude * dy / dist;
        } else {
            force.x = 0.0;
            force.y = 0.0;
        }
        force.z = 0.0;
        return force;
    }

    /**
     * @brief 메인 로직을 처리하는 타이머 콜백 함수 (포텐셜 필드 기반 경로 생성)
     */
    void pathGenerationCallback(const ros::TimerEvent&){
        // 이전 위치 정보가 유효하지 않으면 (첫 실행), 현재 위치를 이전 위치로 설정하고 종료
        if (previous_utm_.header.stamp.isZero() || current_utm_.header.stamp.isZero()) {
            previous_utm_ = current_utm_;
            ROS_WARN_THROTTLE(1.0, "Waiting for initial UTM data to calculate yaw.");
            return;
        }

        // 1. 차량의 Yaw 계산 (위치 변화 기반)
        // current_utm_의 쿼터니언이 유효하지 않으므로, 이전 UTM과의 위치 변화를 기반으로 Yaw 계산
        double vehicle_x = current_utm_.pose.position.x;
        double vehicle_y = current_utm_.pose.position.y;
        double prev_vehicle_x = previous_utm_.pose.position.x;
        double prev_vehicle_y = previous_utm_.pose.position.y;

        double delta_x = vehicle_x - prev_vehicle_x;
        double delta_y = vehicle_y - prev_vehicle_y;

        // 차량이 일정 거리(1cm) 이상 움직였을 때만 yaw 값을 새로 계산
        if (std::sqrt(delta_x * delta_x + delta_y * delta_y) > 0.01) {
            current_vehicle_yaw_ = std::atan2(delta_y, delta_x);
        }
        // 움직임이 거의 없을 경우, 이전에 계산된 yaw 값을 그대로 사용 (안정성 확보)

        // 필수 데이터가 없으면 경로 생성을 실행하지 않습니다.
        if (local_path_.poses.empty()){
            ROS_WARN_THROTTLE(1.0, "Waiting for local_path message for Potential Field.");
            return;
        }

        nav_msgs::Path avoid_path;
        avoid_path.header.stamp = ros::Time::now();
        avoid_path.header.frame_id = "map";

        // 로컬 경로의 첫 번째 웨이포인트부터 시작하여 힘을 적용합니다.
        // 첫 번째 웨이포인트는 현재 로봇 위치에 가까울 것이므로, 그대로 사용하거나 아주 약간만 조정합니다.
        // 보통은 로봇 현재 위치를 새로운 경로의 시작점으로 포함하고 시작합니다.
        
        geometry_msgs::PoseStamped current_robot_pose = current_utm_;
        current_robot_pose.header.frame_id = "map"; // 현재 로봇 위치는 항상 포함
        avoid_path.poses.push_back(current_robot_pose);

        for (size_t i = 0; i < local_path_.poses.size(); ++i){
            geometry_msgs::PoseStamped original_waypoint_pose = local_path_.poses[i];
            geometry_msgs::Point adjusted_point = original_waypoint_pose.pose.position;

            // 2. 장애물 좌표 변환 (base_link -> map)
            // 현재 웨이포인트에 대해 장애물 척력을 계산하기 위해, 장애물 위치를 map 프레임으로 변환
            std::vector<geometry_msgs::Point> global_obstacles;
            for (const auto& marker : obstacles_.markers){
                geometry_msgs::Point global_obs_point;
                // 장애물 마커 위치는 base_link 기준이므로, current_utm_과 current_vehicle_yaw_를 이용해 map 프레임으로 변환
                global_obs_point.x = current_utm_.pose.position.x + marker.pose.position.x * std::cos(current_vehicle_yaw_) - marker.pose.position.y * std::sin(current_vehicle_yaw_);
                global_obs_point.y = current_utm_.pose.position.y + marker.pose.position.x * std::sin(current_vehicle_yaw_) + marker.pose.position.y * std::cos(current_vehicle_yaw_);
                global_obs_point.z = marker.pose.position.z;
                global_obstacles.push_back(global_obs_point);
            }

            // 3. 인력 계산 (로컬 경로의 다음 유효한 웨이포인트를 목표로 설정)
            // local_path_의 현재 웨이포인트를 목표로 하는 인력 계산 (이 웨이포인트가 로봇이 도달해야 할 방향)
            geometry_msgs::Point attractive_force = calculateAttractiveForce(adjusted_point, original_waypoint_pose.pose.position);

            // 4. 척력 계산 (모든 장애물로부터의 척력 합산)
            geometry_msgs::Point total_repulsive_force;
            total_repulsive_force.x = 0.0;
            total_repulsive_force.y = 0.0;
            total_repulsive_force.z = 0.0;

            for (const auto& obs_point : global_obstacles){
                geometry_msgs::Point repulsive_force = calculateRepulsiveForce(adjusted_point, obs_point);
                total_repulsive_force.x += repulsive_force.x;
                total_repulsive_force.y += repulsive_force.y;
            }

            // 5. 합성 힘 계산 및 웨이포인트 조정
            adjusted_point.x += (attractive_force.x + total_repulsive_force.x) * PATH_ADJUST_STRENGTH;
            adjusted_point.y += (attractive_force.y + total_repulsive_force.y) * PATH_ADJUST_STRENGTH;

            // 조정된 웨이포인트를 새로운 경로에 추가
            geometry_msgs::PoseStamped new_pose_stamped;
            new_pose_stamped.header = original_waypoint_pose.header;
            new_pose_stamped.pose.position = adjusted_point;
            // Yaw 정보는 원래 경로의 것을 그대로 사용하거나, 조정된 위치를 기반으로 계산할 수 있습니다.
            // 여기서는 간단히 원래 경로의 Yaw를 유지합니다.
            new_pose_stamped.pose.orientation = original_waypoint_pose.pose.orientation; 
            
            avoid_path.poses.push_back(new_pose_stamped);
        }
        
        // 6. 최종 경로 발행
        avoid_path_pub_.publish(avoid_path);

        // 7. 다음 계산을 위해 현재 위치를 이전 위치로 업데이트
        previous_utm_ = current_utm_;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "potential_field_path_generator_node");
    PotentialFieldPathGenerator planner;
    ros::spin();
    return 0;
}