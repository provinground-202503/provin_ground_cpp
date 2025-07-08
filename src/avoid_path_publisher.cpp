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

// 사용자가 쉽게 조정할 수 있는 파라미터
// 장애물의 충돌 반경 (미터)
const double OBSTACLE_RADIUS = 0.5; 
// 장애물로부터 밀어낼 안전 거리 (미터). OBSTACLE_RADIUS보다 커야 합니다.
const double SAFE_DISTANCE = 1.0; 

class ObstacleAvoidePath{
public:
    ObstacleAvoidePath(){
        ros::NodeHandle nh("~");
        local_path_sub_ = nh.subscribe("/local_path", 1, &ObstacleAvoidePath::localPathCallback, this);
        utm_sub_ = nh.subscribe("/utm", 1, &ObstacleAvoidePath::utmCallback, this);
        // obstacle_sub_ = nh.subscribe("/obstacles", 1, &ObstacleAvoidePath::obstacleCallback, this);
        clustered_sub_ = nh.subscribe("/lidar_clusters",1,&ObstacleAvoidePath::clusteredCallback,this);
        avoid_path_pub_ = nh.advertise<nav_msgs::Path>("/avoid_path", 1);
        
        // 20Hz (0.05초마다) 주기로 pathPublishCallback 함수를 실행하여 회피 경로를 꾸준히 발행합니다.
        path_publish_timer_ = nh.createTimer(ros::Duration(0.05), &ObstacleAvoidePath::pathPublishCallback, this);

        // 멤버 변수 초기화
        previous_utm_.header.stamp.fromSec(0); // 유효하지 않은 시간으로 초기화
        current_vehicle_yaw_ = 0.0;

        ROS_INFO("Obstacle Avoidance Path node has been initialized.");
    }

private:
    // ROS 통신 핸들러
    ros::Subscriber local_path_sub_;
    ros::Subscriber utm_sub_;
    // ros::Subscriber obstacle_sub_;
    ros::Subscriber clustered_sub_;
    ros::Publisher avoid_path_pub_;
    ros::Timer path_publish_timer_;

    // 최신 데이터를 저장하기 위한 멤버 변수
    nav_msgs::Path local_path_;
    geometry_msgs::PoseStamped current_utm_;
    geometry_msgs::PoseStamped previous_utm_; // 이전 UTM 위치 저장을 위한 변수
    visualization_msgs::MarkerArray obstacles_;
    nav_msgs::Path final_avoid_path_; // 최종적으로 발행될 회피 경로
    double current_vehicle_yaw_; // 계산된 현재 차량의 yaw 값

    // 콜백 함수: 수신한 데이터를 해당 멤버 변수에 저장하는 역할만 수행
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg){
        local_path_ = *msg;
    }

    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_utm_ = *msg;
    }

    // void obstacleCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    //     if(msg==nullptr)return;
    //     obstacles_ = *msg;
    // }

    void clusteredCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
        if(msg==nullptr)return;
        obstacles_ = *msg;

        

    }

    /**
     * @brief 메인 로직을 처리하는 타이머 콜백 함수
     * 이 함수에서 좌표 변환, 충돌 감지, 경로 수정을 모두 수행합니다.
     */
    void pathPublishCallback(const ros::TimerEvent&){
        // 로컬 경로가 없거나, 현재 위치 정보가 없으면 아무것도 하지 않음
        if (local_path_.poses.empty() || current_utm_.header.stamp.isZero()){
            ROS_WARN_THROTTLE(1.0, "Waiting for local_path or utm message...");
            return;
        }

        // 이전 위치 정보가 유효하지 않으면 (첫 실행), 현재 위치를 이전 위치로 설정하고 종료
        if (previous_utm_.header.stamp.isZero()) {
            previous_utm_ = current_utm_;
            return;
        }

        // 1. 차량의 Yaw 계산 (위치 변화 기반)
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

        // 2. 장애물 좌표 변환 (base_link -> map)
        std::vector<geometry_msgs::Point> global_obstacles;
        for (const auto& marker : obstacles_.markers){
            double obs_local_x = marker.pose.position.x;
            double obs_local_y = marker.pose.position.y;

            geometry_msgs::Point global_obs_point;
            // 2D 회전 및 이동 변환: 계산된 current_vehicle_yaw_ 사용
            global_obs_point.x = vehicle_x + obs_local_x * std::cos(current_vehicle_yaw_) - obs_local_y * std::sin(current_vehicle_yaw_);
            global_obs_point.y = vehicle_y + obs_local_x * std::sin(current_vehicle_yaw_) + obs_local_y * std::cos(current_vehicle_yaw_);
            global_obs_point.z = current_utm_.pose.position.z;
            
            global_obstacles.push_back(global_obs_point);
        }

        // 3. 경로 수정 (밀어내기)
        nav_msgs::Path avoid_path = local_path_; 
        for (size_t i = 0; i < avoid_path.poses.size(); ++i){
            auto& waypoint = avoid_path.poses[i].pose.position;
            for (const auto& obs_point : global_obstacles){
                double dist = std::sqrt(std::pow(waypoint.x - obs_point.x, 2) + std::pow(waypoint.y - obs_point.y, 2));
                if (dist < OBSTACLE_RADIUS){
                    double vec_x = waypoint.x - obs_point.x;
                    double vec_y = waypoint.y - obs_point.y;
                    double norm = std::sqrt(vec_x*vec_x + vec_y*vec_y);
                    if (norm > 0.001){
                        vec_x /= norm;
                        vec_y /= norm;
                    }
                    waypoint.x = obs_point.x + vec_x * SAFE_DISTANCE;
                    waypoint.y = obs_point.y + vec_y * SAFE_DISTANCE;
                }
            }
        }
        
        // 4. 최종 경로 발행
        final_avoid_path_ = avoid_path;
        final_avoid_path_.header.stamp = ros::Time::now();
        final_avoid_path_.header.frame_id = "map";
        avoid_path_pub_.publish(final_avoid_path_);

        // 5. 다음 계산을 위해 현재 위치를 이전 위치로 업데이트
        previous_utm_ = current_utm_;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_avoid_path_node");
    ObstacleAvoidePath avoider;
    ros::spin();
    return 0;
}
