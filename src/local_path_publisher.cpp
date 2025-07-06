#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
#include <cmath> // std::sqrt, std::pow 사용을 위해 추가
#include <limits> // std::numeric_limits 사용을 위해 추가

class LocalPathPublisher{
public:
    LocalPathPublisher(){
        // ROS 노드 핸들 초기화
        ros::NodeHandle nh("~");

        // Subscriber 설정: /global_path와 /utm 토픽을 구독합니다.
        global_path_sub_ = nh.subscribe("/global_path", 1, &LocalPathPublisher::globalPathCallback, this);
        utm_sub_ = nh.subscribe("/utm", 1, &LocalPathPublisher::utmCallback, this);

        // Publisher 설정: /local_path 토픽으로 메시지를 발행합니다.
        local_path_pub_ = nh.advertise<nav_msgs::Path>("/local_path", 1);

        // 이전 위치 초기값 설정 (임의의 값)
        previous_x_ = 360000;
        previous_y_ = 4000000;
        current_x_ = 360000;
        current_y_ = 4000000;

        ROS_INFO("Local Path Publisher node has been initialized.");
    }

    // global_path 토픽을 수신했을 때 호출되는 콜백 함수
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg){
        global_path_ = *msg;
        // ROS_INFO("Received global path with %zu poses.", msg->poses.size());
    }

    // utm 토픽을 수신했을 때 호출되는 콜백 함수
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        // 현재 UTM 좌표 업데이트
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;

        // (주석 처리된 부분) 이전 좌표와 현재 좌표를 이용해 현재 차량의 yaw 추정 가능
        // double current_yaw = std::atan2(current_y_ - previous_y_, current_x_ - previous_x_);
        // previous_x_ = current_x_;
        // previous_y_ = current_y_;

        // 수신된 global_path가 비어있으면 로직을 실행하지 않음
        if (global_path_.poses.empty()){
            ROS_WARN_THROTTLE(1.0, "Global path is empty. Waiting for global path...");
            return;
        }

        // --- 현재 위치에서 가장 가까운 global_path 상의 점 찾기 ---
        double min_dist = std::numeric_limits<double>::max();
        int closest_waypoint_index = -1;

        for (int i = 0; i < global_path_.poses.size(); ++i){
            const auto& waypoint = global_path_.poses[i].pose.position;
            // 현재 위치와 웨이포인트 사이의 유클리드 거리 계산
            double dist = std::sqrt(std::pow(current_x_ - waypoint.x, 2) + std::pow(current_y_ - waypoint.y, 2));

            if (dist < min_dist){
                min_dist = dist;
                closest_waypoint_index = i;
            }
        }

        // --- Local Path 생성 및 발행 ---
        nav_msgs::Path local_path;
        local_path.header.stamp = ros::Time::now();
        local_path.header.frame_id = "map"; // global_path와 동일한 프레임 ID 사용

        if (closest_waypoint_index != -1){
            // 가장 가까운 지점부터 50개의 웨이포인트를 local_path에 추가
            int end_index = closest_waypoint_index + 50;

            // 경로의 끝을 넘어가지 않도록 인덱스 조정
            if (end_index >= global_path_.poses.size()){
                end_index = global_path_.poses.size();
            }

            // global_path_에서 local_path로 경로점 복사
            for (int i = closest_waypoint_index; i < end_index; ++i){
                local_path.poses.push_back(global_path_.poses[i]);
            }
        }

        // 생성된 local_path 발행
        if (!local_path.poses.empty()){
            local_path_pub_.publish(local_path);
        }
    }

private:
    // ROS 통신 핸들러
    ros::Subscriber global_path_sub_;
    ros::Subscriber utm_sub_;
    ros::Publisher local_path_pub_;

    // 데이터 저장 변수
    nav_msgs::Path global_path_;
    double current_x_, current_y_;
    double previous_x_, previous_y_;
};

int main(int argc, char** argv){
    // ROS 시스템 초기화 및 노드 이름 설정
    ros::init(argc, argv, "local_path_publisher_node");

    // LocalPathPublisher 클래스 인스턴스 생성
    LocalPathPublisher path_publisher;

    // ROS 메시지 콜백 대기
    ros::spin();

    return 0;
}
