#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h> // tf2::getYaw 사용을 위해 추가

#include <erp_driver/erpCmdMsg.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// --- 튜닝 파라미터 ---
// 차량의 축거 (wheelbase) (미터)
const double WHEELBASE = 2.7; 
// 목표 속도 (m/s)
const double VEHICLE_SPEED = 5.0; 
// Look-ahead 거리 계산을 위한 게인 값 (k * 속도)
const double LOOKAHEAD_K = 0.5;
// 최소/최대 Look-ahead 거리 (미터)
const double MIN_LOOKAHEAD = 4.0;
const double MAX_LOOKAHEAD = 10.0;


class PathFollower{
public:
    PathFollower(){
        ros::NodeHandle nh("~");
        path_sub_ = nh.subscribe("/avoid_path", 1, &PathFollower::pathCallback, this);
        utm_sub_ = nh.subscribe("/utm", 1, &PathFollower::utmCallback, this);
        drive_pub_ = nh.advertise<ackermann_msgs::AckermannDrive>("/drive", 1);
        erp_cmd_pub_ = nh.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd",1);
        
        // 50Hz (0.02초) 주기로 제어 루프 실행
        control_timer_ = nh.createTimer(ros::Duration(0.02), &PathFollower::controlLoopCallback, this);

        // 멤버 변수 초기화
        previous_utm_.header.stamp.fromSec(0); // 유효하지 않은 시간으로 초기화
        current_vehicle_yaw_ = 0.0;
    }

private:
    // ROS 핸들러
    ros::Subscriber path_sub_;
    ros::Subscriber utm_sub_;
    ros::Publisher drive_pub_;
    ros::Publisher erp_cmd_pub_;
    ros::Timer control_timer_;

    // 데이터 저장 변수
    nav_msgs::Path current_path_;
    geometry_msgs::PoseStamped current_utm_;
    geometry_msgs::PoseStamped previous_utm_; // 이전 UTM 위치 저장을 위한 변수
    double current_vehicle_yaw_; // 계산된 현재 차량의 yaw 값
    
    // 콜백: 데이터가 들어오면 멤버 변수에 저장
    void pathCallback(const nav_msgs::Path::ConstPtr& msg){
        current_path_ = *msg;
    }
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_utm_ = *msg;
    }

    /**
     * @brief 메인 제어 로직을 수행하는 타이머 콜백
     */
    void controlLoopCallback(const ros::TimerEvent&){
        // 경로 또는 현재 위치 정보가 없으면 정지 명령을 내리고 종료
        if (current_path_.poses.empty() || current_utm_.header.stamp.isZero()){
            publishDrive(0.0, 0.0); // 속도 0, 조향각 0
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


        // 2. 목표 지점(Target Point) 찾기
        double lookahead_dist = std::max(MIN_LOOKAHEAD, std::min(MAX_LOOKAHEAD, VEHICLE_SPEED * LOOKAHEAD_K));
        int target_idx = findTargetPointIndex(lookahead_dist);

        // 경로 끝에 도달했거나 적절한 목표점을 찾지 못하면 정지
        if (target_idx < 0) {
            publishDrive(0.0, 0.0);
            return;
        }
        geometry_msgs::Point target_point = current_path_.poses[target_idx].pose.position;

        // 3. Pure Pursuit 조향각 계산
        double steering_angle = calculateSteeringAngle(target_point);

        // 4. AckermannDrive 메시지 발행
        publishDrive(VEHICLE_SPEED, steering_angle);

        // 5. 다음 계산을 위해 현재 위치를 이전 위치로 업데이트
        previous_utm_ = current_utm_;
    }
    
    /**
     * @brief 현재 차량 위치에서 lookahead_dist 만큼 떨어진 경로상의 목표점 인덱스를 찾음
     * @param lookahead_dist - 미리보기 거리
     * @return 목표점의 인덱스. 찾지 못하면 -1.
     */
    int findTargetPointIndex(double lookahead_dist) {
        double vehicle_x = current_utm_.pose.position.x;
        double vehicle_y = current_utm_.pose.position.y;
        
        // 가장 가까운 경로점부터 탐색 시작
        int closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < current_path_.poses.size(); ++i) {
            double dx = vehicle_x - current_path_.poses[i].pose.position.x;
            double dy = vehicle_y - current_path_.poses[i].pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 가장 가까운 지점부터 시작하여 lookahead_dist 보다 멀리 있는 첫 번째 점을 찾음
        for (size_t i = closest_idx; i < current_path_.poses.size(); ++i) {
            double dx = vehicle_x - current_path_.poses[i].pose.position.x;
            double dy = vehicle_y - current_path_.poses[i].pose.position.y;
            double dist_to_point = std::sqrt(dx * dx + dy * dy);

            if (dist_to_point > lookahead_dist) {
                return i;
            }
        }
        // 경로 끝까지 탐색했는데도 못찾으면 -1 반환
        return -1; 
    }

    /**
     * @brief 목표점(전역 좌표)을 이용해 필요한 조향각을 계산
     * @param target_point - 지도(map) 기준의 목표점 좌표
     * @return 조향각 (라디안)
     */
    double calculateSteeringAngle(const geometry_msgs::Point& target_point) {
        // 1. 목표점을 차량 기준 좌표계로 변환
        double vehicle_x = current_utm_.pose.position.x;
        double vehicle_y = current_utm_.pose.position.y;
        double vehicle_yaw = current_vehicle_yaw_; // 위치 기반으로 계산된 yaw 사용

        double dx = target_point.x - vehicle_x;
        double dy = target_point.y - vehicle_y;
        
        // 변환 공식 적용
        double target_local_x = dx * std::cos(-vehicle_yaw) - dy * std::sin(-vehicle_yaw);
        double target_local_y = dx * std::sin(-vehicle_yaw) + dy * std::cos(-vehicle_yaw);

        // 2. Pure Pursuit 공식 적용
        // alpha: 차량의 현재 방향과 목표점을 잇는 선 사이의 각도
        double alpha = std::atan2(target_local_y, target_local_x);
        
        // Ld: 차량과 목표점 사이의 실제 거리
        double Ld = std::sqrt(dx*dx + dy*dy);

        // 스티어링 각(delta) = atan(2 * L * sin(alpha) / Ld)
        double steering_angle = std::atan2(2.0 * WHEELBASE * std::sin(alpha), Ld);

        return steering_angle;
    }

    /**
     * @brief AckermannDrive 메시지를 생성하고 발행
     * @param speed - 목표 속도 (m/s)
     * @param steering_angle - 목표 조향각 (라디안)
     */
    void publishDrive(double speed, double steering_angle){
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.speed = speed;
        drive_msg.steering_angle = steering_angle;
        drive_pub_.publish(drive_msg);

        erp_driver::erpCmdMsg cmd_msg;
        cmd_msg.brake = 1;
        cmd_msg.e_stop = false;
        cmd_msg.gear = 0;
        cmd_msg.speed = static_cast<uint8_t>(speed * 3600 / 1000 * 10);
        double steering_angle_degree = steering_angle * 180 / M_PI;
        cmd_msg.steer = -steering_angle_degree * 71; // sign inverse
        if(cmd_msg.steer>2000){
            cmd_msg.steer = 2000;
        }
        else if(cmd_msg.steer<-2000){
            cmd_msg.steer = -2000;
        }
        cmd_msg.steer = static_cast<int32_t>(cmd_msg.steer);
        erp_cmd_pub_.publish(cmd_msg);
        std::cout<<"steering_angle: "<<steering_angle<<" erp_angle: "<<cmd_msg.steer<<std::endl;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "path_follower_node");
    PathFollower follower;
    ros::spin();
    return 0;
}
