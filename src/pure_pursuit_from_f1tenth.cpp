#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h> // pitch with break

#include <tf2/utils.h>

// C++ 표준 라이브러리
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <algorithm>

// 웨이포인트 구조체 정의
struct waypoint {
    double x;
    double y;
};

class PurePursuit {
private:
    // ROS 핸들 및 퍼블리셔/서브스크라이버
    ros::NodeHandle nh_;
    ros::Subscriber utm_sub_;
    ros::Subscriber erp_status_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher erp_cmd_pub_;
    ros::Publisher ack_pub_;
    ros::Publisher path_pub_;
    ros::Timer control_timer_;

    // 경로 및 차량 상태
    std::vector<waypoint> waypoint_vector_;
    int target_index_;
    double curr_x_, curr_y_, yaw_;
    double prev_x_, prev_y_; // 이전 위치 저장을 위한 변수
    
    // UTM 좌표 오프셋
    double utm_offset_x_;
    double utm_offset_y_;

    // Pure Pursuit 파라미터
    double car_length_;
    double look_ahead_distance_;

    // erp status parameter
    double current_speed_erp_, current_speed_mps_, current_speed_kph_;
    double current_steer_erp_, current_steer_rad_, current_steer_deg_;
    double erp_status_dt_;
    double prev_error_mps_, error_integral_;

    uint8_t emergency_brake_;

    /**
     * @brief CSV 파일에서 웨이포인트를 로드하고 UTM 오프셋을 적용합니다.
     */
    void loadWaypoints() {
        std::string file_path = "/root/erp42_ws/src/provin_ground_cpp/path/path_morai_xy.csv";
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_FATAL("경로 파일을 열 수 없습니다: %s", file_path.c_str());
            ros::shutdown();
            return;
        }

        waypoint_vector_.clear();
        std::string line;
        std::getline(file, line); // 헤더 라인 건너뛰기

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            waypoint guide;

            std::getline(ss, token, ',');
            guide.x = std::stod(token) - utm_offset_x_;
            std::getline(ss, token, ',');
            guide.y = std::stod(token) - utm_offset_y_;
            
            waypoint_vector_.push_back(guide);
        }
        file.close();
        ROS_INFO("%zu개의 웨이포인트를 로컬 원점 기준으로 로드했습니다.", waypoint_vector_.size());
    }

    /**
     * @brief 로드된 경로를 Rviz 등에서 시각화하기 위해 발행합니다.
     */
    void publishPath() {
        if (waypoint_vector_.empty()) return;

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        for (const auto& waypoint : waypoint_vector_) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = waypoint.x;
            pose.pose.position.y = waypoint.y;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);
    }

    /**
     * @brief UTM 좌표를 수신하여 로컬 좌표로 변환하고, 이전 위치와 비교하여 yaw를 계산합니다.
     */
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // UTM 좌표에 오프셋을 적용하여 로컬 좌표로 변환
        curr_x_ = msg->pose.position.x - utm_offset_x_;
        curr_y_ = msg->pose.position.y - utm_offset_y_;
        
        // ❗ 이전 위치와 현재 위치의 변화량 계산
        double delta_x = curr_x_ - prev_x_;
        double delta_y = curr_y_ - prev_y_;

        // ❗ 차량이 일정 거리(1cm) 이상 움직였을 때만 yaw를 갱신
        if (std::sqrt(delta_x * delta_x + delta_y * delta_y) > 0.01) {
            yaw_ = std::atan2(delta_y, delta_x);
        }

        // ❗ 현재 위치를 다음 계산을 위해 이전 위치로 저장
        prev_x_ = curr_x_;
        prev_y_ = curr_y_;
    }

    void erpStatusCallback(const erp_driver::erpStatusMsg::ConstPtr& msg) {
        
        current_speed_erp_ = msg->speed;
        current_steer_erp_ = msg->steer;

        current_speed_kph_ = current_speed_erp_ / 10;
        current_speed_mps_ = current_speed_kph_ / 3.6;

        current_steer_deg_ = static_cast<double>(current_steer_erp_) / 71.0;
        current_steer_rad_ = current_steer_deg_ * M_PI / 180.0;

    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        if(pitch>=std::tan(0.05)) emergency_brake_ = 100; // mountain or roadblock
        else emergency_brake_ = 1; //
    }
    
    /**
     * @brief 제어 로직을 주기적으로 실행하는 타이머 콜백 함수입니다.
     */
    void controlLoop(const ros::TimerEvent&) {
        if (waypoint_vector_.empty() || std::isnan(curr_x_) || std::isnan(curr_y_)) {
            ROS_WARN_THROTTLE(1.0, "경로가 비어있거나 현재 위치를 알 수 없습니다.");
            return;
        }

        // 1. 차량과 가장 가까운 웨이포인트 찾기
        int closest_index = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < waypoint_vector_.size(); ++i) {
            double dx = curr_x_ - waypoint_vector_[i].x;
            double dy = curr_y_ - waypoint_vector_[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }
        if(closest_index>=920 && closest_index<=1060) look_ahead_distance_=1.05; // s course
        else look_ahead_distance_ = 2.0;

        // 2. 가장 가까운 지점부터 시작하여 Look-ahead-distance를 만족하는 목표 지점 탐색
        target_index_ = -1;
        for (size_t i = closest_index; i < waypoint_vector_.size(); ++i) {
            double dx_target = curr_x_ - waypoint_vector_[i].x;
            double dy_target = curr_y_ - waypoint_vector_[i].y;
            double dist_to_vehicle = std::sqrt(dx_target * dx_target + dy_target * dy_target);

            if (dist_to_vehicle >= look_ahead_distance_) {
                target_index_ = i;
                break;
            }
        }


        
        // 경로의 끝에 도달하여 목표 지점을 찾지 못한 경우, 차량 정지
        if (target_index_ == -1) {
            ROS_INFO("경로의 끝에 도달했습니다. 차량을 정지합니다.");
            publishCommands(0.0, 0.0);
            return;
        }

        // 3. 목표 지점을 차량의 로컬 좌표계(base_link)로 변환
        waypoint target_waypoint = waypoint_vector_[target_index_];
        double target_x_local = (target_waypoint.x - curr_x_) * std::cos(yaw_) + (target_waypoint.y - curr_y_) * std::sin(yaw_);
        double target_y_local = -(target_waypoint.x - curr_x_) * std::sin(yaw_) + (target_waypoint.y - curr_y_) * std::cos(yaw_);

        // 4. 변환된 좌표를 이용하여 조향각 계산
        double ld_sq = target_x_local * target_x_local + target_y_local * target_y_local;
        double steering_angle = std::atan2(2.0 * car_length_ * target_y_local, ld_sq);
        
        double target_speed_mps = 1.5;
        publishCommands(target_speed_mps, steering_angle);
        
        ROS_INFO("Target Index: %d, Steering Angle (rad): %.2f", target_index_, steering_angle);
    }
    
    /**
     * @brief 계산된 속도와 조향각으로 Ackermann 및 ERP42 제어 메시지를 발행합니다.
     */
    void publishCommands(double target_speed_mps, double steer_angle) {

        // pid latitudinal speed
        double p_gain = 0.7;
        double i_gain = 0.0;
        double d_gain = 0.005;
        double error_speed = target_speed_mps - current_speed_mps_;
        double p_term = p_gain * error_speed;
        error_integral_ += error_speed * erp_status_dt_;
        double i_term = i_gain * error_integral_;
        double d_term = d_gain * (error_speed - prev_error_mps_) / erp_status_dt_;
        prev_error_mps_ = error_speed;
        double output_mps_ = p_term + i_term + d_term;

        ackermann_msgs::AckermannDriveStamped ack_msg;
        ack_msg.header.stamp = ros::Time::now();
        ack_msg.header.frame_id = "base_link";
        ack_msg.drive.steering_angle = steer_angle;
        // ack_msg.drive.speed = target_speed_mps;
        ack_msg.drive.speed = output_mps_;
        ack_pub_.publish(ack_msg);

        erp_driver::erpCmdMsg cmd_msg;
        cmd_msg.e_stop = false;
        cmd_msg.gear = (target_speed_mps == 0) ? 1 : 0;
        if(current_speed_mps_ > target_speed_mps){cmd_msg.speed = 0;}
        else cmd_msg.speed = static_cast<uint8_t>(output_mps_ * 3.6 * 10.0);
        cmd_msg.brake = (target_speed_mps == 0) ? 200 : emergency_brake_;

        double steering_angle_degree = steer_angle * 180.0 / M_PI;
        cmd_msg.steer = -static_cast<int32_t>(steering_angle_degree * 71.0);

        cmd_msg.steer = std::max(-2000, std::min(2000, (int)cmd_msg.steer));
        erp_cmd_pub_.publish(cmd_msg);
        std::cout<<"steering_angle: "<<steer_angle<<" erp_angle: "<<cmd_msg.steer<<std::endl;
    }

    /**
     * @brief 클래스 초기화 함수
     */
    void initialize() {
        // utm_offset_x_ = 360777.923575;
        // utm_offset_y_ = 4065980.612646;
        utm_offset_x_ = 0;
        utm_offset_y_ = 0;


        loadWaypoints();

        target_index_ = 0;
        car_length_ = 1.04;
        look_ahead_distance_ = 2.0;
        
        // 로컬 좌표 기준 (0,0)으로 초기화
        curr_x_ = 0.0;
        curr_y_ = 0.0;
        prev_x_ = 0.0;
        prev_y_ = 0.0;
        yaw_ = 0.0;
        
        publishPath();
    }

public:
    PurePursuit() : nh_("~") {
        utm_sub_ = nh_.subscribe("/utm_fix", 1, &PurePursuit::utmCallback, this);
        erp_status_sub_ = nh_.subscribe("/erp42_status", 1, &PurePursuit::erpStatusCallback, this);
        imu_sub_ = nh_.subscribe("/imu/fix",1,&PurePursuit::imuCallback,this);
        yolo_id_sub_ = nh_.subscribe()

        erp_cmd_pub_ = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd", 1);
        ack_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
        
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / 20.0), &PurePursuit::controlLoop, this);
        
        initialize();
        erp_status_dt_ = 0.025;
        error_integral_ = 0;
        prev_error_mps_ = 0;
        emergency_brake_ = 1;
    }
    ~PurePursuit() = default;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit");
    PurePursuit pp;
    ros::spin();
    return 0;
}