#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> // To subscribe to the local path
#include <sensor_msgs/Imu.h> // pitch with break
#include <yolo/YoloDetection.h> // YoloDetection 메시지
#include <yolo/YoloDetectionArray.h> // YoloDetectionArray 메시지

#include <tf2/utils.h> // tf2::getYaw 사용을 위해 추가
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>

double MAX_VEHICLE_SPEED = 1.3; // m/s

// 웨이포인트 구조체 정의 (공통 헤더 파일로 분리하는 것이 이상적)
struct waypoint {
    double x;
    double y;
};

class PurePursuitPID {
private:
    // ROS 핸들 및 퍼블리셔/서브스크라이버
    ros::NodeHandle nh_;
    ros::Subscriber utm_sub_;
    ros::Subscriber erp_status_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber local_path_sub_; // Subscriber for LOCAL path
    ros::Subscriber yolo_sub_; // Yolo detection subscriber
    ros::Subscriber start_sub_;

    ros::Publisher erp_cmd_pub_;
    ros::Publisher ack_pub_;
    ros::Timer control_timer_;

    // 경로 및 차량 상태
    std::vector<waypoint> current_local_path_; // Local path received from local_path_calculate
    int target_index_;
    double curr_x_, curr_y_, yaw_; // 현재 차량 위치 및 yaw (atan2로 계산)
    geometry_msgs::PoseStamped previous_utm_; // 이전 UTM 위치 저장을 위한 변수 (yaw 계산용)
    
    // Pure Pursuit 파라미터
    const double CAR_LENGTH = 1.04; // 차량의 축거 (wheelbase) (미터)
    double dynamic_look_ahead_distance_; // 동적으로 계산되는 Look-ahead 거리

    // Look-ahead 거리 계산을 위한 튜닝 파라미터
    const double LOOKAHEAD_K = 0.3;
    const double MIN_LOOKAHEAD = 1.0;
    const double MAX_LOOKAHEAD = 2.0;

    // ERP status parameter
    double current_speed_erp_, current_speed_mps_, current_speed_kph_;
    double current_steer_erp_, current_steer_rad_, current_steer_deg_;
    double erp_status_dt_; // Assuming a fixed dt for ERP status updates
    double prev_error_mps_, error_integral_;

    // Yolo 관련 변수
    double target_vehicle_speed_mps_; // Yolo 감지에 따라 동적으로 변하는 목표 속도
    uint8_t emergency_brake_; // IMU 피치에 따른 비상 브레이크
    
    // UTM 좌표 오프셋
    const double UTM_OFFSET_X = 360777.923575;
    const double UTM_OFFSET_Y = 4065980.612646;

    bool START_SIGNAL;

    /**
     * @brief 로컬 경로를 수신합니다.
     */
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        current_local_path_.clear();
        for (const auto& pose_stamped : msg->poses) {
            current_local_path_.push_back({pose_stamped.pose.position.x, pose_stamped.pose.position.y});
        }
        // ROS_INFO_ONCE("로컬 경로를 수신했습니다. 웨이포인트 수: %zu", current_local_path_.size()); // Use ONCE or THROTTLE
    }

    /**
     * @brief UTM 좌표를 수신하여 로컬 좌표로 변환하고, 이전 위치와 비교하여 yaw를 계산합니다.
     */
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // UTM 좌표에 오프셋을 적용하여 로컬 좌표로 변환
        curr_x_ = msg->pose.position.x - UTM_OFFSET_X;
        curr_y_ = msg->pose.position.y - UTM_OFFSET_Y;
        
        // ❗ 이전 위치 정보가 유효하지 않으면 (첫 실행 또는 초기화), 현재 위치를 이전 위치로 설정
        if (previous_utm_.header.stamp.isZero()) {
            previous_utm_ = *msg; // 전체 메시지 저장 (오리엔테이션, 스탬프 등 포함)
            previous_utm_.pose.position.x = curr_x_; // 오프셋 적용된 값으로 저장
            previous_utm_.pose.position.y = curr_y_;
            return;
        }

        // ❗ 이전 위치와 현재 위치의 변화량 계산
        double delta_x = curr_x_ - (previous_utm_.pose.position.x - UTM_OFFSET_X);
        double delta_y = curr_y_ - (previous_utm_.pose.position.y - UTM_OFFSET_Y);

        // ❗ 차량이 일정 거리(1cm) 이상 움직였을 때만 yaw를 갱신
        if (std::sqrt(delta_x * delta_x + delta_y * delta_y) > 0.01) {
            yaw_ = std::atan2(delta_y, delta_x);
        }
        // 움직임이 거의 없을 경우, 이전에 계산된 yaw 값을 그대로 사용 (안정성 확보)

        // ❗ 현재 위치를 다음 계산을 위해 이전 위치로 저장
        previous_utm_ = *msg;
        previous_utm_.pose.position.x = curr_x_; // 오프셋 적용된 값으로 저장
        previous_utm_.pose.position.y = curr_y_;
    }

    void erpStatusCallback(const erp_driver::erpStatusMsg::ConstPtr& msg) {
        current_speed_erp_ = msg->speed;
        current_steer_erp_ = msg->steer;

        current_speed_kph_ = current_speed_erp_ / 10.0;
        current_speed_mps_ = current_speed_kph_ / 3.6;

        current_steer_deg_ = static_cast<double>(current_steer_erp_) / 71.0;
        current_steer_rad_ = current_steer_deg_ * M_PI / 180.0;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw_imu;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_imu);
        // 피치 각도에 따라 비상 브레이크 결정 (탄젠트 값으로 비교)
        if(std::tan(pitch) < -0.05){ // -0.05 라디안 (약 -2.86도) 보다 낮아지면 (차량이 아래로 기울면)
            emergency_brake_ = 100; // 강하게 브레이크
        }
        else {
            emergency_brake_ = 1; // 기본 브레이크 값
        }
        // std::cout<<"pitch (rad): "<<pitch<<", tan(pitch): "<<std::tan(pitch)<<", emergency_brake:"<<(int)emergency_brake_<<std::endl;
    }

    void yoloCallback(const yolo::YoloDetectionArray::ConstPtr& msg){
        // std::cout<<"yolo subscribed"<<std::endl;
        
        bool redlight_detected = false;
        bool yellowlight_detected=false;
        bool person_detected = false;
        bool barrel_detected = false;

        for(const auto& object : msg->detections){ // msg->detections 로 직접 접근
            if(object.class_name=="redlight"){
                redlight_detected = true;
                break; // 빨간불이 감지되면 다른 것은 더 이상 확인하지 않음
            }
            if(object.class_name=="yellowlight"){
                yellowlight_detected = true;
                // yellowlight가 redlight보다 우선순위가 낮으므로, redlight가 없어야 유효
            } 
            if(object.class_name=="person"){
                person_detected = true;
            }
            if(object.class_name=="barrel"){
                barrel_detected = true;
            }
        }
        
        // 우선순위가 높은 것부터 속도 결정
        if(redlight_detected) {
            target_vehicle_speed_mps_ = 0.0;
            // std::cout << "Redlight detected: Speed set to 0.0 m/s" << std::endl;
        } else if(yellowlight_detected) {
            target_vehicle_speed_mps_ = MAX_VEHICLE_SPEED * 0.3; // MAX_VEHICLE_SPEED 대신 상수 1.3 사용
            // std::cout << "Yellowlight detected: Speed set to " << target_vehicle_speed_mps_ << " m/s" << std::endl;
        } else if(person_detected) {
            target_vehicle_speed_mps_ = MAX_VEHICLE_SPEED * 0.5; // MAX_VEHICLE_SPEED 대신 상수 1.3 사용
            // std::cout << "Person detected: Speed set to " << target_vehicle_speed_mps_ << " m/s" << std::endl;
        } else if(barrel_detected){
            target_vehicle_speed_mps_ = MAX_VEHICLE_SPEED * 0.7; // MAX_VEHICLE_SPEED 대신 상수 1.3 사용
            // std::cout << "Barrel detected: Speed set to " << target_vehicle_speed_mps_ << " m/s" << std::endl;
        } else {
            target_vehicle_speed_mps_ = MAX_VEHICLE_SPEED * 1.0; // MAX_VEHICLE_SPEED 대신 상수 1.3 사용
            // std::cout << "No critical object detected: Speed set to " << target_vehicle_speed_mps_ << " m/s" << std::endl;
        }
    }

    void startCallback(const std_msgs::Bool::ConstPtr& msg){
        if(msg->data) START_SIGNAL=true;
        else START_SIGNAL=false;
    }
    
    /**
     * @brief 제어 로직을 주기적으로 실행하는 타이머 콜백 함수입니다.
     */
    void controlLoop(const ros::TimerEvent&) {
        // 경로 또는 현재 위치 정보가 없거나, 이전 UTM이 초기화되지 않았으면 정지 명령을 내리고 종료
        if (current_local_path_.empty() || std::isnan(curr_x_) || std::isnan(curr_y_) || previous_utm_.header.stamp.isZero() || !START_SIGNAL) {
            ROS_WARN_THROTTLE(1.0, "empty local path or cannot find utm (UTM init needed).");
            publishCommands(0.0, 0.0); // Stop vehicle if no path or invalid pose
            return;
        }

        // 1. 동적 Look-ahead 거리 계산
        dynamic_look_ahead_distance_ = std::max(MIN_LOOKAHEAD, std::min(MAX_LOOKAHEAD, current_speed_mps_ * LOOKAHEAD_K));
        // std::cout<<"Current Speed (m/s): "<<current_speed_mps_<<", Dynamic Lookahead Dist: "<<dynamic_look_ahead_distance_<<std::endl;

        // 2. 차량과 가장 가까운 로컬 경로 웨이포인트 찾기
        int closest_index = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_local_path_.size(); ++i) {
            double dx = curr_x_ - current_local_path_[i].x;
            double dy = curr_y_ - current_local_path_[i].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }
        
        // 로컬 경로의 끝에 도달하여 목표 지점을 찾지 못한 경우, 차량 정지
        if (closest_index == -1 || current_local_path_.size() == 1) { // 1개만 남았을 때는 더 이상 추적할 점이 없다고 판단
            ROS_INFO("로컬 경로의 끝에 도달했습니다. 차량을 정지합니다.");
            publishCommands(0.0, 0.0);
            return;
        }

        // 3. 가장 가까운 지점부터 시작하여 Dynamic Look-ahead-distance를 만족하는 목표 지점 탐색
        target_index_ = -1;
        // closest_index부터 시작하여 look_ahead_distance_보다 멀리 있는 첫 번째 점을 찾음
        for (size_t i = closest_index; i < current_local_path_.size(); ++i) {
            double dx_target = curr_x_ - current_local_path_[i].x;
            double dy_target = curr_y_ - current_local_path_[i].y;
            double dist_to_vehicle = std::sqrt(dx_target * dx_target + dy_target * dy_target);

            if (dist_to_vehicle >= dynamic_look_ahead_distance_) {
                target_index_ = i;
                break;
            }
        }
        
        // 경로 끝까지 탐색했는데도 적절한 목표 지점을 찾지 못하면, 경로의 마지막 점을 목표로 설정
        if (target_index_ == -1) {
             target_index_ = current_local_path_.size() - 1;
             ROS_WARN_THROTTLE(1.0, "avaliable Look-ahead not found. chase last point.");
             if (current_local_path_.size() == 0) { // 혹시나 경로가 비어있으면 다시 정지
                 publishCommands(0.0, 0.0);
                 return;
             }
        }

        // 4. 목표 지점을 차량의 로컬 좌표계(base_link)로 변환
        waypoint target_waypoint = current_local_path_[target_index_];
        double dx_map = target_waypoint.x - curr_x_;
        double dy_map = target_waypoint.y - curr_y_;

        // 현재 차량의 yaw를 사용하여 목표점을 차량 로컬 좌표계로 변환
        double target_x_local = dx_map * std::cos(yaw_) + dy_map * std::sin(yaw_);
        double target_y_local = -dx_map * std::sin(yaw_) + dy_map * std::cos(yaw_);

        // 5. Pure Pursuit 조향각 계산
        double alpha = std::atan2(target_y_local, target_x_local); // 목표점과 현재 위치 사이의 각도
        double Ld_sq = target_x_local * target_x_local + target_y_local * target_y_local; // 실제 Look-ahead 거리의 제곱

        double steering_angle = std::atan2(2.0 * CAR_LENGTH * target_y_local, Ld_sq);
        
        // 6. 명령 발행
        publishCommands(target_vehicle_speed_mps_, steering_angle);
        
        // ROS_INFO("Target Index (Local Path): %d, Steering Angle (rad): %.2f", target_index_, steering_angle);
    }
    
    /**
     * @brief 계산된 속도와 조향각으로 Ackermann 및 ERP42 제어 메시지를 발행합니다.
     */
    void publishCommands(double target_speed_mps, double steer_angle) {

        // PID 속도 제어
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

        // AckermannDriveStamped 메시지
        ackermann_msgs::AckermannDriveStamped ack_msg;
        ack_msg.header.stamp = ros::Time::now();
        ack_msg.header.frame_id = "base_link";
        ack_msg.drive.steering_angle = steer_angle;
        ack_msg.drive.speed = output_mps_; // PID 제어된 속도 적용
        ack_pub_.publish(ack_msg);

        // ERP42 Cmd 메시지
        erp_driver::erpCmdMsg cmd_msg;
        cmd_msg.e_stop = false; // 기본적으로 E-Stop 해제

        // 기어 설정: 목표 속도가 0이고 현재 속도가 거의 없으면 후진(1), 아니면 전진(0)
        // 실제 ERP42에서는 0: 전진, 1: 중립, 2: 후진 일 수 있으니 확인 필요. 여기서는 1이 후진으로 가정
        // 일반적으로 0은 전진, 1은 중립, 2는 후진입니다. `PathFollower` 코드에서는 1을 정지 상태 기어로 사용했으므로 통일합니다.
        // 현재는 0 (전진) 고정. 0에 가까우면 정지.
        cmd_msg.gear = 0; // 전진

        // 속도 설정: 비상 브레이크 활성화되거나, 현재 속도가 목표 속도를 초과하면 0으로 설정
        if (emergency_brake_ >= 80 || current_speed_mps_ > target_speed_mps + 0.01) { // 0.01 m/s 여유
            cmd_msg.speed = 0; // 즉시 정지
            // ROS_INFO_THROTTLE(0.5, "Emergency brake active (%d) or over speed (%.2f > %.2f). Speed set to 0.", emergency_brake_, current_speed_mps_, target_speed_mps);
        } else {
            // PID 제어된 속도를 ERP42 형식(km/h * 10)으로 변환
            // PathFollower 방식: speed * 3600 / 1000 * 10 => speed * 3.6 * 10
            cmd_msg.speed = static_cast<uint8_t>(output_mps_ * 3.6 * 10.0);
            cmd_msg.speed = std::min(static_cast<uint8_t>(MAX_VEHICLE_SPEED*36), cmd_msg.speed); // 최대 속도 10km/h (200) 제한
            // ROS_INFO_THROTTLE(0.5, "Normal operation. Target (%.2f), Current (%.2f), Output (%.2f), ERP Speed (%d)", target_speed_mps, current_speed_mps_, output_mps_, cmd_msg.speed);
        }
        
        // 브레이크 설정: Yolo 감지에 따른 정지(0m/s) 또는 IMU 비상 브레이크 값 적용
        // target_speed_mps가 0이고 현재 속도가 매우 낮으면 강한 브레이크, 아니면 IMU 값 적용
        if (target_speed_mps < 0.1 && current_speed_mps_ < 0.2) { // 거의 멈춰야 할 때
            cmd_msg.brake = 200; // 강하게 브레이크
            // ROS_INFO_THROTTLE(0.5, "Target speed near zero. Applying strong brake.");
        } else {
            cmd_msg.brake = emergency_brake_; // IMU에서 오는 비상 브레이크 값 적용
        }

        // 조향각 설정: 라디안 -> 도 -> ERP42 형식 (음수 부호 주의)
        double steering_angle_degree = steer_angle * 180.0 / M_PI;
        cmd_msg.steer = -static_cast<int32_t>(steering_angle_degree * 71.0); // 71은 ERP42의 스티어링 비례 상수

        // 조향각 제한 (-2000 ~ 2000)
        cmd_msg.steer = std::max(-2000, std::min(2000, (int)cmd_msg.steer));
        
        erp_cmd_pub_.publish(cmd_msg);

        // 디버그 출력
        // std::cout << "--- Command Published ---" << std::endl;
        // std::cout << "Steering Angle (rad): " << steer_angle << ", ERP Steer Command: " << cmd_msg.steer << std::endl;
        // std::cout << "Target Speed (m/s): " << target_speed_mps << ", Current Speed (m/s): " << current_speed_mps_ << ", ERP Speed Command: " << static_cast<double>(cmd_msg.speed)/10.0 << " km/h" << std::endl;
        // std::cout << "Brake: " << (int)cmd_msg.brake << ", E-Stop: " << (int)cmd_msg.e_stop << ", Gear: " << (int)cmd_msg.gear << std::endl;
        // std::cout << "Yaw: " << yaw_ * 180.0 / M_PI << " deg" << std::endl;
        // std::cout << "-------------------------" << std::endl;
    }

    /**
     * @brief 클래스 초기화 함수
     */
    void initialize() {
        target_index_ = 0;
        dynamic_look_ahead_distance_ = MIN_LOOKAHEAD; // 초기 look-ahead 거리

        curr_x_ = 0.0;
        curr_y_ = 0.0;
        yaw_ = 0.0;
        previous_utm_.header.stamp.fromSec(0); // 유효하지 않은 시간으로 초기화

        erp_status_dt_ = 0.025; // 40Hz ERP status update rate 가정
        error_integral_ = 0;
        prev_error_mps_ = 0;
        emergency_brake_ = 1; // 기본 브레이크 값
        target_vehicle_speed_mps_ = 1.3; // 초기 목표 속도
    }

public:
    PurePursuitPID() : nh_("~") {
        local_path_sub_ = nh_.subscribe("/local_path_dwa", 1, &PurePursuitPID::localPathCallback, this); // Subscribe to local path
        utm_sub_ = nh_.subscribe("/utm_fix", 1, &PurePursuitPID::utmCallback, this); // /utm_fix 구독
        erp_status_sub_ = nh_.subscribe("/erp42_status", 1, &PurePursuitPID::erpStatusCallback, this);
        imu_sub_ = nh_.subscribe("/imu_fix", 1, &PurePursuitPID::imuCallback, this); // /imu_fix 구독
        yolo_sub_ = nh_.subscribe("/yolo_detections", 1, &PurePursuitPID::yoloCallback, this); // Yolo detection 구독
        start_sub_ = nh_.subscribe("/erp42_start",1,&PurePursuitPID::startCallback, this);

        erp_cmd_pub_ = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd", 1);
        ack_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
        
        control_timer_ = nh_.createTimer(ros::Duration(0.05), &PurePursuitPID::controlLoop, this); // 20Hz (0.05초) 제어 루프

        initialize();
    }
    ~PurePursuitPID() = default;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "purepursuit_pid");
    PurePursuitPID pp_pid;
    ros::spin();
    return 0;
}