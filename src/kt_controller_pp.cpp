#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>               // obj_distance 메시지를 받을 때 사용
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <erp_driver/erpCmdMsg.h>
#include <erp_driver/erpStatusMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <yolo/YoloDetectionArray.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>
#include <iostream>

// 단순히 (x,y) 좌표를 저장하는 구조체
struct waypoint {
    double x;
    double y;
};

class PurePursuitController {
private:
    // ROS 통신을 위한 노드 핸들 및 퍼블리셔/서브스크라이버
    ros::NodeHandle nh_;
    ros::Subscriber utm_sub_;            // UTM 위치 업데이트 구독
    ros::Subscriber erp_status_sub_;     // ERP42 상태 메시지 구독
    ros::Subscriber yolo_sub_;           // YOLO 교통신호 감지 구독
    ros::Subscriber imu_sub_;            // IMU 데이터 구독
    ros::Subscriber global_path_sub_;    // 전역 경로(Path) 구독
    ros::Subscriber start_signal_sub_;   // 주행 시작 신호 구독
    ros::Subscriber obj_dist_sub_;       // Python에서 퍼블리시된 객체 거리 구독

    ros::Publisher erp_cmd_pub_;         // ERP42 제어 명령 퍼블리시
    ros::Publisher ack_pub_;             // Ackermann 메시지 퍼블리시 (시각화용)
    ros::Timer control_timer_;           // 제어 루프 타이머 (주기적 호출)

    // 전역 경로를 저장하는 벡터
    std::vector<waypoint> waypoint_vector_;
    int target_index_;                   // 현재 목표 웨이포인트 인덱스

    // 차량의 현재 위치/방향 상태
    double curr_x_, curr_y_, yaw_;
    double prev_x_, prev_y_;             // Yaw 계산을 위한 이전 위치

    // Pure Pursuit 제어 파라미터
    double car_length_;                  // 차량 축간 거리
    double look_ahead_distance_;         // look-ahead 거리

    // ERP 상태 정보
    double current_speed_erp_;           // ERP 단위 속도 값
    double current_speed_mps_;           // m/s 단위 속도
    double current_speed_kph_;           // km/h 단위 속도
    double current_steer_erp_;           // ERP 단위 조향 값
    double current_steer_rad_;           // 라디안 단위 조향 각
    double current_steer_deg_;           // 도 단위 조향 각
    double erp_status_dt_;               // ERP 상태 메시지 수신 간격
    double prev_error_mps_, error_integral_;  // PID 제어용 누적 오차
    double yolo_speed_mps_, MAX_SPEED_MPS_;   // YOLO 신호에 따른 목표 속도

    uint8_t emergency_brake_;            // 비상 제동 강도
    bool START_SIGNAL_;                  // 주행 시작 플래그

    // 회피 로직용 변수
    double obj_distance_;                // 감지된 객체까지 거리 (m)
    bool avoid_mode_;                    // 회피 모드 활성화 여부
    int  avoid_stage_;                   // 회피 단계 (1: WP1, 2: WP2)
    waypoint avoid_wp1_, avoid_wp2_;     // 회피 웨이포인트 좌표
    const double REACH_TOL_ = 0.5;       // 웨이포인트 도달 허용 오차 (m)

    // ------------------------------------------------------------------------
    // 콜백: 새로운 전역 경로가 수신되면 waypoint_vector_를 갱신
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoint_vector_.clear();
        for (const auto& pose_stamped : msg->poses) {
            waypoint wp;
            wp.x = pose_stamped.pose.position.x;
            wp.y = pose_stamped.pose.position.y;
            waypoint_vector_.push_back(wp);
        }
        ROS_INFO_ONCE("Global path received with %zu waypoints.", waypoint_vector_.size());
    }

    // 콜백: UTM 위치를 받아와 curr_x_, curr_y_를 갱신하고, 이동 방향으로 yaw 계산
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_x_ = msg->pose.position.x;
        curr_y_ = msg->pose.position.y;
        double dx = curr_x_ - prev_x_;
        double dy = curr_y_ - prev_y_;
        if (std::sqrt(dx*dx + dy*dy) > 0.01) {
            yaw_ = std::atan2(dy, dx);
        }
        prev_x_ = curr_x_;
        prev_y_ = curr_y_;
    }

    // 콜백: ERP42 상태 메시지로부터 속도·조향 상태 추출
    void erpStatusCallback(const erp_driver::erpStatusMsg::ConstPtr& msg) {
        current_speed_erp_ = msg->speed;
        current_steer_erp_ = msg->steer;
        current_speed_kph_ = current_speed_erp_ / 10.0;
        current_speed_mps_ = current_speed_kph_ / 3.6;
        current_steer_deg_ = static_cast<double>(current_steer_erp_) / 71.0;
        current_steer_rad_ = current_steer_deg_ * M_PI / 180.0;
    }

    // 콜백: IMU 데이터를 받아 필요 시 비상 제동 로직에 활용
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w
        );
        double roll, pitch, yaw_imu;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_imu);
        // slope detection 등에서 emergency_brake_를 설정할 수 있음
    }

    // 콜백: YOLO 교통신호 감지 결과에 따라 속도 조절
    void yoloCallback(const yolo::YoloDetectionArray::ConstPtr& msg) {
        bool redlight = false, yellowlight = false;
        for (auto& det : msg->detections) {
            if (det.class_name == "redlight") { redlight = true; break; }
            if (det.class_name == "yellowlight") { yellowlight = true; break; }
        }
        yolo_speed_mps_ = (redlight || yellowlight) ? 0.0 : MAX_SPEED_MPS_;
    }

    // 콜백: 주행 시작 신호 수신
    void startSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
        START_SIGNAL_ = msg->data;
    }

    // 콜백: Python에서 퍼블리시한 객체 거리 수신
    void objDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
        obj_distance_ = msg->data;
        // 0보다 큰 값만 Python에서 30% 이상일 때 퍼블리시됨
        // 또는 obj_distance자체의 값을 조건문의 값으로 사용하면됨
        // 해당 정보는 kt_camera_distance.py에서 토픽 발행 조건에 해당함.
    }

    // 제어 루프: 50Hz로 동작하며 회피 모드/기본 모드 제어
    void controlLoop(const ros::TimerEvent&) {
        // 시작 신호 없으면 즉시 정지
        if (!START_SIGNAL_) {
            publishCommands(0.0, 0.0);
            return;
        }

        // ---------------- 회피 모드 진입 ----------------
        if (!avoid_mode_ && obj_distance_ > 0.0) {
            ROS_INFO("Entering avoidance: obj_dist=%.2f", obj_distance_);
            avoid_wp1_.x = curr_x_ + obj_distance_;
            avoid_wp1_.y = curr_y_ + 1.0;
            avoid_wp2_.x = curr_x_ + obj_distance_ + 1.0;
            avoid_wp2_.y = curr_y_;
            avoid_mode_  = true;
            avoid_stage_ = 1;
        }

        // ---------------- 회피 모드 실행 ----------------
        if (avoid_mode_) {
            // 현재 단계에 맞는 목표 웨이포인트 선택
            waypoint target = (avoid_stage_ == 1 ? avoid_wp1_ : avoid_wp2_);

            // 로컬 좌표계로 변환
            double dx = target.x - curr_x_;
            double dy = target.y - curr_y_;
            double x_local =  dx * std::cos(yaw_) + dy * std::sin(yaw_);
            double y_local = -dx * std::sin(yaw_) + dy * std::cos(yaw_);

            // Pure Pursuit 스티어링 계산
            double ld_sq = x_local*x_local + y_local*y_local;
            double steer = std::atan2(2.0 * car_length_ * y_local, ld_sq);

            // 회피 속도 및 조향 명령 발행
            publishCommands(yolo_speed_mps_, steer);

            // 목표 도달 판단
            double dist_to_wp = std::hypot(dx, dy);
            if (dist_to_wp < REACH_TOL_) {
                if (avoid_stage_ == 1) {
                    ROS_INFO("Reached avoid WP1");
                    avoid_stage_ = 2;  // 다음 단계로
                } else {
                    ROS_INFO("Reached avoid WP2, returning to path");
                    avoid_mode_   = false;
                    obj_distance_ = 0.0;  // 리셋
                }
            }
            return;  // 회피 처리 후 기본 로직 스킵
        }

        // ---------------- 기본 순수추종 로직 ----------------
        if (waypoint_vector_.empty() || std::isnan(curr_x_) || std::isnan(curr_y_)) {
            ROS_WARN_THROTTLE(1.0, "Path empty or position unknown.");
            publishCommands(0.0, 0.0);
            return;
        }

        // 1) 가장 가까운 웨이포인트 탐색
        int closest_index = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < waypoint_vector_.size(); ++i) {
            double dx = curr_x_ - waypoint_vector_[i].x;
            double dy = curr_y_ - waypoint_vector_[i].y;
            double d  = std::hypot(dx, dy);
            if (d < min_dist) {
                min_dist  = d;
                closest_index = i;
            }
        }

        // 2) Look-ahead 거리 동적 조정 예시 (커브 구간에서 짧게)
        if (closest_index >= 920 && closest_index <= 1060) {
            look_ahead_distance_ = 1.05;
        } else {
            look_ahead_distance_ = 3.0;
        }

        // 3) 목표 웨이포인트 인덱스 결정
        target_index_ = -1;
        for (size_t i = closest_index; i < waypoint_vector_.size(); ++i) {
            double dx = curr_x_ - waypoint_vector_[i].x;
            double dy = curr_y_ - waypoint_vector_[i].y;
            if (std::hypot(dx, dy) >= look_ahead_distance_) {
                target_index_ = i;
                break;
            }
        }
        if (target_index_ == -1) {
            ROS_INFO("Reached end of path. Stopping.");
            publishCommands(0.0, 0.0);
            return;
        }

        // 4) 로컬 좌표 변환
        waypoint tgt = waypoint_vector_[target_index_];
        double dx_t = tgt.x - curr_x_;
        double dy_t = tgt.y - curr_y_;
        double x_loc =  dx_t * std::cos(yaw_) + dy_t * std::sin(yaw_);
        double y_loc = -dx_t * std::sin(yaw_) + dy_t * std::cos(yaw_);

        // 5) Pure Pursuit 스티어링 각도 계산
        double ld2 = x_loc*x_loc + y_loc*y_loc;
        double steering_angle = std::atan2(2.0 * car_length_ * y_loc, ld2);

        // 6) 속도·조향 명령 발행
        publishCommands(yolo_speed_mps_, steering_angle);
        ROS_INFO_THROTTLE(1.0, "TargetIdx: %d, Steer: %.2f rad", target_index_, steering_angle);
    }

    // 속도(PID) 및 조향 명령 생성/발행 함수
    void publishCommands(double target_speed_mps, double steer_angle) {
        // ---- longitudinal PID 제어 ----
        double p_gain = 0.7, i_gain = 0.0, d_gain = 0.005;
        double error_speed = target_speed_mps - current_speed_mps_;
        double output_mps = 0.0;
        if (error_speed <= 0) {
            output_mps       = 0.0;
            emergency_brake_ = static_cast<uint8_t>(-error_speed * 660);
        } else {
            double p_term = p_gain * error_speed;
            error_integral_ += error_speed * erp_status_dt_;
            double i_term = i_gain * error_integral_;
            double d_term = d_gain * (error_speed - prev_error_mps_) / erp_status_dt_;
            prev_error_mps_ = error_speed;
            output_mps = p_term + i_term + d_term;
        }

        // ---- Ackermann 메시지 퍼블리시 (시각화용) ----
        ackermann_msgs::AckermannDriveStamped ack_msg;
        ack_msg.header.stamp = ros::Time::now();
        ack_msg.header.frame_id = "base_link";
        ack_msg.drive.steering_angle = steer_angle;
        ack_msg.drive.speed = output_mps;
        ack_pub_.publish(ack_msg);

        // ---- ERP42 제어 메시지 구성 및 퍼블리시 ----
        erp_driver::erpCmdMsg cmd_msg;
        cmd_msg.e_stop = false;
        cmd_msg.gear   = 0;
        // 속도: 출력값이 목표보다 크면 0으로 설정, 아니면 환산하여 설정
        cmd_msg.speed  = (current_speed_mps_ > target_speed_mps)
                           ? 0
                           : static_cast<uint8_t>(output_mps * 3.6 * 10.0);
        cmd_msg.brake  = emergency_brake_;
        double steer_deg = steer_angle * 180.0 / M_PI;
        // ERP42 스티어 명령은 부호 반전 및 클램핑
        cmd_msg.steer  = -std::max(-2000, std::min(2000, (int)(steer_deg * 71.0)));
        erp_cmd_pub_.publish(cmd_msg);

        std::cout << "Steer(rad): " << steer_angle
                  << ", ERP steer: " << cmd_msg.steer
                  << ", Speed unit: " << (int)cmd_msg.speed << std::endl;
    }

    // 멤버 변수 초기화 함수
    void initialize() {
        target_index_    = 0;
        car_length_      = 1.04;
        look_ahead_distance_ = 3.0;

        curr_x_ = curr_y_ = prev_x_ = prev_y_ = yaw_ = 0.0;
        erp_status_dt_   = 0.025;
        error_integral_  = prev_error_mps_ = 0.0;
        emergency_brake_ = 1;
        MAX_SPEED_MPS_   = 2.0;
        yolo_speed_mps_  = MAX_SPEED_MPS_;
        START_SIGNAL_    = false;

        obj_distance_    = 0.0;
        avoid_mode_      = false;
        avoid_stage_     = 0;
    }

public:
    PurePursuitController() : nh_("~") {
        // 기존 구독자 초기화
        utm_sub_           = nh_.subscribe("/utm_fix",        1, &PurePursuitController::utmCallback, this);
        erp_status_sub_    = nh_.subscribe("/erp42_status",   1, &PurePursuitController::erpStatusCallback, this);
        yolo_sub_          = nh_.subscribe("/yolo_detections",1, &PurePursuitController::yoloCallback, this);
        imu_sub_           = nh_.subscribe("/imu_fix",        1, &PurePursuitController::imuCallback, this);
<<<<<<< HEAD
        global_path_sub_   = nh_.subscribe("/local_path",     1, &PurePursuitController::globalPathCallback, this);
=======
        global_path_sub_   = nh_.subscribe("/avoid_path",     1, &PurePursuitController::globalPathCallback, this);
>>>>>>> 1e8b62ec3411af03410a4b86c48f421915037f0b
        start_signal_sub_  = nh_.subscribe("/erp42_start",    1, &PurePursuitController::startSignalCallback, this);
        obj_dist_sub_      = nh_.subscribe("/obj_distance",   1, &PurePursuitController::objDistanceCallback, this);

        // 퍼블리셔 초기화
        erp_cmd_pub_       = nh_.advertise<erp_driver::erpCmdMsg>("/erp42_ctrl_cmd", 1);
        ack_pub_           = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);

        // 제어 루프 타이머 설정 (50Hz)
        control_timer_     = nh_.createTimer(
                                ros::Duration(1.0/50.0),
                                &PurePursuitController::controlLoop, this
                             );

        initialize();  // 모든 멤버 변수 초기값 설정
    }

    ~PurePursuitController() = default;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    PurePursuitController ppc;
    ros::spin();
    return 0;
}
