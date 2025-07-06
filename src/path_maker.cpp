#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> // UTM 좌표 수신을 위해 PoseStamped 사용 (Pose도 가능)
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h> // tf::Quaternion, tf::Matrix3x3, tf::getYaw 등을 사용하기 위함

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath> // M_PI 사용

// 경로 생성을 위한 클래스
class PathMaker {
public:
    // 생성자: 노드 초기화, 구독자 및 파일 설정
    PathMaker() : is_first_imu_(true), has_utm_data_(false) {
        // ROS 노드 핸들 초기화
        nh_ = ros::NodeHandle("~"); // private 노드 핸들 사용

        // 파라미터 로드 (토픽 이름 및 파일 경로)
        std::string utm_topic, imu_topic, file_path;
        nh_.param<std::string>("utm_topic", utm_topic, "/utm_pose");
        nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
        nh_.param<std::string>("file_path", file_path, "ctrack-path.csv");

        ROS_INFO("UTM Topic: %s", utm_topic.c_str());
        ROS_INFO("IMU Topic: %s", imu_topic.c_str());
        ROS_INFO("Output File Path: %s", file_path.c_str());
        
        // 구독자(Subscriber) 설정
        // UTM 좌표 (x, y) 데이터를 수신합니다. geometry_msgs/PoseStamped 타입을 사용합니다.
        sub_utm_ = nh_.subscribe(utm_topic, 10, &PathMaker::utmCallback, this);
        // IMU 데이터 (자세)를 수신합니다. sensor_msgs/Imu 타입을 사용합니다.
        sub_imu_ = nh_.subscribe(imu_topic, 10, &PathMaker::imuCallback, this);

        // CSV 파일 열기
        // std::ios::out: 쓰기 모드로 파일 열기
        csv_file_.open(file_path, std::ios::out);
        if (!csv_file_.is_open()) {
            ROS_ERROR("Failed to open file: %s", file_path.c_str());
            ros::shutdown();
            return;
        }

        // CSV 파일 헤더 작성
        // 형식: x(m), y(m), velocity(m/s), yaw(rad), yawrate(rad/s)
        csv_file_ << "x,y,velocity,yaw,yawrate\n";
        ROS_INFO("Path Maker node initialized. Writing to %s", file_path.c_str());
    }

    // 소멸자: 파일 닫기
    ~PathMaker() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            ROS_INFO("CSV file closed.");
        }
    }

private:
    // UTM 토픽 콜백 함수
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 수신된 메시지에서 x, y 좌표 저장
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
        has_utm_data_ = true; // UTM 데이터를 수신했음을 표시
    }

    // IMU 토픽 콜백 함수
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // UTM 데이터가 아직 수신되지 않았다면, 아무 작업도 하지 않고 반환
        if (!has_utm_data_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for UTM data...");
            return;
        }

        // 1. Quaternion에서 Yaw 각도 추출
        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // Roll, Pitch, Yaw 추출 (단위: 라디안)

        // 2. Yaw Rate 계산
        double yaw_rate = 0.0;
        ros::Time current_time = msg->header.stamp;

        if (is_first_imu_) {
            // 첫 번째 IMU 데이터인 경우, yaw rate 계산이 불가능하므로 현재 값만 저장
            is_first_imu_ = false;
        } else {
            // 시간 변화량 계산
            double dt = (current_time - last_time_).toSec();
            if (dt > 1e-6) { // 매우 작은 시간 변화는 무시 (0으로 나누기 방지)
                // Yaw 각도 변화량 계산 (Angle Wrap-around 처리)
                // 예: -pi에서 pi로 넘어갈 때의 큰 변화를 작은 변화로 보정
                double delta_yaw = yaw - last_yaw_;
                if (delta_yaw > M_PI) {
                    delta_yaw -= 2 * M_PI;
                } else if (delta_yaw < -M_PI) {
                    delta_yaw += 2 * M_PI;
                }
                yaw_rate = delta_yaw / dt;
            }
        }

        // 3. 파일에 데이터 쓰기
        // 고정 속도 (10 km/h -> m/s)
        const double velocity_mps = 10.0 * 1000.0 / 3600.0;

        csv_file_ << std::fixed << std::setprecision(6) // 소수점 6자리까지 표시
                  << current_x_ << ","
                  << current_y_ << ","
                  << velocity_mps << ","
                  << yaw << ","
                  << yaw_rate << "\n";

        // 다음 계산을 위해 현재 yaw와 시간 저장
        last_yaw_ = yaw;
        last_time_ = current_time;
    }

    // ROS 관련 멤버 변수
    ros::NodeHandle nh_;
    ros::Subscriber sub_utm_;
    ros::Subscriber sub_imu_;

    // 파일 스트림
    std::ofstream csv_file_;

    // 데이터 저장을 위한 멤버 변수
    double current_x_;
    double current_y_;
    bool has_utm_data_; // UTM 데이터 수신 여부 플래그

    // Yaw Rate 계산을 위한 멤버 변수
    double last_yaw_;
    ros::Time last_time_;
    bool is_first_imu_; // 첫 번째 IMU 데이터 여부 플래그
};

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "path_maker_node");

    // PathMaker 객체 생성
    PathMaker path_maker;

    // ROS 이벤트 루프 실행 (콜백 함수들이 호출되도록 대기)
    ros::spin();

    return 0;
}
