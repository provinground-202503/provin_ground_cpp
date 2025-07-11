#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h> // tf/transform_datatypes.h 대신 사용 가능

#include <cmath>    // 수학 함수 사용
#include <cstdlib>  // system("clear") 함수 사용

// 클래스 정의
class PurePursuit {
public:
    // 생성자: 노드 초기화, 퍼블리셔/서브스크라이버 설정
    PurePursuit() {
        // ROS 노드 핸들 초기화
        nh_ = ros::NodeHandle();

        // 서브스크라이버 설정
        path_sub_ = nh_.subscribe("/local_path", 1, &PurePursuit::pathCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &PurePursuit::odomCallback, this);

        // 퍼블리셔 설정
        ctrl_cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

        // CtrlCmd 메시지 초기 설정
        ctrl_cmd_msg_.longlCmdType = 2; // 제어 명령 타입 설정

        // 멤버 변수 초기화
        is_path_received_ = false;
        is_odom_received_ = false;
        vehicle_length_ = 1.0; // 차량 축거 (L)
        lfd_ = 10.0;           // 전방 주시 거리 (Look-Forward Distance)
    }

    // 메인 루프를 실행하는 함수
    void run() {
        ros::Rate rate(15); // 15Hz

        while (ros::ok()) {
            // 경로와 위치(Odom) 정보가 모두 수신되었을 때만 로직 실행
            if (is_path_received_ && is_odom_received_) {
                
                geometry_msgs::Point vehicle_position = current_position_;
                bool is_look_forward_point = false;

                // 차량의 현재 yaw 각도와 위치를 이용해 변환 행렬의 역행렬 계산 준비
                double cos_yaw = cos(vehicle_yaw_);
                double sin_yaw = sin(vehicle_yaw_);

                // 경로 상의 점들을 순회하며 전방 주시점(Forward Point) 탐색
                geometry_msgs::Point forward_point;
                double target_local_x = 0.0;
                double target_local_y = 0.0;

                for (const auto& pose_stamped : local_path_.poses) {
                    geometry_msgs::Point path_point = pose_stamped.pose.position;

                    // 전역 경로점을 차량 로컬 좌표계로 변환
                    // Python의 det_t.dot(global_path_point)와 동일한 연산
                    double local_path_point_x = (path_point.x - vehicle_position.x) * cos_yaw + (path_point.y - vehicle_position.y) * sin_yaw;
                    double local_path_point_y = -(path_point.x - vehicle_position.x) * sin_yaw + (path_point.y - vehicle_position.y) * cos_yaw;

                    // 차량 전방에 있는 점인지 확인
                    if (local_path_point_x > 0) {
                        double dis = std::sqrt(std::pow(local_path_point_x, 2) + std::pow(local_path_point_y, 2));
                        
                        // 계산된 거리가 설정된 전방 주시 거리(lfd)보다 크거나 같으면 해당 점을 목표점으로 설정
                        if (dis >= lfd_) {
                            forward_point = path_point;
                            is_look_forward_point = true;
                            target_local_x = local_path_point_x;
                            target_local_y = local_path_point_y;
                            break; // 목표점을 찾았으므로 루프 종료
                        }
                    }
                }

                // 목표점을 찾았을 경우
                if (is_look_forward_point) {
                    // 목표점까지의 각도(theta) 계산
                    double theta = std::atan2(target_local_y, target_local_x);
                    
                    // Pure Pursuit 조향각 계산 공식
                    double steering_angle = std::atan2(2 * vehicle_length_ * std::sin(theta), lfd_);

                    ctrl_cmd_msg_.steering = steering_angle;
                    ctrl_cmd_msg_.velocity = 15.0; // kph 단위

                    system("clear");
                    std::cout << "-------------------------------------" << std::endl;
                    std::cout << " steering (deg) = " << ctrl_cmd_msg_.steering * 180 / M_PI << std::endl;
                    std::cout << " velocity (kph) = " << ctrl_cmd_msg_.velocity << std::endl;
                    std::cout << "-------------------------------------" << std::endl;

                } else { // 목표점을 찾지 못했을 경우
                    std::cout << "no found forward point" << std::endl;
                    ctrl_cmd_msg_.steering = 0.0;
                    ctrl_cmd_msg_.velocity = 0.0;
                }

                // 제어 명령 퍼블리시
                ctrl_cmd_pub_.publish(ctrl_cmd_msg_);

            } else {
                system("clear");
                if (!is_path_received_) {
                    std::cout << "[1] can't subscribe '/local_path' topic..." << std::endl;
                }
                if (!is_odom_received_) {
                    std::cout << "[2] can't subscribe '/odom' topic..." << std::endl;
                }
            }

            // 다음 루프를 위해 플래그 초기화
            is_path_received_ = false;
            is_odom_received_ = false;

            ros::spinOnce(); // 콜백 함수 처리
            rate.sleep();    // 루프 주기 맞춤
        }
    }

private:
    // Path 메시지 콜백 함수
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        local_path_ = *msg;
        is_path_received_ = true;
    }

    // Odometry 메시지 콜백 함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_ = msg->pose.pose.position;

        // 쿼터니언(quaternion)에서 yaw 각도 추출
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, vehicle_yaw_); // roll, pitch, yaw 추출

        is_odom_received_ = true;
    }

    // ROS 관련 멤버 변수
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher ctrl_cmd_pub_;
    morai_msgs::CtrlCmd ctrl_cmd_msg_;

    // 알고리즘에 필요한 멤버 변수
    nav_msgs::Path local_path_;
    geometry_msgs::Point current_position_;
    double vehicle_yaw_;
    double vehicle_length_;
    double lfd_;

    // 데이터 수신 확인 플래그
    bool is_path_received_;
    bool is_odom_received_;
};

// main 함수
int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "pure_pursuit_cpp");
    
    // PurePursuit 객체 생성
    PurePursuit pure_pursuit_node;
    
    // 노드 실행
    pure_pursuit_node.run();

    return 0;
}