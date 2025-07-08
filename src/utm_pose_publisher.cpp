#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h> // /utm이 PointStamped일 가능성도 고려
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h> // tf2::Transform 사용
#include <cmath> // atan2 사용

class UTMPosePublisher {
public:
    UTMPosePublisher() : nh_("~") {
        // 파라미터 서버에서 GPS 센서의 base_link 기준 오프셋을 불러옵니다.
        // 기본값: x=0.0, y=-0.10, z=0.0
        nh_.param<double>("gps_offset_x", gps_offset_x_, 0.0);
        nh_.param<double>("gps_offset_y", gps_offset_y_, 0.16);
        nh_.param<double>("gps_offset_z", gps_offset_z_, 0.0);

        // /utm 토픽 구독
        utm_sub_ = nh_.subscribe("/utm", 10, &UTMPosePublisher::utmCallback, this);

        // /utm_fix 토픽 발행 (PoseStamped 형태)
        utm_fix_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/utm_fix", 10);

        prev_position_initialized_ = false; // 이전 위치 초기화 여부 플래그

        ROS_INFO("UTM Pose Publisher Node initialized. Publishing to /utm_fix with yaw estimation and offset compensation.");
        ROS_INFO("GPS Sensor Offset (base_link frame): x=%f, y=%f, z=%f", gps_offset_x_, gps_offset_y_, gps_offset_z_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber utm_sub_;
    ros::Publisher utm_fix_pub_;

    double gps_offset_x_;
    double gps_offset_y_;
    double gps_offset_z_;

    geometry_msgs::Point prev_position_;
    bool prev_position_initialized_;

    // /utm 토픽이 geometry_msgs/PoseStamped 타입이라고 가정하고 position 필드를 사용합니다.
    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& utm_msg) {
        geometry_msgs::PoseStamped utm_fix_pose;

        // 1. 헤더 복사 (frame_id와 timestamp)
        utm_fix_pose.header = utm_msg->header;

        // 2. Yaw (방향) 추정
        double estimated_yaw = 0.0; // 기본값은 0 (단위 쿼터니언)

        if (prev_position_initialized_) {
            double dx = utm_msg->pose.position.x - prev_position_.x;
            double dy = utm_msg->pose.position.y - prev_position_.y;

            // 이동 거리가 너무 짧으면 Yaw 추정하지 않음 (노이즈에 민감)
            if (std::hypot(dx, dy) > 0.05) { // 5cm 이상 이동했을 때만 추정
                estimated_yaw = atan2(dy, dx);
            } else {
                // 이전 Yaw를 유지하고 싶다면, 여기에 prev_yaw_ 멤버 변수를 사용
                // 여기서는 이동이 없거나 미미하면 yaw 변화가 없다고 가정하고 이전 estimated_yaw 값을 유지하지 않음
                // (이전 콜백에서 계산된 estimated_yaw는 이 범위 밖에서는 0이 아님)
                // 필요하다면 이곳에 prev_estimated_yaw_ 멤버 변수를 추가하여 사용할 수 있습니다.
            }
        } else {
            ROS_INFO_ONCE("First UTM message received. Cannot estimate yaw yet.");
        }

        // 3. 현재 위치를 이전 위치로 저장
        prev_position_ = utm_msg->pose.position;
        prev_position_initialized_ = true;

        // 4. UTM 좌표에 base_link 기준 오프셋 적용 (회전된 오프셋)
        // GPS 센서의 위치가 utm_msg->pose.position 입니다.
        // base_link는 이 센서로부터 (0, -0.10, 0) 만큼 떨어져 있습니다 (base_link 기준).
        // 따라서 base_link의 UTM 위치는 센서의 UTM 위치에서 이 오프셋의 글로벌 좌표계 변환 값을 "빼야" 합니다.

        // base_link의 추정된 방향 쿼터니언 생성 (roll=0, pitch=0, yaw=estimated_yaw)
        tf2::Quaternion base_link_orientation_quat;
        base_link_orientation_quat.setRPY(0, 0, estimated_yaw);

        // base_link 프레임에서의 오프셋 벡터
        tf2::Vector3 offset_bl(gps_offset_x_, gps_offset_y_, gps_offset_z_);

        // tf2::Transform을 생성합니다. 여기서는 회전만 필요하므로, translation은 (0,0,0)입니다.
        tf2::Transform base_link_to_utm_transform(base_link_orientation_quat, tf2::Vector3(0,0,0));

        // 이 오프셋 벡터를 UTM 프레임으로 회전 변환합니다.
        tf2::Vector3 offset_utm_frame = base_link_to_utm_transform * offset_bl;
        // tf2::Transform::operator*(const tf2::Vector3&)는 올바른 연산자 오버로딩이며 tf2::Vector3를 반환합니다.

        // 5. base_link의 UTM 위치 계산
        // 센서 위치에서 회전된 오프셋을 뺍니다.
        utm_fix_pose.pose.position.x = utm_msg->pose.position.x - offset_utm_frame.x();
        utm_fix_pose.pose.position.y = utm_msg->pose.position.y - offset_utm_frame.y();
        utm_fix_pose.pose.position.z = utm_msg->pose.position.z - offset_utm_frame.z();

        // 6. 추정된 Yaw를 orientation에 설정
        // tf2::Quaternion을 geometry_msgs::Quaternion으로 변환
        utm_fix_pose.pose.orientation.x = base_link_orientation_quat.x();
        utm_fix_pose.pose.orientation.y = base_link_orientation_quat.y();
        utm_fix_pose.pose.orientation.z = base_link_orientation_quat.z();
        utm_fix_pose.pose.orientation.w = base_link_orientation_quat.w();

        // 7. /utm_fix 토픽 발행
        utm_fix_pub_.publish(utm_fix_pose);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "utm_pose_publisher_node");
    UTMPosePublisher publisher;
    ros::spin();
    return 0;
}