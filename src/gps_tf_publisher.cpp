#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

// 이전 Pose 데이터를 저장하기 위한 전역 변수
geometry_msgs::PoseStamped prev_pose;
bool is_first_pose = true;

// PoseStamped 메시지 콜백 함수
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // TF를 발행하기 위한 TransformBroadcaster 객체
    // static으로 선언하여 콜백이 호출될 때마다 다시 생성되지 않도록 함
    static tf2_ros::TransformBroadcaster broadcaster;

    // 첫 번째 메시지를 받으면, 이전 위치로 저장하고 함수 종료
    // 아직 이동량이 없으므로 Yaw를 계산할 수 없음
    if (is_first_pose) {
        prev_pose = *msg;
        is_first_pose = false;
        ROS_INFO("First UTM pose received. Waiting for the next pose to calculate yaw.");
        return;
    }

    // 현재 위치와 이전 위치의 차이(dx, dy) 계산
    double dx = msg->pose.position.x - prev_pose.pose.position.x;
    double dy = msg->pose.position.y - prev_pose.pose.position.y;

    // atan2를 사용하여 이동 방향(Yaw) 계산
    // atan2(y, x)는 x축 양의 방향으로부터 (x, y) 점까지의 각도를 라디안 단위로 반환
    double yaw = atan2(dy, dx);

    // 계산된 Yaw로부터 Quaternion 생성 (Roll, Pitch는 0으로 가정)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    // 발행할 TransformStamped 메시지 생성
    geometry_msgs::TransformStamped transformStamped;

    // 헤더 정보 설정
    transformStamped.header.stamp = ros::Time::now(); // 현재 시간으로 설정
    transformStamped.header.frame_id = "gps";        // 부모 프레임 (GPS 좌표계)
    transformStamped.child_frame_id = "base_link";   // 자식 프레임 (로봇 베이스)

    // 위치 정보 설정 (수신한 메시지의 position 사용)
    transformStamped.transform.translation.x = msg->pose.position.x;
    transformStamped.transform.translation.y = msg->pose.position.y;
    transformStamped.transform.translation.z = msg->pose.position.z;

    // 방향 정보 설정 (계산된 Quaternion 사용)
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // TF 발행
    broadcaster.sendTransform(transformStamped);

    // 현재 위치를 다음 계산을 위해 이전 위치로 저장
    prev_pose = *msg;
}

int main(int argc, char** argv){
    // ROS 노드 초기화
    ros::init(argc, argv, "gps_to_baselink_tf_publisher");
    ros::NodeHandle nh;

    // /utm 토픽 구독자 설정
    ros::Subscriber sub = nh.subscribe("/utm", 10, &poseCallback);

    // 콜백 함수 대기
    ros::spin();
    return 0;
};