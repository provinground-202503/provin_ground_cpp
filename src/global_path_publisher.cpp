#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
#include <fstream>  // CSV 파일 입출력을 위해 필요
#include <sstream>  // 문자열 파싱을 위해 필요

class GlobalPathPublisher {
public:
    GlobalPathPublisher() {
        // Private NodeHandle을 사용하여 파라미터를 안전하게 받아옵니다.
        ros::NodeHandle nh("~");

        // ROS Publisher 설정: /global_path 토픽에 nav_msgs::Path 메시지를 발행합니다.
        // Queue size는 1, latched는 true로 설정합니다.
        // Latching: 새로운 구독자가 연결되었을 때, 가장 마지막에 발행된 메시지를 즉시 전달해줍니다.
        // 경로 데이터처럼 자주 변하지 않는 데이터에 매우 유용합니다.
        path_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 1, true);

        // 파라미터 서버로부터 CSV 파일 경로를 받아옵니다.
        // 실행 시 "_path_file:=/경로/파일명.csv" 형태로 지정할 수 있습니다.
        std::string file_path;
        // 기본 CSV 파일 경로를 /root/erp42_ws/src/provin_ground_cpp/path/path_morai_xy.csv 로 설정
        nh.param<std::string>("path_file", file_path, "/root/erp42_ws/src/provin_ground_cpp/path/path_morai_xy.csv"); 
        if (file_path.empty()) {
            ROS_FATAL("CSV file path is empty! set the parameter _path_file .");
            ros::shutdown();
            return;
        }
        nh.param<double>("utm_x_offset", UTM_X_OFFSET, 360825.8208815998);
        nh.param<double>("utm_y_offset", UTM_Y_OFFSET, 4065896.29857793);

        // 파라미터로 받은 경로의 CSV 파일을 로드합니다.
        loadPathFromCSV(file_path);

        // 10Hz 주기로 경로를 발행하기 위한 타이머를 설정합니다.
        pub_timer_ = nh.createTimer(ros::Duration(0.1), &GlobalPathPublisher::timerCallback, this);
        
        ROS_INFO("Global Path Publisher started. %zu waypoint will be published.", global_path_.poses.size());
    }

private:
    // CSV 파일에서 경로 데이터를 로드하는 함수
    void loadPathFromCSV(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_FATAL("cannot open waypoint file: %s", filename.c_str());
            ros::shutdown();
            return;
        }

        // Path 메시지의 헤더 설정
        global_path_.header.frame_id = "map"; // 경로는 "map" 좌표계를 기준으로 함
        global_path_.header.stamp = ros::Time::now();

        std::string line;
        // 첫 번째 줄 (헤더)은 건너뜁니다.
        // 만약 CSV 파일에 헤더가 없다면 이 줄을 주석 처리해야 합니다.
        std::getline(file, line); 

        // 파일의 각 줄을 읽어 x, y를 파싱
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> values;

            while (std::getline(ss, segment, ',')) {
                values.push_back(segment);
            }

            // 필요한 컬럼 (x, y)이 모두 있는지 확인 (최소 2개 컬럼 필요)
            if (values.size() >= 2) { // yaw 정보가 없으므로 2개 이상으로 변경
                try {
                    geometry_msgs::PoseStamped pose;
                    // x, y 좌표 파싱 (컬럼 0, 1)
                    pose.pose.position.x = std::stod(values[0])+UTM_X_OFFSET;
                    pose.pose.position.y = std::stod(values[1])+UTM_Y_OFFSET;
                    pose.pose.position.z = 0; // 2D 주행이므로 z는 0으로 고정

                    // Yaw 정보가 CSV 파일에 없으므로, 기본 Quaternion (회전 없음)을 사용
                    // tf2::Quaternion q;
                    // q.setRPY(0, 0, 0); // Roll, Pitch, Yaw 모두 0 (회전 없음)
                    // pose.pose.orientation = tf2::toMsg(q);
                    // 또는 geometry_msgs::Quaternion의 기본값 (0,0,0,1)을 그대로 사용
                    pose.pose.orientation.x = 0;
                    pose.pose.orientation.y = 0;
                    pose.pose.orientation.z = 0;
                    pose.pose.orientation.w = 1;


                    // 생성된 PoseStamped를 경로에 추가
                    global_path_.poses.push_back(pose);

                } catch (const std::exception& e) {
                    ROS_ERROR("error occurred when parsing CSV (line: %s): %s", line.c_str(), e.what());
                }
            } else {
                ROS_WARN("Skipping line due to insufficient columns (expected at least 2 for X, Y): %s", line.c_str());
            }
        }
        file.close();
    }

    // 타이머 콜백 함수: 10Hz마다 호출되어 경로를 발행
    void timerCallback(const ros::TimerEvent& event) {
        // 경로 데이터가 있을 경우에만 발행
        if (!global_path_.poses.empty()) {
            // 메시지의 타임스탬프를 현재 시간으로 업데이트하여 발행
            global_path_.header.stamp = ros::Time::now();
            path_pub_.publish(global_path_);
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Timer pub_timer_;
    nav_msgs::Path global_path_; // 로드된 전체 경로를 저장하는 변수
    double UTM_X_OFFSET;
    double UTM_Y_OFFSET;

    // double UTM_X_OFFSET=360825.8208815998;
    // double UTM_Y_OFFSET=4065896.298577933;
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_path_publisher");
    GlobalPathPublisher publisher;
    ros::spin();
    return 0;
}