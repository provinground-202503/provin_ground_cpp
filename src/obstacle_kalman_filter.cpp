#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <opencv2/video/tracking.hpp> // 칼만 필터를 위해 OpenCV 포함
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/**
 * @class TrackedObstacle
 * @brief 개별 장애물을 추적하고 칼만 필터를 적용하는 클래스
 */
class TrackedObstacle {
public:
    cv::KalmanFilter kf_;               // OpenCV 칼만 필터 객체
    ros::Time last_seen_;               // 장애물이 마지막으로 관측된 시간
    int coasting_age_ = 0;              // 관측되지 않은 횟수 (프레임)
    int id_;                            // 추적 객체의 고유 ID
    geometry_msgs::Point predicted_pos_;// 예측된 위치

    /**
     * @brief 생성자. 칼만 필터를 초기화합니다.
     * @param initial_pos 장애물의 초기 감지 위치 (전역 좌표)
     * @param id 고유 ID
     */
    TrackedObstacle(const geometry_msgs::Point& initial_pos, int id) : id_(id) {
        // 칼만 필터 초기화: 상태 변수 4개(x, y, vx, vy), 측정 변수 2개(x, y)
        kf_ = cv::KalmanFilter(4, 2, 0, CV_32F);

        // 상태 변수 초기값 설정
        kf_.statePost.at<float>(0) = initial_pos.x;
        kf_.statePost.at<float>(1) = initial_pos.y;
        kf_.statePost.at<float>(2) = 0; // 초기 속도는 0으로 가정
        kf_.statePost.at<float>(3) = 0;

        // 상태 전이 행렬 (A) - 등속도 모델 가정
        // x_k = x_{k-1} + vx * dt
        // y_k = y_{k-1} + vy * dt
        // 이 행렬은 predict 함수에서 매번 dt에 따라 갱신됩니다.
        cv::setIdentity(kf_.transitionMatrix);

        // 측정 행렬 (H) - 위치(x, y)만 측정 가능
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
                                                          0, 1, 0, 0);

        // 프로세스 노이즈 공분산 (Q) - 모델의 불확실성을 표현
        // 시간이 지남에 따라 속도가 변할 수 있다는 불확실성을 크게 설정
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));
        kf_.processNoiseCov.at<float>(2,2) = 1.0; // 속도 변화에 대한 불확실성을 더 크게
        kf_.processNoiseCov.at<float>(3,3) = 1.0;


        // 측정 노이즈 공분산 (R) - 센서 측정값의 불확실성을 표현
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));

        // 오차 공분산 행렬 (P) - 초기 오차 공분산
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

        last_seen_ = ros::Time::now();
    }

    /**
     * @brief 칼만 필터의 예측 단계를 수행합니다.
     * @param dt 이전 프레임과의 시간 간격
     * @return 예측된 장애물 위치
     */
    geometry_msgs::Point predict(double dt) {
        // dt에 따라 상태 전이 행렬 갱신
        kf_.transitionMatrix.at<float>(0, 2) = dt;
        kf_.transitionMatrix.at<float>(1, 3) = dt;

        cv::Mat prediction = kf_.predict();
        predicted_pos_.x = prediction.at<float>(0);
        predicted_pos_.y = prediction.at<float>(1);
        predicted_pos_.z = 0; // 2D 평면에서 추적

        return predicted_pos_;
    }

    /**
     * @brief 칼만 필터의 수정(업데이트) 단계를 수행합니다.
     * @param measurement 실제 측정된 장애물 위치
     */
    void correct(const geometry_msgs::Point& measurement) {
        cv::Mat measurement_mat = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);
        kf_.correct(measurement_mat);

        last_seen_ = ros::Time::now();
        coasting_age_ = 0; // 다시 관측되었으므로 coasting age 초기화
    }
};

/**
 * @class ObstacleKalmanFilter
 * @brief 장애물 클러스터를 받아 칼만 필터로 추적하고 결과를 발행하는 ROS 노드 클래스
 */
class ObstacleKalmanFilter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_clusters_;
    ros::Subscriber sub_utm_;
    ros::Publisher pub_filtered_obstacles_;

    std::vector<TrackedObstacle> tracked_obstacles_;
    geometry_msgs::PoseStamped current_utm_;
    double current_vehicle_yaw_ = 0.0;
    ros::Time last_update_time_;
    int next_id_ = 0;

    // --- 튜닝 파라미터 ---
    double association_threshold_ = 1.0; // 데이터 연관을 위한 최대 거리 (m)
    int max_coasting_age_ = 5;           // 장애물이 보이지 않아도 추적을 유지할 최대 프레임 수

public:
    ObstacleKalmanFilter() : nh_("~") { // Private NodeHandle for parameters
        sub_clusters_ = nh_.subscribe("/lidar_clusters", 1, &ObstacleKalmanFilter::clusteredCallback, this);
        sub_utm_ = nh_.subscribe("/utm", 1, &ObstacleKalmanFilter::utmCallback, this);
        pub_filtered_obstacles_ = nh_.advertise<visualization_msgs::MarkerArray>("/filtered_obstacles", 10);

        last_update_time_ = ros::Time::now();
        ROS_INFO("Obstacle Kalman Filter node has started.");
    }

    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        static geometry_msgs::PoseStamped previous_utm;

        current_utm_ = *msg;

        // Yaw 계산 (두 위치 간의 방향으로 계산, 더 안정적일 수 있음)
        if (!previous_utm.header.stamp.isZero()) {
            double delta_x = current_utm_.pose.position.x - previous_utm.pose.position.x;
            double delta_y = current_utm_.pose.position.y - previous_utm.pose.position.y;
            // 이동 거리가 충분할 때만 Yaw 갱신
            if (std::hypot(delta_x, delta_y) > 0.1) {
                current_vehicle_yaw_ = std::atan2(delta_y, delta_x);
            }
        }
        previous_utm = current_utm_;
    }

    void clusteredCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        if (current_utm_.header.stamp.isZero()) {
            ROS_WARN_THROTTLE(2.0, "Cannot process obstacles, vehicle UTM position not available.");
            return;
        }

        // 1. 로컬 좌표 장애물을 전역(UTM) 좌표로 변환
        std::vector<geometry_msgs::Point> global_obstacles;
        double vehicle_x = current_utm_.pose.position.x;
        double vehicle_y = current_utm_.pose.position.y;
        
        for (const auto& marker : msg->markers) {
            double obs_local_x = marker.pose.position.x;
            double obs_local_y = marker.pose.position.y;

            geometry_msgs::Point global_obs_point;
            global_obs_point.x = vehicle_x + obs_local_x * std::cos(current_vehicle_yaw_) - obs_local_y * std::sin(current_vehicle_yaw_);
            global_obs_point.y = vehicle_y + obs_local_x * std::sin(current_vehicle_yaw_) + obs_local_y * std::cos(current_vehicle_yaw_);
            global_obs_point.z = current_utm_.pose.position.z;
            global_obstacles.push_back(global_obs_point);
        }

        // 2. 칼만 필터 추적 로직 수행
        processTracking(global_obstacles);
    }

    void processTracking(const std::vector<geometry_msgs::Point>& current_detections) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        if (dt <= 0.001) dt = 0.05; // dt가 너무 작거나 음수일 경우 기본값 사용
        last_update_time_ = current_time;

        // --- 예측 단계 ---
        for (auto& tracker : tracked_obstacles_) {
            tracker.predict(dt);
        }

        // --- 데이터 연관 (가장 가까운 이웃) ---
        std::vector<bool> detection_matched(current_detections.size(), false);
        std::vector<int> tracker_to_detection_map(tracked_obstacles_.size(), -1);

        for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
            double min_dist = association_threshold_;
            int best_match_idx = -1;

            for (size_t j = 0; j < current_detections.size(); ++j) {
                if (!detection_matched[j]) {
                    double dist = std::hypot(tracked_obstacles_[i].predicted_pos_.x - current_detections[j].x,
                                             tracked_obstacles_[i].predicted_pos_.y - current_detections[j].y);
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_match_idx = j;
                    }
                }
            }

            if (best_match_idx != -1) {
                tracker_to_detection_map[i] = best_match_idx;
                detection_matched[best_match_idx] = true;
            }
        }

        // --- 수정 및 추적 관리 ---
        std::vector<TrackedObstacle> next_tracked_obstacles;
        for (size_t i = 0; i < tracked_obstacles_.size(); ++i) {
            int detection_idx = tracker_to_detection_map[i];
            if (detection_idx != -1) { // 매칭된 추적 객체
                tracked_obstacles_[i].correct(current_detections[detection_idx]);
                next_tracked_obstacles.push_back(tracked_obstacles_[i]);
            } else { // 매칭되지 않은 추적 객체
                tracked_obstacles_[i].coasting_age_++;
                if (tracked_obstacles_[i].coasting_age_ <= max_coasting_age_) {
                    next_tracked_obstacles.push_back(tracked_obstacles_[i]);
                }
                // max_coasting_age_를 넘으면 추적 목록에서 자동 제거
            }
        }

        // --- 새로운 추적 시작 ---
        for (size_t i = 0; i < current_detections.size(); ++i) {
            if (!detection_matched[i]) {
                next_tracked_obstacles.emplace_back(current_detections[i], next_id_++);
            }
        }

        tracked_obstacles_ = next_tracked_obstacles;

        // --- 결과 시각화 ---
        publishFilteredObstacles();
    }

    void publishFilteredObstacles() {
        visualization_msgs::MarkerArray markers;
        for (const auto& tracker : tracked_obstacles_) {
            // 추적이 안정적인 객체만 발행 (예: 몇 프레임 이상 관측된 객체)
            if (tracker.coasting_age_ > 0) continue; // 예측만으로 존재하는 객체는 제외

            visualization_msgs::Marker marker;
            marker.header.frame_id = "utm"; // 전역 좌표계
            marker.header.stamp = ros::Time::now();
            marker.ns = "filtered_obstacles";
            marker.id = tracker.id_;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 칼만 필터의 최종 상태(statePost)를 사용
            marker.pose.position.x = tracker.kf_.statePost.at<float>(0);
            marker.pose.position.y = tracker.kf_.statePost.at<float>(1);
            marker.pose.position.z = 0.5;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.lifetime = ros::Duration(0.2);
            markers.markers.push_back(marker);
        }
        pub_filtered_obstacles_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_kalman_filter_node");
    ObstacleKalmanFilter filter_node;
    ros::spin();
    return 0;
}