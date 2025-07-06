#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
# YOUR_PACKAGE_NAME.msg 대신 실제 메시지 파일 경로를 사용합니다.
# 예: from erp_driver.msg import erpStatusMsg
# 예: from car_state_msgs.msg import CarState
from erp_driver.msg import ErpStatusMsg # erpStatusMsg 메시지 정의 파일
from provin_ground.msg import CarState     # carState 메시지 정의 파일

import numpy as np
from scipy.linalg import block_diag
from math import sin, cos, tan, radians, degrees
import tf # ROS1 tf library for quaternion conversion

# EKF 파라미터 (예시 값, 실제 차량에 맞게 튜닝 필요)
# Process Noise Covariance (상태 전이 모델의 불확실성)
Q_COVARIANCE = np.diag([
    0.01,  # x_variance
    0.01,  # y_variance
    0.1,   # velocity_x_variance
    0.1,   # velocity_y_variance
    0.01,  # yaw_variance
    0.01   # yawrate_variance
])

# Measurement Noise Covariance for UTM
R_COVARIANCE_UTM = np.diag([
    0.1,   # x_measurement_variance
    0.1,   # y_measurement_variance
    0.05   # yaw_measurement_variance
])

# Measurement Noise Covariance for IMU (yawrate only for EKF update)
R_COVARIANCE_IMU_YAWRATE = np.diag([
    0.05   # yawrate_measurement_variance
])

# Measurement Noise Covariance for ERP (velocity_x only for EKF update)
R_COVARIANCE_ERP_VELOCITY = np.diag([
    0.1    # velocity_x_measurement_variance
])

# 차량 파라미터 (예시 값, 실제 차량에 맞게 튜닝 필요)
WHEELBASE = 2.0 # 차량 휠베이스 (미터)

class CarStateEstimator:
    def __init__(self):
        rospy.init_node('car_state_estimator', anonymous=True)

        self.car_state_publisher = rospy.Publisher('/car_state', CarState, queue_size=10)

        # 센서 구독
        self.utm_subscriber = rospy.Subscriber(
            '/utm',
            PoseStamped,
            self.utm_callback,
            queue_size=10
        )
        self.imu_subscriber = rospy.Subscriber(
            '/imu_fix',
            Imu,
            self.imu_callback,
            queue_size=10
        )
        self.erp_status_subscriber = rospy.Subscriber(
            '/erp_status',
            ErpStatusMsg,
            self.erp_status_callback,
            queue_size=10
        )

        # EKF 상태 변수 초기화
        # [x, y, velocity_x, velocity_y, yaw, yawrate]
        self.x_hat = np.zeros(6) # 추정된 상태 벡터
        self.P = np.eye(6) * 1000.0 # 오차 공분산 행렬 (초기 불확실성을 높게 설정)

        # 마지막 수신 시간 저장 (델타 시간 계산용)
        self.last_update_time = rospy.Time.now()

        # 최근 센서 데이터 저장 (EKF 갱신에 사용)
        self.latest_utm_data = None
        self.latest_imu_data = None
        self.latest_erp_status_data = None

        # 타이머를 사용하여 EKF 루프 주기적으로 실행
        self.ekf_timer = rospy.Timer(rospy.Duration(0.02), self.ekf_loop) # 50 Hz (20ms)

        rospy.loginfo('CarState Estimator Node Initialized for ROS1 Noetic!')

    def utm_callback(self, msg):
        # /utm 토픽에서 x, y, yaw(quaternion을 radian으로 변환) 추출
        # Quaternion to Euler (yaw) using tf
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, 
                            msg.pose.orientation.z, msg.pose.orientation.w]
        (roll, pitch, yaw_rad) = tf.transformations.euler_from_quaternion(orientation_list)

        self.latest_utm_data = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': yaw_rad,
            'stamp': msg.header.stamp
        }

    def imu_callback(self, msg):
        # /imu_fix 토픽에서 가속도(body frame)와 yaw rate 추출
        self.latest_imu_data = {
            'accel_x': msg.linear_acceleration.x,  # X-axis acceleration in body frame
            'accel_y': msg.linear_acceleration.y,  # Y-axis acceleration in body frame
            'yawrate': msg.angular_velocity.z,     # Z-axis angular velocity (yaw rate)
            'stamp': msg.header.stamp
        }

    def erp_status_callback(self, msg):
        # /erp_status 토픽에서 속도(km/h -> m/s)와 조향각(degree -> radian) 추출
        # msg.speed: km/h * 0.1 -> m/s
        # msg.steer: degree (ERP steer value) -> radian
        self.latest_erp_status_data = {
            'velocity_x_erp': msg.speed * 0.1 / 3.6, # km/h * 0.1 -> m/s
            'steering_angle': radians(msg.steer / 71.0), # ERP steer value (0~2000) to degree and then radian. (adjust divisor 71.0 as needed)
            'stamp': rospy.Time.now() # erpStatusMsg에 header가 없을 경우 현재 시간 사용
        }

    def ekf_loop(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec() # s
        self.last_update_time = current_time

        if dt <= 0: # 시간 역전 방지
            return

        # 1. Prediction Step
        self._predict_state(dt)

        # 2. Update Step (if measurements available)
        # UTM 데이터 갱신
        if self.latest_utm_data is not None and \
           (current_time - self.latest_utm_data['stamp']).to_sec() < 0.1: # 최근 0.1초 이내 데이터만 사용
            z_utm = np.array([
                self.latest_utm_data['x'],
                self.latest_utm_data['y'],
                self.latest_utm_data['yaw']
            ])
            H_utm = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0]
            ]) # Measurement matrix for UTM (x, y, yaw)
            self._update_state(z_utm, H_utm, R_COVARIANCE_UTM)
            self.latest_utm_data = None # 사용한 데이터는 초기화하여 중복 사용 방지

        # IMU 데이터 갱신 (yawrate만 EKF 업데이트에 사용)
        if self.latest_imu_data is not None and \
           (current_time - self.latest_imu_data['stamp']).to_sec() < 0.1:
            z_imu_yawrate = np.array([self.latest_imu_data['yawrate']])
            H_imu_yawrate = np.array([[0, 0, 0, 0, 0, 1]]) # Measurement matrix for IMU yawrate
            self._update_state(z_imu_yawrate, H_imu_yawrate, R_COVARIANCE_IMU_YAWRATE)
            self.latest_imu_data = None

        # ERP Status 데이터 갱신 (velocity_x만 EKF 업데이트에 사용)
        if self.latest_erp_status_data is not None and \
           (current_time - self.latest_erp_status_data['stamp']).to_sec() < 0.1:
            z_erp_vel = np.array([self.latest_erp_status_data['velocity_x_erp']])
            H_erp_vel = np.array([[0, 0, 1, 0, 0, 0]]) # Measurement matrix for velocity_x
            self._update_state(z_erp_vel, H_erp_vel, R_COVARIANCE_ERP_VELOCITY)
            self.latest_erp_status_data = None


        # 3. Publish CarState
        self._publish_car_state(current_time)

    def _predict_state(self, dt):
        # 상태 벡터: [x, y, vx, vy, yaw, yawrate]
        x, y, vx, vy, yaw, yawrate = self.x_hat

        # IMU 가속도 (body frame)
        accel_x_imu = self.latest_imu_data['accel_x'] if self.latest_imu_data else 0.0
        accel_y_imu = self.latest_imu_data['accel_y'] if self.latest_imu_data else 0.0

        # 예측된 상태 (xk+1 = f(xk, uk))
        # vx, vy는 글로벌 기준의 속도가 아닌, 차량 본체 기준의 종/횡방향 속도로 정의하는게 EKF 모델링에 더 자연스러움
        # 여기서는 편의상 글로벌 좌표계에서의 속도로 EKF 상태를 구성
        # 하지만 실제 차량 동역학 모델은 종/횡방향 속도, 조향각, yawrate 등을 입력으로 받아야 함
        # 현재 코드에서는 vx, vy를 글로벌로 간주하고, accel_x_imu, accel_y_imu를 활용 (이는 바디 프레임 가속도)
        # 복잡한 동역학 모델은 EKF의 Fk 계산을 복잡하게 하므로, 초기에는 간단하게 시작
        
        # 간단한 예측 모델: 현재 속도와 가속도를 이용하여 다음 위치 및 속도 예측
        # IMU accel_x/y는 바디 프레임이므로, 이를 글로벌 가속도로 변환하여 사용
        global_ax = accel_x_imu * cos(yaw) - accel_y_imu * sin(yaw)
        global_ay = accel_x_imu * sin(yaw) + accel_y_imu * cos(yaw)

        x_pred = x + (vx * dt) + (0.5 * global_ax * dt**2)
        y_pred = y + (vy * dt) + (0.5 * global_ay * dt**2)
        vx_pred = vx + global_ax * dt
        vy_pred = vy + global_ay * dt
        yaw_pred = yaw + yawrate * dt
        yawrate_pred = yawrate # Simple constant yawrate model

        # 상태 전이 자코비안 (Fk)
        # Fk = df/dx_hat |x_hat_k, u_k
        # f(x) = [
        #    x + vx * dt + 0.5 * (accel_x_imu * cos(yaw) - accel_y_imu * sin(yaw)) * dt^2
        #    y + vy * dt + 0.5 * (accel_x_imu * sin(yaw) + accel_y_imu * cos(yaw)) * dt^2
        #    vx + (accel_x_imu * cos(yaw) - accel_y_imu * sin(yaw)) * dt
        #    vy + (accel_x_imu * sin(yaw) + accel_y_imu * cos(yaw)) * dt
        #    yaw + yawrate * dt
        #    yawrate
        # ]

        Fk = np.array([
            [1, 0, dt, 0, 0.5 * (-accel_x_imu*sin(yaw) - accel_y_imu*cos(yaw))*dt**2, 0],
            [0, 1, 0, dt, 0.5 * (accel_x_imu*cos(yaw) - accel_y_imu*sin(yaw))*dt**2, 0],
            [0, 0, 1, 0, (-accel_x_imu*sin(yaw) - accel_y_imu*cos(yaw))*dt, 0],
            [0, 0, 0, 1, (accel_x_imu*cos(yaw) - accel_y_imu*sin(yaw))*dt, 0],
            [0, 0, 0, 0, 1, dt],
            [0, 0, 0, 0, 0, 1]
        ])

        # 예측된 오차 공분산
        self.P = Fk @ self.P @ Fk.T + Q_COVARIANCE

        self.x_hat = np.array([x_pred, y_pred, vx_pred, vy_pred, yaw_pred, yawrate_pred])

        # Yaw 각도를 -pi ~ pi 범위로 정규화
        self.x_hat[4] = np.arctan2(sin(self.x_hat[4]), cos(self.x_hat[4]))


    def _update_state(self, z, H, R):
        # 갱신 단계
        # y_k = z - h(x_hat_pred)
        # 여기서 h(x)는 H @ x_hat이므로, y_k = z - (H @ x_hat)
        y_k = z - (H @ self.x_hat) # Innovation (측정 오차)
        S_k = H @ self.P @ H.T + R # Innovation covariance
        K_k = self.P @ H.T @ np.linalg.inv(S_k) # Kalman Gain

        self.x_hat = self.x_hat + (K_k @ y_k) # 상태 갱신
        self.P = (np.eye(self.x_hat.shape[0]) - K_k @ H) @ self.P # 오차 공분산 갱신

        # Yaw 각도를 -pi ~ pi 범위로 정규화 (갱신 후에도 적용)
        self.x_hat[4] = np.arctan2(sin(self.x_hat[4]), cos(self.x_hat[4]))


    def _publish_car_state(self, current_time):
        msg = CarState()
        msg.header.stamp = current_time
        msg.header.frame_id = 'map' # 또는 'odom' 등 적절한 좌표계 명시

        # EKF 추정값
        msg.x = self.x_hat[0]
        msg.y = self.x_hat[1]
        
        # velocity_lon, velocity_lat
        # EKF 상태 벡터의 vx, vy는 글로벌 좌표계 기준이므로, 이를 차량 본체 기준으로 변환
        msg.velocity_lon = self.x_hat[2] * cos(self.x_hat[4]) + self.x_hat[3] * sin(self.x_hat[4]) # vx_global * cos(yaw) + vy_global * sin(yaw)
        msg.velocity_lat = -self.x_hat[2] * sin(self.x_hat[4]) + self.x_hat[3] * cos(self.x_hat[4]) # -vx_global * sin(yaw) + vy_global * cos(yaw)
        
        msg.dx = self.x_hat[2]
        msg.dy = self.x_hat[3]

        # accel_lon, accel_lat (IMU에서 직접 가져옴)
        # IMU는 바디 프레임 기준의 가속도를 제공하므로, EKF와는 별개로 직접 사용
        msg.accel_lon = self.latest_imu_data['accel_x'] if self.latest_imu_data else 0.0
        msg.accel_lat = self.latest_imu_data['accel_y'] if self.latest_imu_data else 0.0
        
        # ax, ay (글로벌 가속도)
        # IMU 가속도 (body frame)를 EKF 추정 yaw를 사용하여 글로벌 프레임으로 변환
        imu_ax_body = self.latest_imu_data['accel_x'] if self.latest_imu_data else 0.0
        imu_ay_body = self.latest_imu_data['accel_y'] if self.latest_imu_data else 0.0
        
        msg.ax = imu_ax_body * cos(self.x_hat[4]) - imu_ay_body * sin(self.x_hat[4])
        msg.ay = imu_ax_body * sin(self.x_hat[4]) + imu_ay_body * cos(self.x_hat[4])

        msg.yaw = self.x_hat[4]
        msg.yawrate = self.x_hat[5]

        # Steering Angle (ERP Status에서 직접 가져옴)
        msg.steering_angle = self.latest_erp_status_data['steering_angle'] if self.latest_erp_status_data else 0.0

        self.car_state_publisher.publish(msg)


if __name__ == '__main__':
    try:
        CarStateEstimator()
        rospy.spin() # ROS 노드를 계속 실행
    except rospy.ROSInterruptException:
        pass