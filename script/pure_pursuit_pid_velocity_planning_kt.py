#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from geometry_msgs.msg import PointStamped
import numpy as np
from tf.transformations import euler_from_quaternion

class pure_pursuit:
    def __init__(self):
        # 1) 노드 초기화
        rospy.init_node('pure_pursuit', anonymous=True)

        # 2) 전역 경로 CSV 파일 로드
        pkg_path = rospkg.RosPack().get_path('provin_ground')
        csv_param = os.path.join(pkg_path, 'path', 'path_xy.csv')
        self.global_path = self.load_path_from_csv(csv_param)
        self.is_global_path = True  # 경로 로드 완료 플래그

        # 3) 로컬(회피) 경로, odom, 차량 상태 구독자 설정
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        # 4) LiDAR 클러스터 중심 구독자 추가
        rospy.Subscriber("/cluster_positions", PointStamped, self.cluster_callback)
        # 클러스터 수신 여부 및 저장 버퍼
        self.cluster_list = []             # [(x,y), ...]
        self.cluster_stamp = rospy.Time(0) # 마지막 프레임 타임스탬프
        self.is_cluster = False            # 클러스터 준비 플래그

        # 5) 제어 명령 퍼블리셔 설정
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1  # 가속/제동 모드

        # 6) 상태 플래그 초기화
        self.is_path = False
        self.is_odom = False
        self.is_status = False

        # 7) 내부 변수 초기화
        self.is_look_forward_point = False
        self.forward_point = Point()
        self.current_postion = Point()

        # 8) Pure Pursuit 및 PID 파라미터
        self.vehicle_length = 2.6   # 전축 간 거리 (m)
        self.lfd = 8                # 초기 look‐ahead 거리 (m)
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 15   # 목표 속도 (km/h)

        # 9) PID 컨트롤러, 속도 플래닝 인스턴스 생성
        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)

        # 10) 전역 경로 준비 대기
        while not rospy.is_shutdown():
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        # 11) 메인 루프: 30Hz
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # 로컬 경로, odom, 차량 상태, LiDAR 클러스터 모두 준비되면
            if self.is_path and self.is_odom and self.is_status and self.is_cluster:
                # 12) 현재 waypoint 인덱스 계산
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6

                # 13) 순수 추종 스티어링 계산
                steering = self.calc_pure_pursuit()

                # 14) look‐ahead 포인트 주변에 장애물(LiDAR 클러스터)이 있으면 비상 정지
                if self.check_collision_ahead(self.forward_point):
                    rospy.logwarn("LiDAR 장애물 감지! 비상 제동 실행")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0  # 풀 브레이크
                else:
                    # 정상 주행
                    if self.is_look_forward_point:
                        self.ctrl_cmd_msg.steering = steering
                    else:
                        rospy.loginfo("no found forward point")
                        self.ctrl_cmd_msg.steering = 0.0

                    # 15) PID 속도 제어
                    output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)
                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output

                # 16) 제어 명령 퍼블리시
                print(f"Steering: {steering:.3f}, Target Vel: {self.target_velocity:.1f}, Clusters: {len(self.cluster_list)}")
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def load_path_from_csv(self, filename):
        """
        CSV → nav_msgs/Path 메시지 변환
        헤더 자동 스킵, X,Y → PoseStamped
        """
        import csv
        path_msg = Path()
        path_msg.header.frame_id = '/map'
        path_msg.header.stamp = rospy.Time.now()

        with open(filename, 'r') as f:
            reader = csv.reader(f)
            first = next(reader)
            try:
                float(first[0]); float(first[1])
                rows = [first] + list(reader)
            except ValueError:
                rows = list(reader)

        for row in rows:
            if len(row) < 2:
                continue
            x, y = float(row[0]), float(row[1])
            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        rospy.loginfo(f"Loaded {len(path_msg.poses)} waypoints from CSV: {filename}")
        return path_msg

    # ───────────────────────────────────────────
    # 콜백 영역
    # ───────────────────────────────────────────

    def path_callback(self, msg):
        # /lattice_path 수신
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        # /odom 수신 → 차량 위치·yaw 갱신
        self.is_odom = True
        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(quat)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):
        # /Ego_topic 수신 → 속도 등 상태 저장
        self.is_status = True
        self.status_msg = msg

    def cluster_callback(self, msg):
        """
        /cluster_positions(PointStamped) 수신
        같은 프레임에 여러 클러스터가 순차적으로 날아오므로,
        stamp가 바뀔 때만 리스트 리셋 후 누적 저장
        """
        if msg.header.stamp != self.cluster_stamp:
            self.cluster_list = []
            self.cluster_stamp = msg.header.stamp
        self.cluster_list.append((msg.point.x, msg.point.y))
        self.is_cluster = True

    # ───────────────────────────────────────────
    # Pure Pursuit 내부 유틸
    # ───────────────────────────────────────────

    def get_current_waypoint(self, ego_status, global_path):
        # 차량과 가장 가까운 전역 경로 상의 인덱스 검색
        min_dist = float('inf')
        currnet_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y
            dist = sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self):
        # Look‐ahead 거리 동적 계산
        self.lfd = self.status_msg.velocity.x * self.lfd_gain
        self.lfd = max(self.min_lfd, min(self.lfd, self.max_lfd))
        rospy.loginfo(f"lfd: {self.lfd:.2f}")

        # 좌표변환 (map → vehicle)
        trans = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), self.current_postion.x],
            [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), self.current_postion.y],
            [0,                      0,                     1]
        ])
        inv_trans = np.linalg.inv(trans)

        # look‐ahead 포인트 탐색
        local_path = self.path.poses
        self.is_look_forward_point = False
        for pose_stamped in local_path:
            gp = pose_stamped.pose.position
            gp_h = np.array([gp.x, gp.y, 1.0])
            lp = inv_trans.dot(gp_h)
            if lp[0] > 0 and sqrt(lp[0]**2 + lp[1]**2) >= self.lfd:
                self.forward_point = gp
                self.is_look_forward_point = True
                break

        # 스티어링 각도 계산
        theta = atan2(lp[1], lp[0])
        return atan2(2 * self.vehicle_length * sin(theta), self.lfd)

    # ───────────────────────────────────────────
    # LiDAR 기반 충돌 검사
    # ───────────────────────────────────────────

    def check_collision_ahead(self, look_pt):
        """
        look_pt(look‐ahead 포인트) 주위 반경 2.35m 이내
        LiDAR 클러스터가 있으면 True(충돌 위험)
        """
        for ox, oy in self.cluster_list:
            dx = look_pt.x - ox
            dy = look_pt.y - oy
            if dx*dx + dy*dy < 2.35**2:
                return True
        return False

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        # 속도 오차 기반 PID → accel/brake
        error = target_vel - current_vel
        p = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d = self.d_gain * (error - self.prev_error) / self.controlTime
        self.prev_error = error
        return p + self.i_control + d

class velocityPlanning:
    def __init__(self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, global_path, point_num):
        # 곡률 기반 속도 플래닝 (원본 유지)
        out_vel = [self.car_max_speed] * point_num
        for i in range(point_num, len(global_path.poses) - point_num):
            out_vel.append(self.car_max_speed)
        out_vel += [30] * 10 + [0] * 10
        return out_vel

if __name__ == '__main__':
    try:
        pure_pursuit()
    except rospy.ROSInterruptException:
        pass
