#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pure_pursuit_pid_velocity_planning_kt.py 를 기반으로
#  lidar의 장애물 위치 토픽을 받아서, local_path 선택에 반영하도록 수정!



#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from morai_msgs.msg  import EgoVehicleStatus
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import Path
import numpy as np

class latticePlanner:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('lattice_planner', anonymous=True)

        # 구독자 설정
        rospy.Subscriber("/local_path", Path, self.path_callback)           # 기준 로컬 경로
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)  # 차량 상태
        rospy.Subscriber("/cluster_positions", PointStamped, self.cluster_callback)  # LiDAR 클러스터

        # 회피 경로 퍼블리셔
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)

        # 상태 플래그
        self.is_path = False
        self.is_status = False
        self.is_obj = False  # 클러스터 수신 여부로 재사용

        # LiDAR 클러스터 저장
        self.cluster_list = []
        self.cluster_stamp = rospy.Time(0)

        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            # 로컬 경로, 차량 상태, 클러스터 정보 모두 수신되면 처리
            if self.is_path and self.is_status and self.is_obj:
                # 클러스터 충돌 검사
                if self.checkObject(self.local_path):
                    # 충돌 예상 시 후보 경로 생성 및 비용 계산
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(lattice_path)
                    # 선택된 회피 경로 퍼블리시
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    # 충돌 없으면 원본 경로 유지
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def path_callback(self, msg):
        # 로컬 경로 수신
        self.is_path = True
        self.local_path = msg

    def status_callback(self, msg):
        # 차량 상태 수신
        self.is_status = True
        self.status_msg = msg

    def cluster_callback(self, msg):
        # LiDAR 클러스터 중심 수신
        if msg.header.stamp != self.cluster_stamp:
            self.cluster_list = []
            self.cluster_stamp = msg.header.stamp
        self.cluster_list.append((msg.point.x, msg.point.y))
        self.is_obj = True  # 클러스터 정보 준비 완료

    def checkObject(self, ref_path):
        """
        ref_path 상에 LiDAR 클러스터가
        일정 거리(2.35m) 이내에 있으면 True 반환
        """
        for ox, oy in self.cluster_list:
            for wp in ref_path.poses:
                dx = wp.pose.position.x - ox
                dy = wp.pose.position.y - oy
                if dx*dx + dy*dy < 2.35**2:
                    return True
        return False

    def collision_check(self, out_path):
        """
        LiDAR 클러스터만 반영해
        각 후보 경로별 가중치 계산 후 최저 가중치 인덱스 반환
        """
        lane_weight = [3, 2, 1, 1, 2, 3]

        for ox, oy in self.cluster_list:
            for idx, path in enumerate(out_path):
                for wp in path.poses:
                    dx = ox - wp.pose.position.x
                    dy = oy - wp.pose.position.y
                    if dx*dx + dy*dy < 1.5**2:
                        lane_weight[idx] += 100

        return lane_weight.index(min(lane_weight))

    def latticePlanner(self, ref_path, vehicle_status):
        """
        기존 latticePlanner.latticePlanner() 로직 유지
        ref_path와 차량 상태로부터 6개 후보 경로 생성 후 리스트 반환
        """
        out_path = []
        vx = vehicle_status.position.x
        vy = vehicle_status.position.y
        v_kmh = vehicle_status.velocity.x * 3.6

        # look-ahead 거리 계산
        look_distance = int(v_kmh * 0.2 * 2)
        if look_distance < 20:
            look_distance = 20

        if len(ref_path.poses) > look_distance:
            # 좌표 변환 매트릭스 생성
            p0 = ref_path.poses[0].pose.position
            p1 = ref_path.poses[1].pose.position
            p_end = ref_path.poses[look_distance*2].pose.position

            theta = atan2(p1.y - p0.y, p1.x - p0.x)
            trans = [p0.x, p0.y]
            T = np.array([[cos(theta), -sin(theta), trans[0]],
                          [sin(theta),  cos(theta), trans[1]],
                          [0,            0,          1        ]])
            T_inv = np.array([
                [T[0,0], T[1,0], -(T[0,0]*trans[0] + T[1,0]*trans[1])],
                [T[0,1], T[1,1], -(T[0,1]*trans[0] + T[1,1]*trans[1])],
                [0,      0,       1                                 ]
            ])

            # 로컬 좌표 변환
            end_global = np.array([[p_end.x], [p_end.y], [1]])
            end_local = T_inv.dot(end_global)
            ego_global = np.array([[vx], [vy], [1]])
            ego_local = T_inv.dot(ego_global)

            lane_offset = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_ends = [[end_local[0,0], end_local[1,0]+d, 1] for d in lane_offset]

            # 후보 경로 생성
            for ep in local_ends:
                path_msg = Path(); path_msg.header.frame_id = 'map'
                xs, ys = 0, ego_local[1,0]
                xf, yf = ep[0], ep[1]
                x_interval = 0.5
                x_points = np.arange(xs, xf, x_interval)

                a0 = ys
                a2 = 3*(yf - ys)/(xf**2)
                a3 = -2*(yf - ys)/(xf**3)

                for x in x_points:
                    y = a3*x**3 + a2*x**2 + a0
                    local_pt = np.array([[x], [y], [1]])
                    global_pt = T.dot(local_pt)
                    pose = PoseStamped()
                    pose.pose.position.x = global_pt[0,0]
                    pose.pose.position.y = global_pt[1,0]
                    pose.pose.position.z = 0
                    pose.pose.orientation.w = 1
                    path_msg.poses.append(pose)

                out_path.append(path_msg)

            # 종점 이후 웨이포인트 연결
            add_size = min(int(v_kmh*2), len(ref_path.poses))
            for i in range(look_distance*2, add_size):
                if i+1 < len(ref_path.poses):
                    p_cur = ref_path.poses[i].pose.position
                    p_next = ref_path.poses[i+1].pose.position
                    tmp_theta = atan2(p_next.y-p_cur.y, p_next.x-p_cur.x)
                    tmp_trans = [p_cur.x, p_cur.y]
                    tmp_T = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_trans[0]],
                                      [sin(tmp_theta),  cos(tmp_theta), tmp_trans[1]],
                                      [0,               0,              1           ]])
                    for idx, d in enumerate(lane_offset):
                        local_off = np.array([[0], [d], [1]])
                        global_off = tmp_T.dot(local_off)
                        pose = PoseStamped()
                        pose.pose.position.x = global_off[0,0]
                        pose.pose.position.y = global_off[1,0]
                        pose.pose.position.z = 0
                        pose.pose.orientation.w = 1
                        out_path[idx].poses.append(pose)

        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass



