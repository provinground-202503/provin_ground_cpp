#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class read_path_pub:
    """
    전역 경로를 CSV에서 읽어서 /global_path 토픽으로 퍼블리시합니다.
    - 클래스명, 변수명, 함수명은 기존과 동일하게 유지했습니다.
    """
    def __init__(self):
        # 노드 초기화
        rospy.init_node('read_path_pub', anonymous=True)
        # 퍼블리셔 설정
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        # Path 메시지 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        # 'provin_ground' 패키지에서 CSV 파일 경로 얻기
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('provin_ground')
        csv_path = pkg_path + '/path/path_xy.csv'

        # CSV 파일 읽기
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            first = next(reader)
            # 첫 줄이 헤더(X,Y)라면 스킵, 아니면 포함
            try:
                float(first[0]); float(first[1])
                rows = [first] + list(reader)
            except ValueError:
                rows = list(reader)

        # 각 행을 PoseStamped로 변환해 Path에 추가
        for row in rows:
            if len(row) < 2:
                continue
            x = float(row[0])
            y = float(row[1])
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            self.global_path_msg.poses.append(pose)

        rospy.loginfo(f"Loaded {len(self.global_path_msg.poses)} waypoints from CSV: {csv_path}")

        # 주기적으로 퍼블리시
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            # 헤더 타임스탬프 갱신
            self.global_path_msg.header.stamp = rospy.Time.now()
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        read_path_pub()
    except rospy.ROSInterruptException:
        pass
