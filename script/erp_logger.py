#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from datetime import datetime
from erp_driver.msg import erpCmdMsg  # erpCmdMsg 메시지 타입을 import 합니다.

class ErpCommandLogger:
    """
    '/erp_command' 토픽에서 erpCmdMsg 메시지를 구독하고,
    수신된 제어 데이터를 타임스탬프와 함께 CSV 파일로 저장하는 클래스입니다.
    """
    def __init__(self):
        # 1. ROS 노드 초기화
        rospy.init_node('erp_command_logger', anonymous=True)

        # 2. CSV 파일 설정
        # 사용자의 홈 디렉터리에 타임스탬프를 포함한 고유한 파일 이름으로 저장합니다.
        now = datetime.now()
        filename = f"erp_log_{now.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        # os.path.join을 사용하여 OS에 독립적인 경로를 생성합니다.
        self.log_file_path = os.path.join(os.path.expanduser('~'), filename)

        rospy.loginfo(f"ERP42 제어 명령을 다음 경로에 저장합니다: {self.log_file_path}")

        # CSV 파일을 쓰기 모드로 열고, writer 객체를 생성합니다.
        # newline='' 옵션은 CSV 파일에 빈 줄이 추가되는 것을 방지합니다.
        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # CSV 파일의 헤더(첫 번째 행)를 작성합니다.
        self.csv_writer.writerow(['timestamp', 'e_stop', 'gear', 'speed', 'steer', 'brake'])

        # 3. Subscriber 설정
        # C++ 코드에서 발행(publish)하는 토픽 이름을 적어주어야 합니다.
        # 일반적으로 '/erp_command' 또는 '/cmd_msg'와 같은 이름을 사용합니다.
        self.subscriber = rospy.Subscriber(
            '/erp42_ctrl_cmd',  # <-- 발행하는 토픽 이름에 맞게 수정이 필요할 수 있습니다.
            erpCmdMsg,
            self.command_callback
        )

        # 4. 종료 시 파일 정리(cleanup)를 위한 Hook 설정
        # Ctrl+C로 노드를 종료할 때 self.cleanup 함수가 호출됩니다.
        rospy.on_shutdown(self.cleanup)

    def command_callback(self, msg):
        """erpCmdMsg 메시지를 수신했을 때 호출되는 콜백 함수"""
        try:
            # 현재 ROS 시간을 타임스탬프로 사용합니다.
            timestamp = rospy.get_time()

            # CSV 파일에 쓸 데이터 행(row)을 준비합니다.
            row = [
                f"{timestamp:.3f}",  # 소수점 3자리까지 표시
                msg.e_stop,
                msg.gear,
                msg.speed,
                msg.steer,
                msg.brake
            ]

            # 준비된 데이터를 CSV 파일에 씁니다.
            self.csv_writer.writerow(row)

            # (선택) 데이터가 수신되고 있음을 확인하기 위해 콘솔에 로그를 출력합니다.
            # rospy.loginfo(f"명령 기록: Steer={msg.steer}, Speed={msg.speed}")

        except Exception as e:
            rospy.logerr(f"콜백 함수에서 에러 발생: {e}")

    def cleanup(self):
        """노드 종료 시 CSV 파일을 안전하게 닫습니다."""
        if self.csv_file:
            self.csv_file.close()
        rospy.loginfo(f"CSV 파일이 성공적으로 저장 및 종료되었습니다: {self.log_file_path}")

if __name__ == '__main__':
    try:
        ErpCommandLogger()
        # rospy.spin()은 노드가 종료될 때까지 콜백 함수를 계속해서 호출 대기 상태로 둡니다.
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("erp_command_logger 노드가 종료되었습니다.")