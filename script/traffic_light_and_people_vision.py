#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TrafficLightVision 노드 로직 개요:
---------------------------------
1) `/camera/image_raw` 토픽을 구독하여 카메라 이미지를 수신합니다.
2) 프레임을 3×3 그리드로 분할한 뒤, 상단 중앙 셀(“2번 영역”)을 ROI로 사용합니다.
3) ROI 영역에 대해 YOLOv5를 실행하여 “traffic light” 바운딩 박스를 검출합니다.
4) 검출된 박스의 높이를 ROI 높이로 나눈 비율을 계산:
     - 임계치 이하 → 너무 멀리 있음 → 무시(“FAR”).
     - 임계치 이상 → 충분히 가까움 → 박스 영역을 잘라냅니다.
5) 잘라낸 박스를 HSV로 변환 후 색상 마스크 적용:
     - RED 마스크 (두 개의 색상 범위)
     - YELLOW 마스크 (Hue 약 15–35)
     - GREEN 마스크 (Hue 약 40–80)
6) 상태 결정:
     • RED+YELLOW 검출 → RED (고장 시나리오, 무조건 정지)  
     • YELLOW만 검출   → YELLOW (주의, 정지)  
     • GREEN만 검출    → GREEN (진행)  
     • 검출 없음        → CAUTION (정지)  
7) 결과 퍼블리시:
     • `/traffic_light/stop` (Bool)  
     • `/traffic_light/color` (String)  
8) (디버그용) ROI 및 바운딩 박스, 상태를 화면에 표시합니다.

파라미터:
  ~size_ratio_threshold: ROI 대비 신호등 박스 높이 비율 임계치 (기본값=0.15)
"""

"""

TrafficLightVision Node Logic Overview:
---------------------------------------
1) Subscribe to camera images on `/camera/image_raw`.
2) Define a 3×3 grid on each frame; use the top-center cell (region “2”) as ROI.
3) Run YOLOv5 on the ROI to detect “traffic light” bounding boxes.
4) Compute the detected box’s height relative to the ROI height:
     - If below threshold → too far → ignore (“FAR”).
     - If above threshold → close enough → crop the box region.
5) Convert the crop to HSV and apply color masks:
     - RED mask (two hue ranges)
     - YELLOW mask (hue ~15–35)
     - GREEN mask (hue ~40–80)
6) Decide state:
     • RED+YELLOW → RED (fault scenario, always stop)  
     • YELLOW only     → YELLOW (caution, stop)  
     • GREEN only      → GREEN (go)  
     • none            → CAUTION (stop)  
7) Publish:
     • `/traffic_light/stop` (Bool)  
     • `/traffic_light/color` (String)  
8) (Optional) Draw ROI & box + label on a debug window.

Parameter:
  ~size_ratio_threshold: ROI 대비 TL-box 높이 비율 임계치 (default=0.15)

"""

# 다음과 같이 기존 모델을 이용하려고 했으나
#  필터로 인한 낮은 해상도로 절반만 잡는 문제가 있었고 사람 인식도 거의 못해서

#  파인 튜닝 작업을 하는게 맞다고 판단함!

# 이거는 그냥 시행착오 단계 느낌으로 남겨 두었고, 추후 해당 파일 내부에서 모델 가져다가 쓰는 코드로 
#  변형할 예정임@

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError

class TrafficLightVision:
    def __init__(self):
        # 1) 노드 초기화
        rospy.init_node('traffic_light_and_people_vision', anonymous=True)
        self.bridge = CvBridge()

        # 2) 파라미터: 크기 비율 임계치 (ROI 대비 박스 높이 비율)
        self.size_ratio_th = rospy.get_param('~size_ratio_threshold', 0.15)

        # 3) 퍼블리셔: 정지 플래그, 색상 정보
        self.stop_pub  = rospy.Publisher('/traffic_light/stop',  Bool,   queue_size=1)
        self.color_pub = rospy.Publisher('/traffic_light/color', String, queue_size=1)

        # 4) YOLOv5 모델 로드 (ultralytics hub)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # 5) 카메라 이미지 구독
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.cb_image)

        rospy.loginfo(f"[TL Vision] started (size_ratio_th={self.size_ratio_th:.2f})")
        rospy.spin()

    def cb_image(self, msg: Image):
        # A) ROS Image → OpenCV BGR
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"[TL Vision] CvBridge Error: {e}")
            return

        H, W = img.shape[:2]

        # B) ROI 설정: 3×3 그리드 중 상단 중앙(2번 영역)
        cell_w, cell_h = W // 3, H // 3
        x0, x1 = cell_w, 2 * cell_w
        y0, y1 = 0,      cell_h
        roi = img[y0:y1, x0:x1]

        # C) YOLO 추론 (ROI 기준 좌표)
        results = self.model(roi)
        df = results.pandas().xyxy[0]
        tl_df = df[df['name'] == 'traffic light']

        # 기본값: 진행, 색상 NONE
        stop_flag = False
        color_str = "NONE"

        if not tl_df.empty:
            # 첫 번째 검출만 사용
            r = tl_df.iloc[0]

            # ROI 좌표 → 전체 이미지 좌표 변환
            x1b = int(r.xmin) + x0
            y1b = int(r.ymin) + y0
            x2b = int(r.xmax) + x0
            y2b = int(r.ymax) + y0

            # D) 바운딩박스 높이 → ROI 높이 비율 계산
            box_h     = y2b - y1b
            size_ratio = box_h / float(cell_h)

            # E) 충분히 가까워졌을 때만 색상 판별
            if size_ratio >= self.size_ratio_th:
                # 검출된 TL 박스만 크롭
                tl_crop = img[y1b:y2b, x1b:x2b]
                hsv     = cv2.cvtColor(tl_crop, cv2.COLOR_BGR2HSV)

                # 빨강 마스크 (두 범위)
                m1 = cv2.inRange(hsv, (0,100,100),   (10,255,255))
                m2 = cv2.inRange(hsv, (160,100,100), (180,255,255))
                cnt_r = cv2.countNonZero(m1 | m2)

                # 노랑 마스크 (예: H 15~35)
                cnt_y = cv2.countNonZero(cv2.inRange(hsv, (15,100,100), (35,255,255)))

                # 초록 마스크 (예: H 40~80)
                cnt_g = cv2.countNonZero(cv2.inRange(hsv, (40,100,100), (80,255,255)))

                # F) 색상+고장 논리 순서:
                #   - 빨+노 → RED (고장 시 빨강 우선)
                #   - 노랑만 → YELLOW
                #   - 초록만 → GREEN
                #   - 그 외(색상 없거나 애매) → CAUTION (정지)
                if cnt_r > 0 and cnt_y > 0:
                    stop_flag = True
                    color_str = "RED"
                elif cnt_y > 0:
                    stop_flag = True  # 노랑일 때도 일단 정지
                    color_str = "YELLOW"
                elif cnt_g > 0:
                    stop_flag = False
                    color_str = "GREEN"
                else:
                    stop_flag = True
                    color_str = "CAUTION"
            else:
                # 멀리 있음 → 무시
                color_str = "FAR"

            # G) 디버그용 시각화 (원하는 경우)
            vis = img.copy()
            # ROI 구역 표시
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0,255,0), 2)
            # TL 바운딩박스 표시
            cv2.rectangle(vis, (x1b, y1b), (x2b, y2b), (0,0,255), 2)
            cv2.putText(vis,
                        f"{color_str} ratio={size_ratio:.2f}",
                        (x1b, y1b - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow('TL Vision', vis)
            cv2.waitKey(1)

        # H) 결과 퍼블리시
        self.stop_pub.publish(Bool(data=stop_flag))
        self.color_pub.publish(String(data=color_str))


if __name__ == '__main__':
    try:
        TrafficLightVision()
    except rospy.ROSInterruptException:
        pass
