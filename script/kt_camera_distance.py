# # 삼각측량을 이용

# '''
#  OpenCV로 YOLO 등으로 받은 바운딩박스에 대해, 
#  카메라 내부 행렬과 물체의 실제 높이를 이용해 카메라–물체 간 거리를 계산하고,
#   프레임 위에 바운딩박스와 거리를 시각화하는 파이썬 코드
# '''


# 조건문 1,2 중 하나를 선택하면 됨_ 둘다 토픽 발행 조건에 해당해서 큰 차이는 없음

# <1> 계산된 거리값이 1.5 보다 작으면 토픽을 발행
# <2> 바운딩박스의 높이가 프레임 높이 대비 30프로 이상이면 토픽을 밮행

# <1> 계산된 거리값이 1.5 보다 작으면 토픽을 발행
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import cv2
import numpy as np
from ultralytics import YOLO

def main():
    # 1) ROS 노드 및 퍼블리셔 초기화
    rospy.init_node('yolo_distance_publisher', anonymous=True)
    obj_dist_pub = rospy.Publisher('/obj_distance', Float32, queue_size=10)

    # 2) 보정된 내부 파라미터 (거리 계산용)
    K = np.array([[779.3273504,   0.       , 684.55504224],
                  [  0.      , 847.4942847, 371.93940769],
                  [  0.      ,   0.      ,   1.        ]], dtype=np.float32)

    # 3) 실제 객체 높이 [m] (예: 배럴 높이 0.8m)
    OBJECT_REAL_HEIGHT = 0.8

    # 4) YOLOv8 모델 로드
    
    # model_path = "/home/a/erp42_ws/src/yolo_visualizer/models/yolov8s.pt"
    model_path = "/root/erp42_ws/src/perception_examples/yolo/models/yolov8n_0707.pt"
    model = YOLO(model_path)

    # 5) 웹캠 열기 (인덱스 2)
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        rospy.logerr("웹캠을 열 수 없습니다 (index=2)")
        return

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("프레임 읽기 실패")
                break

            vis = frame.copy()
            frame_h = frame.shape[0]

            # 6) YOLO 추론
            results = model(frame)[0]

            # 7) 거리 계산 & 시각화
            for box, cls_id in zip(results.boxes.xyxy.cpu().numpy().astype(int),
                                    results.boxes.cls.cpu().numpy().astype(int)):
                x1, y1, x2, y2 = box
                h_pix = float(y2 - y1)
                if h_pix <= 0:
                    continue

                # 전체 프레임 대비 bbox 높이 비율
                ratio = h_pix / float(frame_h)
                if ratio < 0.3:
                    # 30% 미만 → 녹색 박스만 시각화
                    label = f"{model.names[int(cls_id)]} ({ratio*100:.1f}%)"
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(vis, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    continue

                # 거리 추정
                dist_m = (K[1,1] * OBJECT_REAL_HEIGHT) / h_pix
                tuned_dist_m = dist_m - 1.5  # 보정값

                # 수정: 1.5m 미만일 때만 퍼블리시
                if tuned_dist_m < 1.5:
                    obj_dist_pub.publish(tuned_dist_m)
                    rospy.logdebug(
                        f"Published obj distance: {tuned_dist_m:.2f} m (ratio {ratio:.2f})"
                    )
                    # 빨간 박스 + 거리 레이블
                    class_name = model.names.get(int(cls_id), str(cls_id))
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    label = f"{class_name} {tuned_dist_m:.2f}m"
                    cv2.putText(vis, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    # 1.5m 이상 → 노란 박스 시각화만
                    class_name = model.names.get(int(cls_id), str(cls_id))
                    label = f"{class_name} {tuned_dist_m:.2f}m"
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(vis, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 8) 결과 출력
            cv2.imshow("YOLO Distance & ID Visualization", vis)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


# <2> 바운딩박스의 높이가 프레임 높이 대비 30프로 이상이면 토픽을 밮행
#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Float32
# import cv2
# import numpy as np
# from ultralytics import YOLO

# def main():
#     # 1) ROS 노드 및 퍼블리셔 초기화
#     rospy.init_node('yolo_distance_publisher', anonymous=True)
#     obj_dist_pub = rospy.Publisher('/obj_distance', Float32, queue_size=10)

#     # 2) 보정된 내부 파라미터 (거리 계산용)
#     K = np.array([[779.3273504,   0.       , 684.55504224],
#                   [  0.      , 847.4942847, 371.93940769],
#                   [  0.      ,   0.      ,   1.        ]], dtype=np.float32)

#     # 3) 실제 객체 높이 [m] (예: 배럴 높이 0.8m)
#     OBJECT_REAL_HEIGHT = 0.8

#     # 4) YOLOv8 모델 로드
#     model_path = "/home/a/erp42_ws/src/yolo_visualizer/models/yolov8s.pt"
#     model = YOLO(model_path)

#     # 5) 웹캠 열기 (인덱스 2)
#     cap = cv2.VideoCapture(2)
#     if not cap.isOpened():
#         rospy.logerr("웹캠을 열 수 없습니다 (index=2)")
#         return

#     try:
#         while not rospy.is_shutdown():
#             ret, frame = cap.read()
#             if not ret:
#                 rospy.logwarn("프레임 읽기 실패")
#                 break

#             vis = frame.copy()
#             frame_h = frame.shape[0]

#             # 6) YOLO 추론
#             results = model(frame)[0]

#             # 7) 거리 계산 & 시각화 (바운딩박스, 거리, 클래스명 표시)
#             for box, cls_id in zip(results.boxes.xyxy.cpu().numpy().astype(int),
#                                     results.boxes.cls.cpu().numpy().astype(int)):
#                 x1, y1, x2, y2 = box
#                 h_pix = float(y2 - y1)
#                 if h_pix <= 0:
#                     continue

#                 # 전체 프레임 대비 높이 비율 계산
#                 ratio = h_pix / float(frame_h)
#                 if ratio < 0.3:
#                     # 30% 미만은 퍼블리시하지 않고 시각화만
#                     label = f"{model.names[int(cls_id)]} ({ratio*100:.1f}%)"
#                     cv2.rectangle(vis, (x1, y1), (x2, y2), (0,255,0), 2)
#                     cv2.putText(vis, label, (x1, y1 - 10),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
#                     continue

#                 # 거리 추정
#                 dist_m = (K[1,1] * OBJECT_REAL_HEIGHT) / h_pix
#                 tuned_dist_m = dist_m - 1.5  # 보정값

#                 # 토픽 퍼블리시
#                 obj_dist_pub.publish(tuned_dist_m)
#                 rospy.logdebug(f"Published obj distance: {tuned_dist_m:.2f} m (ratio {ratio:.2f})")

#                 # 바운딩박스 + 레이블 시각화
#                 class_name = model.names.get(int(cls_id), str(cls_id))
#                 cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 30% 이상은 빨간색
#                 label = f"{class_name} {tuned_dist_m:.2f}m"
#                 cv2.putText(vis, label,
#                             (x1, y1 - 10),
#                             cv2.FONT_HERSHEY_SIMPLEX,
#                             0.6, (0, 0, 255), 2)

#             # 8) 결과 출력
#             cv2.imshow("YOLO Distance & ID Visualization", vis)
#             if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
#                 break

#     finally:
#         cap.release()
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()





# ----------------------------------------
# 카메라 테스트

# import cv2
# import numpy as np

# # 1) 카메라 내부 행렬 (K) 정의
# #    fx, fy: 초점거리 [pixel]
# #    cx, cy: 주점(principal point) [pixel]
# K = np.array([[fx,  0, cx],
#               [ 0, fy, cy],
#               [ 0,  0,  1]], dtype=np.float32)

# # 2) 물체 실제 높이 (H) [m]
# OBJECT_REAL_HEIGHT = 1.7  # 예: 성인 사람 키 1.7m

# # 3) 동영상 또는 카메라 캡처 열기
# cap = cv2.VideoCapture(0)  # 0번 카메라

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     # --- (여기서 YOLO 등으로 bbox 검출) ---
#     # detections = [ (x, y, w, h), ... ]
#     # 예시용으로 임의 bbox 하나 사용:
#     # detections = [(100, 150, 80, 200)]  # x,y,width,height

#     for (x, y, w, h) in detections:
#         # 4) 픽셀 단위로 바운딩박스 높이 h_pix 구하기
#         h_pix = float(h)
#         if h_pix <= 0:
#             continue

#         # 5) 단일 축(fy)을 이용한 거리 계산
#         #    d = (fy * H) / h_pix
#         dist_m = (K[1,1] * OBJECT_REAL_HEIGHT) / h_pix

#         # 6) 시각화
#         cv2.rectangle(frame, (x, y), (x + w, y + h), (0,255,0), 2)
#         label = f"{dist_m:.2f} m"
#         cv2.putText(frame, label, (x, y - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

#     cv2.imshow("Distance Estimation", frame)
#     if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
#         break

# cap.release()
# cv2.destroyAllWindows()

