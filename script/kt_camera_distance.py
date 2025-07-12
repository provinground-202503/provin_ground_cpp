#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import cv2
import numpy as np
from ultralytics import YOLO


'''
초록색(0,255,0):
– 바운딩박스 높이 비율(ratio) < 30% 인, 즉 “아직 멀리 있는” barrel/person 객체

빨간색(0,0,255):
– ratio ≥ 30% 이고 보정된 거리(tuned_dist_m) < 1.5m 인 “즉시 회피가 필요한” 매우 가까운 객체
– 이 때 /obj_distance 토픽을 퍼블리시

노란색(0,255,255):
– ratio ≥ 30% 이지만 tuned_dist_m ≥ 1.5m 인 “충분히 가까워지긴 했으나 아직 회피 기준(1.5m) 이내는 아닌” 중간 상태

흰색 텍스트:
– 프레임 상단에 “검출 여부(Detected: Yes/No)”와 “최대 높이 비율(Max Height Ratio)”을 표시
'''

def main():
    rospy.init_node('yolo_distance_publisher', anonymous=True)
    obj_dist_pub = rospy.Publisher('/obj_distance', Float32, queue_size=10)

    K = np.array([[779.3273504,   0.       , 684.55504224],
                  [  0.      , 847.4942847, 371.93940769],
                  [  0.      ,   0.      ,   1.        ]], dtype=np.float32)
    OBJECT_REAL_HEIGHT = 0.8
    model = YOLO("/home/a/erp42_ws/src/yolo_visualizer/models/yolov8s.pt")

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

            # YOLO 추론
            results = model(frame)[0]

            # 프레임 단위 상태
            detected_any = False
            max_ratio = 0.0

            # 거리 계산 & 시각화
            for box, cls_id in zip(results.boxes.xyxy.cpu().numpy().astype(int),
                                    results.boxes.cls.cpu().numpy().astype(int)):
                # 클래스 필터링: barrel(0)과 person(1)만 처리
                if int(cls_id) not in (0, 1):
                    continue

                x1, y1, x2, y2 = box
                h_pix = float(y2 - y1)
                if h_pix <= 0:
                    continue

                ratio = h_pix / float(frame_h)
                max_ratio = max(max_ratio, ratio)
                detected_any = True

                if ratio < 0.3:
                    label = f"{model.names[int(cls_id)]} ({ratio*100:.1f}%)"
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(vis, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    continue

                dist_m = (K[1,1] * OBJECT_REAL_HEIGHT) / h_pix
                tuned_dist_m = dist_m - 1.5

                if tuned_dist_m < 1.5:
                    obj_dist_pub.publish(tuned_dist_m)
                    rospy.logdebug(f"Published obj distance: {tuned_dist_m:.2f} m (ratio {ratio:.2f})")
                    color = (0,0,255)
                    label = f"{model.names[int(cls_id)]} {tuned_dist_m:.2f}m"
                else:
                    color = (0,255,255)
                    label = f"{model.names[int(cls_id)]} {tuned_dist_m:.2f}m"

                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                cv2.putText(vis, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 전체 프레임 상단에 상태 텍스트 추가
            status = "Yes" if detected_any else "No"
            status_text = f"Detected: {status}, Max Height Ratio: {max_ratio*100:.1f}%"
            cv2.putText(vis, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            cv2.imshow("YOLO Distance & ID Visualization", vis)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
