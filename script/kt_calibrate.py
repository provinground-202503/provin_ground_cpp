# import cv2

# cap = cv2.VideoCapture(2)  # 위에서 확인한 인덱스로 맞춰 주세요
# if not cap.isOpened():
#     raise RuntimeError("웹캠 열기 실패")

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("프레임 읽기 실패")
#         break
#     cv2.imshow("RAW Frame", frame)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()

#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO

def main():
    # 1) 보정된 내부 파라미터 (거리 계산용)
    K = np.array([[779.3273504,   0.       , 684.55504224],
                  [  0.      , 847.4942847, 371.93940769],
                  [  0.      ,   0.      ,   1.        ]], dtype=np.float32)

    # 2) 실제 객체 높이 [m] (예: 배럴 높이 0.8m)
    OBJECT_REAL_HEIGHT = 0.8

    # 3) YOLOv8 모델 로드
    # model_path = "/home/a/erp42_ws/src/yolo_visualizer/models/best.pt"
    model_path = "/home/a/erp42_ws/src/yolo_visualizer/models/yolov8s.pt"
    model = YOLO(model_path)

    # 4) 웹캠 열기 (인덱스 2)
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        raise RuntimeError("웹캠을 열 수 없습니다 (index=2)")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            vis = frame.copy()

            # 5) YOLO 추론
            results = model(frame)[0]

            # 6) 거리 계산 & 시각화 (바운딩박스, 거리, 클래스명 표시)
            for box, cls_id in zip(results.boxes.xyxy.cpu().numpy().astype(int),
                                    results.boxes.cls.cpu().numpy().astype(int)):
                x1, y1, x2, y2 = box
                h_pix = float(y2 - y1)
                if h_pix <= 0:
                    continue

                # 거리 추정
                dist_m = (K[1,1] * OBJECT_REAL_HEIGHT) / h_pix
                tuned_dist_m = dist_m - 1.5 # <- 1.5를 뺸건 실험을 통해 임의로 뻈음,,_ 보정값, 

                # 클래스명 얻기
                class_name = model.names.get(int(cls_id), str(cls_id))

                # 바운딩박스
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 레이블: 클래스와 거리
                label = f"{class_name} {tuned_dist_m:.2f}m"
                cv2.putText(vis, label,
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)

            # 7) 결과 출력
            cv2.imshow("YOLO Distance & ID Visualization", vis)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

