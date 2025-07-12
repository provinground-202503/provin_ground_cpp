#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from ultralytics import YOLO

def draw_boxes(img, results):
    """
    results: ultralytics Results 객체
    """
    for r in results:
        boxes = r.boxes.xyxy.cpu().numpy()      # [N,4] (x1,y1,x2,y2)
        scores = r.boxes.conf.cpu().numpy()     # [N]
        classes = r.boxes.cls.cpu().numpy().astype(int)  # [N]
        names = r.names  # dict: 클래스 인덱스->이름

        for box, score, cls in zip(boxes, scores, classes):
            x1,y1,x2,y2 = map(int, box)
            label = f"{names[cls]} {score:.2f}"
            # draw rectangle
            cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
            # draw label background
            (w,h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img, (x1, y1-h-4), (x1+w, y1), (0,255,0), -1)
            # put text
            cv2.putText(img, label, (x1, y1-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)

if __name__ == "__main__":
    # 1) 모델 로드
    #model_path = "/home/a/erp42_ws/src/yolo_visualizer/models/best.pt"
    model_path = "/home/a/erp42_ws/src/yolo_visualizer/models/kt_best.pt"
    model = YOLO(model_path)
    #MODEL 이 학습한 클래스명 확인 가능,
    print(model.names)
    #{0: 'barrel', 1: 'person', 2: 'greenlight', 3: 'redlight', 4: 'yellowlight'}



    # 2) 이미지 읽기
    img_path = "/home/a/erp42_ws/src/provin_ground_cpp/script/barrel_img.jpg"
    img = cv2.imread(img_path)
    if img is None:
        print(f"Error: cannot read image at {img_path}")
        exit(1)

    # 3) 추론
    results = model(img)  # ultralytics YOLOv8 Inference 결과

    # 4) 바운딩 박스 및 라벨 그리기
    draw_boxes(img, results)

    # 5) 결과 출력
    cv2.namedWindow("YOLO Detections", cv2.WINDOW_NORMAL)
    cv2.imshow("YOLO Detections", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
