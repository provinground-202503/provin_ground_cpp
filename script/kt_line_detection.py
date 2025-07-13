#!/usr/bin/env python3
import cv2
import numpy as np

# L,R 의 값을 vector로 토픽을 날린다.
from geometry_msgs.msg import Vector3 

# 1) ROI 사다리꼴 비율 정의 (top-left, top-right, bottom-right, bottom-left)
ROI_RATIO = [
    (0.2, 0.85),   # top-left
    (0.8, 0.85),   # top-right
    (1.0, 1.0),    # bottom-right
    (0.0, 1.0),    # bottom-left
]

def get_roi_pts(img, pts_ratio):
    h, w = img.shape[:2]
    return np.float32([ (x * w, y * h) for x, y in pts_ratio ])

def main():

    # ROS추가!
    # /lane_points
    rospy.init_node('lane_point_publisher', anonymous=True)       # 노드 이름 설정
    pub = rospy.Publisher('lane_points', Vector3, queue_size=10)  # 토픽 이름: /lane_points

    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("카메라 열기 실패")
        return

    # 첫 프레임 읽어 크기 확인
    ret, frame = cap.read()
    if not ret:
        print("첫 프레임 읽기 실패")
        return
    h, w = frame.shape[:2]
    dst_size = (w, h)

    # Bird’s-eye View의 4개 모서리(직사각형) 정의
    dst_pts = np.float32([
        (0,   0),    # top-left
        (w,   0),    # top-right
        (w,   h),    # bottom-right
        (0,   h),    # bottom-left
    ])

    # ROI의 4개 픽셀 좌표 계산 및 Homography 계산
    src_pts = get_roi_pts(frame, ROI_RATIO)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # 디버그: src_pts, M 값 출력
    print("src_pts:", np.round(src_pts, 1))
    print("dst_pts:", dst_pts)
    print("Homography M:\n", np.round(M, 3))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # (1) ROI 시각화
        vis = frame.copy()
        cv2.polylines(vis,
                      [src_pts.reshape(-1,1,2).astype(np.int32)],
                      isClosed=True,
                      color=(0,255,0),
                      thickness=2)
        cv2.imshow("ROI View", vis)

        # (2) Bird’s-eye View 변환
        bird = cv2.warpPerspective(frame, M, dst_size, flags=cv2.INTER_LINEAR)
        cv2.imshow("Bird's-eye View", bird)

        # (3) Canny Edge 검출
        gray = cv2.cvtColor(bird, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        cv2.imshow("Canny Edges", edges)

        # (4) 모폴로지 연산 (Closing → Opening)
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 5))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel_close, iterations=1)
        kernel_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        processed    = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_open, iterations=1)
        cv2.imshow("Morphology", processed)

        # (5) Blob 검출 (area < 2000)
        noise_pts = []
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(processed)
        for i in range(1, n_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 2000:
                cx, cy = centroids[i]
                noise_pts.append((cx, cy))

        # (6) 컨투어 검출 및 중점 계산
        contour_mid_pts = []
        contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            xs = approx[:,:,0].flatten()
            ys = approx[:,:,1].flatten()
            if xs.size == 0:
                continue
            idx_min = ys.argmin()
            idx_max = ys.argmax()
            x_min, y_min = xs[idx_min], ys[idx_min]
            x_max, y_max = xs[idx_max], ys[idx_max]
            mx = (x_min + x_max) / 2
            my = (y_min + y_max) / 2
            contour_mid_pts.append((mx, my))

        # (7) 노이즈 매칭·제거 (가장 가까운 한 쌍)
        pairs = []
        for bi, (bx, by) in enumerate(noise_pts):
            for ci, (cx, cy) in enumerate(contour_mid_pts):
                dist = np.hypot(bx - cx, by - cy)
                pairs.append((dist, bi, ci))
        if pairs:
            pairs.sort(key=lambda x: x[0])
            _, bi, ci = pairs[0]
            noise_pts.pop(bi)
            contour_mid_pts.pop(ci)

        # (8) 왼쪽/오른쪽 차선만 추출
        left_pts  = [(x,y) for x,y in contour_mid_pts if x < w*0.5]
        right_pts = [(x,y) for x,y in contour_mid_pts if x > w*0.5]

        # (9) 왼쪽 차선 중심 계산 (빨간점)
        x_center_left = None
        if left_pts:
            x_min_left  = min(x for x,_ in left_pts)
            x_max_left  = max(x for x,_ in left_pts)
            x_center_left = (x_min_left + x_max_left) / 2

        # (10) 오른쪽 차선 중심 계산 (파란점)
        x_center_right = None
        if right_pts:
            x_min_right   = min(x for x,_ in right_pts)
            x_max_right   = max(x for x,_ in right_pts)
            x_center_right = (x_min_right + x_max_right) / 2
        # ros left, right point publish
        msg = Vector3()
        if x_center_left is not None:
            msg.x = float(x_center_left)
        else:
            msg.x = float('nan')
        if x_center_right is not None:
            msg.y = float(x_center_right)
        else:
            msg.y = float('nan')
        msg.z = 0.0   # 필요에 따라 쓰지 않을 필드는 0으로
        pub.publish(msg)
        
        # (11) 시각화
        y_vis = int(h * 0.9)
        lane_vis = bird.copy()
        # 왼쪽 차선 중심 (빨간)
        if x_center_left is not None:
            cv2.circle(lane_vis, (int(x_center_left), y_vis), 5, (0,0,255), -1)
        # 오른쪽 차선 중심 (파란)
        if x_center_right is not None:
            cv2.circle(lane_vis, (int(x_center_right), y_vis), 5, (255,0,0), -1)
        # 프레임 중앙 (초록) — 좌표 확인용
        cv2.circle(lane_vis, (int(w*0.5), y_vis), 5, (0,255,0), -1)

        cv2.imshow("Lane Detection", lane_vis)

        # 종료 키
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()