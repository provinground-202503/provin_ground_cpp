#!/usr/bin/env python3
import cv2
import numpy as np

def compute_homography_from_checkerboard(corners, pattern_size=(9,6), square_size=1.0):
    # objp: (Nx3) world 좌표
    objp = np.zeros((pattern_size[1]*pattern_size[0], 3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
    # corners: (Nx1x2) → (Nx2)
    src_pts = objp[:,:2]
    dst_pts = corners.reshape(-1,2)
    H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC)
    return H

if __name__ == "__main__":
    cap = cv2.VideoCapture(3) #3번
    if not cap.isOpened():
        raise RuntimeError("카메라 열기 실패")

    # 4,5
    pattern_size = (3,4)
    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
    H = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        vis = frame.copy()
        if found:
            # 코너 정밀화
            corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            # 코너 그리기
            cv2.drawChessboardCorners(vis, pattern_size, corners_refined, found)

            # 최초 발견 시에만 H 계산
            if H is None:
                try:
                    H = compute_homography_from_checkerboard(corners_refined, pattern_size, square_size=1.0)
                    print("Homography computed:\n", H)
                except Exception as e:
                    print("H 계산 오류:", e)

        # Bird’s-eye view 생성 (H 준비되면)
        if H is not None:
            h, w = frame.shape[:2]
            bird = cv2.warpPerspective(frame, H, (w, h))
        else:
            bird = np.zeros_like(frame)
            cv2.putText(bird, "No H yet", (50,50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        # 화면 합치기 & 출력
        h, w = frame.shape[:2]
        left = cv2.resize(vis, (w//2, h//2))
        right = cv2.resize(bird, (w//2, h//2))
        combo = cv2.hconcat([left, right])
        cv2.imshow("Camera View (with corners) | Bird's-eye View", combo)

        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

    cap.release()
    cv2.destroyAllWindows()