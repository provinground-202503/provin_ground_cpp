#!/usr/bin/env python3
import cv2
import numpy as np
import glob

'''
objpoints → “체스보드 위에 있는 실제 3D 점들의 집합”

imgpoints → “그 점들이 영상에서는 어디 픽셀에 찍혔는지 대응”

calibrateCamera → 이 매칭 정보를 바탕으로

K (intrinsic): 카메라 내부 파라미터

rvecs, tvecs (extrinsic): 체스보드(월드) → 카메라 좌표계로의 회전·병진
'''

def calibrate_camera(
    image_folder: str,
    #image_folder = "/home/a/erp42_ws/src/provin_ground_cpp/script/cam_data_camera.jpg",
    pattern_size=(9, 6),
    square_size=0.024
):
    """
    Calibrate camera using chessboard images.

    Args:
        image_folder: Glob pattern for calibration images (e.g., './calib_images/*.jpg')
        pattern_size: Number of inner corners per chessboard (cols, rows)
        square_size: Size of one square edge in meters
    Returns:
        ret: RMS re-projection error
        K:   Intrinsic camera matrix
        dist: Distortion coefficients
    """

    # 1) 객체 좌표 준비 (0,0,0), (1,0,0), ..., (cols-1, rows-1, 0)
    # . objpoints에 정의하는 좌표들은 체스보드 기준(월드 프레임)에서의 3D 점들
    objp = np.zeros((pattern_size[1]*pattern_size[0], 3), np.float32)
    objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2) * square_size

    objpoints = []  # 3D 점들
    imgpoints = []  # 2D 점들

    # 2) 이미지 읽어 코너 검출
    images = glob.glob(image_folder)
    if not images:
        print(f"No images found: {image_folder}")
        return None, None, None

    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if not found:
            continue

        corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_refined)

        # (옵션) 검출 시 시각화
        cv2.drawChessboardCorners(img, pattern_size, corners_refined, found)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)

    cv2.destroyAllWindows()

    # 3) 카메라 보정
    img_size = gray.shape[::-1]
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )

    return ret, K, dist

if __name__ == "__main__":
    # 예시: pattern_size=(9,6), square_size=0.15m, 이미지 폴더 './calib_images/*.jpg'
    ret, K, dist = calibrate_camera(
        #'./calib_images/*.jpg',
        "/home/a/erp42_ws/src/provin_ground_cpp/script/cam_data_camera2.jpg",
        pattern_size=(3,4),
        square_size=0.15
    )
    if K is not None:
        print("RMS re-projection error:", ret)
        print("Intrinsic matrix K:\n", K)
        print("Distortion coefficients:\n", dist.ravel())
    else:
        print("Calibration failed. 이미지 경로와 패턴 크기를 확인하세요.")

'''
a@a:~/erp42_ws/src/provin_ground_cpp/script$ python3 kt_calculate_k.py 
RMS re-projection error: 0.1333276977677061
Intrinsic matrix K:
 [[1.22007391e+03 0.00000000e+00 6.39827466e+02]
 [0.00000000e+00 1.21938698e+03 3.59997102e+02]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
Distortion coefficients:
 [ 2.41562496e+00  9.11672524e+01  1.79662268e-01  1.41438618e-01
 -4.57391511e+03]
a@a:~/erp42_ws/src/provin_ground_cpp/script$ 

=> fₓ = K[0,0] ≃ 1220.1 px,

fᵧ = K[1,1] ≃ 1219.4 px,

cₓ = K[0,2] ≃ 639.8 px,

cᵧ = K[1,2] ≃ 360.0 px.
'''