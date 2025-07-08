#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import struct

class GroundRemover:
    def __init__(self):
        rospy.init_node('ground_remover', anonymous=True)

        # /ouster_scan 토픽 구독
        self.subscriber = rospy.Subscriber("/ouster_scan", PointCloud2, self.point_cloud_callback)

        # 지면이 제거된 PointCloud2 메시지를 발행할 퍼블리셔
        self.publisher = rospy.Publisher("/ouster_scan/ground_removed", PointCloud2, queue_size=1)

        rospy.loginfo("Ground Remover Node Initialized.")
        rospy.loginfo("Subscribing to /ouster_scan")
        rospy.loginfo("Publishing to /ouster_scan/ground_removed")

        # RANSAC 파라미터 (조정 필요)
        self.distance_threshold = 0.05  # 평면과의 최대 거리 (미터)
        self.ransac_n = 3             # 평면 모델을 추정하는 데 필요한 최소 포인트 수
        self.num_iterations = 1000    # RANSAC 반복 횟수

    def point_cloud_callback(self, ros_point_cloud):
        # PointCloud2 메시지를 NumPy 배열로 변환
        # sensor_msgs.point_cloud2.read_points 함수는 generator를 반환하므로 리스트로 변환
        points_list = []
        for p in pc2.read_points(ros_point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([p[0], p[1], p[2], p[3]])

        if not points_list:
            rospy.logwarn("Received empty point cloud, skipping processing.")
            return

        points_np = np.array(points_list, dtype=np.float32)

        # Open3D PointCloud 객체 생성
        # Open3D는 (N, 3) 형태의 NumPy 배열을 기대하므로 x, y, z만 전달
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np[:, :3])

        # RANSAC을 이용한 평면 세그멘테이션 (지면 감지)
        # segment_plane 함수는 (model_coefficients, inliers)를 반환
        # model_coefficients: [a, b, c, d] for ax + by + cz + d = 0
        # inliers: 평면에 속하는 포인트들의 인덱스
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.num_iterations
        )

        # Inliers (지면)와 Outliers (지면 제외) 분리
        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        # 지면이 제거된 포인트들을 NumPy 배열로 다시 변환
        # Open3D에서 NumPy 배열로 가져올 때 복사본이 생성됨
        non_ground_points_xyz = np.asarray(outlier_cloud.points)
        
        # 원래 intensity 정보도 유지하려면, inliers를 이용해 원래 points_np에서 필터링해야 함
        # intensity를 포함한 전체 데이터에서 inliers를 제외
        all_indices = np.arange(len(points_np))
        non_ground_indices = np.setdiff1d(all_indices, inliers)
        non_ground_points_with_intensity = points_np[non_ground_indices]

        # 지면이 제거된 포인트가 없는 경우 처리
        if len(non_ground_points_with_intensity) == 0:
            rospy.logwarn("No non-ground points remaining after RANSAC, publishing empty cloud.")
            # 빈 PointCloud2 메시지 발행
            header = ros_point_cloud.header
            header.stamp = rospy.Time.now()
            empty_pc2_msg = pc2.create_cloud(header, ros_point_cloud.fields, [])
            self.publisher.publish(empty_pc2_msg)
            return

        # 새 PointCloud2 메시지 생성
        # Open3D는 intensity 정보를 직접 다루지 않으므로, 원래 데이터에서 intensity를 가져와야 함.
        # 따라서 `pc2.create_cloud`를 사용하기 위해 flat_points 형태로 변환.
        
        # PointCloud2 메시지 필드 정의 (원본과 동일하게)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]
        
        # 새로운 PointCloud2 메시지 생성
        header = ros_point_cloud.header
        header.stamp = rospy.Time.now() # 타임스탬프 업데이트

        # points_np에서 inliers에 해당하지 않는 점들만 선택하여 새로운 PointCloud2 생성
        # pc2.create_cloud_xyzrgb (or similar) cannot handle intensity directly.
        # We need to manually construct the points for pc2.create_cloud.
        
        # 각 점의 데이터를 바이트로 패킹
        packed_points = []
        for p in non_ground_points_with_intensity:
            x, y, z, intensity = p[0], p[1], p[2], p[3]
            s = struct.pack('<ffff', x, y, z, intensity) # little-endian float for 4 values
            packed_points.append(s)

        # packed_points를 bytes 객체로 결합
        byte_data = b''.join(packed_points)

        # PointCloud2 메시지 생성
        # width: 전체 포인트 수 (1D 배열)
        # height: 1 (unorganized point cloud)
        non_ground_pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(non_ground_points_with_intensity),
            is_dense=True, # No NaNs
            is_bigendian=False, # Little endian (common)
            fields=fields,
            point_step=16, # 4 floats * 4 bytes/float = 16 bytes per point
            row_step=16 * len(non_ground_points_with_intensity), # point_step * width
            data=byte_data
        )

        self.publisher.publish(non_ground_pc2_msg)
        # rospy.loginfo(f"Published ground-removed cloud with {len(non_ground_points_with_intensity)} points.")

if __name__ == '__main__':
    try:
        ground_remover = GroundRemover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass