#!/usr/bin/env python3
"""
6-DOF Quad-Teat Exact Tip & Approach Vector Perception Node.

Subscribes to PointCloud2 from an RGB-D camera observing an automated cow milking rig.
Isolates the 4 anatomical teats (FL, FR, RL, RR) using pass-through filtering and quadrant clustering.
Applies Principal Component Analysis (PCA) on each teat cluster to:
  1. Determine the exact 3D cylinder orientation vector (insertion vector for cup attachment).
  2. Pinpoint the EXACT 3D distal tip apex (streak canal / cup entry orifice).
  3. Compute the 6-DOF orientation quaternion.
  4. Calculate the teat tilt angle from the vertical.

Publishes:
  - /teat_poses (geometry_msgs/PoseArray with 4 6-DOF tip poses)
  - /teat_fl/pose, /teat_fr/pose, /teat_rl/pose, /teat_rr/pose (PoseStamped)
  - /teat_fl/centroid, /teat_fr/centroid, /teat_rl/centroid, /teat_rr/centroid (PointStamped)
  - /teat_centroid (PointStamped - primary teat)
  - /teat_markers (visualization_msgs/MarkerArray for RViz2 tip spheres & 3D approach arrows)
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# SciPy Rotation with pure-numpy fallback
try:
    from scipy.spatial.transform import Rotation
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 orthonormal rotation matrix to a unit quaternion [x, y, z, w]."""
    if SCIPY_AVAILABLE:
        return Rotation.from_matrix(R).as_quat()

    # Fallback Shepperd's algorithm
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qx, qy, qz, qw], dtype=float)
    return q / np.linalg.norm(q)


class QuadTeat6DOFEstimator(Node):
    """ROS 2 Node for 6-DOF teat pose & approach vector estimation using PCA."""

    def __init__(self):
        super().__init__('teat_detector')

        # Spatial bounding box parameters (in camera frame coordinates)
        self.declare_parameter('x_min', 0.35)
        self.declare_parameter('x_max', 0.65)
        self.declare_parameter('x_mid', 0.49)  # Split front vs rear pairs
        self.declare_parameter('y_min', -0.15)
        self.declare_parameter('y_max', 0.15)
        self.declare_parameter('z_min', -0.18)
        self.declare_parameter('z_max', -0.02)
        self.declare_parameter('min_cluster_points', 35)

        # Subscriber for PointCloud2
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        # 6-DOF PoseArray publisher
        self.poses_pub = self.create_publisher(PoseArray, '/teat_poses', 10)

        # Individual 6-DOF PoseStamped publishers
        self.pub_pose_fl = self.create_publisher(PoseStamped, '/teat_fl/pose', 10)
        self.pub_pose_fr = self.create_publisher(PoseStamped, '/teat_fr/pose', 10)
        self.pub_pose_rl = self.create_publisher(PoseStamped, '/teat_rl/pose', 10)
        self.pub_pose_rr = self.create_publisher(PoseStamped, '/teat_rr/pose', 10)

        # Exact tip / Centroid publishers
        self.pub_fl = self.create_publisher(PointStamped, '/teat_fl/centroid', 10)
        self.pub_fr = self.create_publisher(PointStamped, '/teat_fr/centroid', 10)
        self.pub_rl = self.create_publisher(PointStamped, '/teat_rl/centroid', 10)
        self.pub_rr = self.create_publisher(PointStamped, '/teat_rr/centroid', 10)
        self.pub_primary = self.create_publisher(PointStamped, '/teat_centroid', 10)

        # RViz 3D MarkerArray publisher (Spheres, Insertion Arrows, Text Tags)
        self.marker_pub = self.create_publisher(MarkerArray, '/teat_markers', 10)

        self.get_logger().info('QuadTeat6DOFEstimator initialized. Tracking exact tips on /camera/depth/points...')

    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming PointCloud2, locate exact teat tips, and publish 6-DOF poses."""
        # 1. Read (x, y, z) fields using sensor_msgs_py
        try:
            points_structured = pc2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=False
            )
        except Exception as e:
            self.get_logger().error(f'Failed to parse PointCloud2: {str(e)}')
            return

        xyz = np.stack(
            [points_structured['x'], points_structured['y'], points_structured['z']],
            axis=-1
        )
        valid = xyz[np.isfinite(xyz).all(axis=1)]
        if len(valid) == 0:
            return

        # 2. Retrieve parameters
        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        x_mid = self.get_parameter('x_mid').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        min_pts = self.get_parameter('min_cluster_points').value

        # 3. Spatial pass-through filter isolating the teat layer
        teat_layer_mask = (
            (valid[:, 0] >= x_min) & (valid[:, 0] <= x_max) &
            (valid[:, 1] >= y_min) & (valid[:, 1] <= y_max) &
            (valid[:, 2] >= z_min) & (valid[:, 2] <= z_max)
        )
        teat_points = valid[teat_layer_mask]

        if len(teat_points) < min_pts:
            return

        # 4. Partition into 4 anatomical quadrants
        clusters = {
            'FL': teat_points[(teat_points[:, 0] < x_mid) & (teat_points[:, 1] > 0.01)],
            'FR': teat_points[(teat_points[:, 0] < x_mid) & (teat_points[:, 1] < -0.01)],
            'RL': teat_points[(teat_points[:, 0] >= x_mid) & (teat_points[:, 1] > 0.01)],
            'RR': teat_points[(teat_points[:, 0] >= x_mid) & (teat_points[:, 1] < -0.01)],
        }

        frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link'
        stamp = msg.header.stamp

        teat_results = {}
        for name, pts in clusters.items():
            if len(pts) < min_pts:
                teat_results[name] = None
                continue

            # ========================================================
            # 5. Principal Component Analysis (PCA)
            # ========================================================
            centroid = np.mean(pts, axis=0)
            centered = pts - centroid

            cov = np.cov(centered, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            # Sort descending by eigenvalue
            idx = np.argsort(eigenvalues)[::-1]
            eigenvalues = eigenvalues[idx]
            eigenvectors = eigenvectors[:, idx]

            # Primary eigenvector v1 represents cylinder axis
            v1 = eigenvectors[:, 0]

            # Ensure v1 points downward towards the teat tip (negative Z)
            if v1[2] > 0.0:
                v1 = -v1

            # ========================================================
            # 6. EXACT TEAT TIP APEX EXTRACTION
            # Project all cluster points onto the primary axis v1.
            # Points on the distal dome apex have maximum projection.
            # We take the centroid of the distal-most points to pinpoint
            # the exact teat tip apex on the real point cloud surface.
            # ========================================================
            projections = centered @ v1
            s_max = np.max(projections)
            # Select points within the top 3mm of the apex (or 99th percentile)
            s_cutoff = max(np.percentile(projections, 99.0), s_max - 0.003)
            apex_mask = projections >= s_cutoff
            exact_tip = np.mean(pts[apex_mask], axis=0)

            # Build orthonormal right-handed basis (Z-axis = insertion vector)
            z_axis = v1 / np.linalg.norm(v1)
            ref = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            y_axis = np.cross(z_axis, ref)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)

            R_mat = np.column_stack([x_axis, y_axis, z_axis])
            quat = rotation_matrix_to_quaternion(R_mat)

            # Tilt angle from vertical downward axis [0, 0, -1]
            cos_tilt = np.clip(np.dot(z_axis, np.array([0.0, 0.0, -1.0])), -1.0, 1.0)
            tilt_deg = np.degrees(np.arccos(cos_tilt))

            teat_results[name] = {
                'centroid': centroid,
                'tip': exact_tip,
                'axis': z_axis,
                'quat': quat,
                'tilt_deg': tilt_deg,
                'points_count': len(pts),
            }

        # 7. Publish Exact Tip Coordinates (PointStamped)
        centroid_pubs = {
            'FL': self.pub_fl,
            'FR': self.pub_fr,
            'RL': self.pub_rl,
            'RR': self.pub_rr
        }
        for name, pub in centroid_pubs.items():
            res = teat_results.get(name)
            if res is not None:
                pt_msg = PointStamped()
                pt_msg.header.stamp = stamp
                pt_msg.header.frame_id = frame_id
                pt_msg.point.x = float(res['tip'][0])
                pt_msg.point.y = float(res['tip'][1])
                pt_msg.point.z = float(res['tip'][2])
                pub.publish(pt_msg)

        # 8. Publish Individual 6-DOF Poses (PoseStamped)
        pose_pubs = {
            'FL': self.pub_pose_fl,
            'FR': self.pub_pose_fr,
            'RL': self.pub_pose_rl,
            'RR': self.pub_pose_rr
        }
        for name, pub in pose_pubs.items():
            res = teat_results.get(name)
            if res is not None:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = frame_id
                pose_msg.pose.position.x = float(res['tip'][0])
                pose_msg.pose.position.y = float(res['tip'][1])
                pose_msg.pose.position.z = float(res['tip'][2])
                pose_msg.pose.orientation.x = float(res['quat'][0])
                pose_msg.pose.orientation.y = float(res['quat'][1])
                pose_msg.pose.orientation.z = float(res['quat'][2])
                pose_msg.pose.orientation.w = float(res['quat'][3])
                pub.publish(pose_msg)

        # 9. Publish 6-DOF PoseArray
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame_id

        for name in ['FL', 'FR', 'RL', 'RR']:
            res = teat_results.get(name)
            if res is not None:
                p = Pose()
                p.position.x = float(res['tip'][0])
                p.position.y = float(res['tip'][1])
                p.position.z = float(res['tip'][2])
                p.orientation.x = float(res['quat'][0])
                p.orientation.y = float(res['quat'][1])
                p.orientation.z = float(res['quat'][2])
                p.orientation.w = float(res['quat'][3])
                pose_array.poses.append(p)

        self.poses_pub.publish(pose_array)

        # 10. Backward-Compatible /teat_centroid
        if teat_results.get('FL') is not None:
            prim_c = teat_results['FL']['tip']
            prim_msg = PointStamped()
            prim_msg.header.stamp = stamp
            prim_msg.header.frame_id = frame_id
            prim_msg.point.x = float(prim_c[0])
            prim_msg.point.y = float(prim_c[1])
            prim_msg.point.z = float(prim_c[2])
            self.pub_primary.publish(prim_msg)

        # 11. Publish RViz 3D Markers (Tips, Insertion Arrows, Labels)
        self.publish_rviz_markers(teat_results, frame_id, stamp)

        # 12. Throttled Summary Logging (1 Hz)
        log_parts = []
        for k in ['FL', 'FR', 'RL', 'RR']:
            res = teat_results.get(k)
            if res is not None:
                t = res['tip']
                log_parts.append(
                    f"{k} Tip:[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] Tilt:{res['tilt_deg']:.1f}°"
                )
            else:
                log_parts.append(f"{k}:[LOST]")

        self.get_logger().info(
            f"Exact Teat Tips -> {' | '.join(log_parts)}",
            throttle_duration_sec=1.0
        )

    def publish_rviz_markers(self, teat_results, frame_id, stamp):
        """Publish 3D spherical tip markers, approach vector arrows, and text tags for RViz2."""
        marker_array = MarkerArray()

        color_map = {
            'FL': (0.1, 0.9, 0.1),  # Green
            'FR': (0.1, 0.8, 0.9),  # Cyan
            'RL': (0.9, 0.8, 0.1),  # Yellow
            'RR': (0.9, 0.2, 0.2),  # Red
        }

        m_id = 0
        for name in ['FL', 'FR', 'RL', 'RR']:
            res = teat_results.get(name)
            if res is None:
                continue

            r, g, b = color_map[name]
            tip = res['tip']
            axis = res['axis']

            # A. EXACT TIP SPHERE (Target orifice entry point)
            # Exactly centered at the distal tip apex with 12mm diameter
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'teat_tips'
            sphere.id = m_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(tip[0])
            sphere.pose.position.y = float(tip[1])
            sphere.pose.position.z = float(tip[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.012
            sphere.scale.y = 0.012
            sphere.scale.z = 0.012
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)
            m_id += 1

            # B. 3D APPROACH VECTOR ARROW (Trajectory of the milking cup)
            # Starts 4.5cm directly below the tip in space and points straight into the tip apex
            arrow = Marker()
            arrow.header.frame_id = frame_id
            arrow.header.stamp = stamp
            arrow.ns = 'teat_approach_arrows'
            arrow.id = m_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            # Start point: 4.5cm below tip along approach vector
            p_start = Point()
            p_start.x = float(tip[0] + 0.045 * axis[0])
            p_start.y = float(tip[1] + 0.045 * axis[1])
            p_start.z = float(tip[2] + 0.045 * axis[2])

            # End point: EXACTLY at the distal tip apex
            p_end = Point()
            p_end.x = float(tip[0])
            p_end.y = float(tip[1])
            p_end.z = float(tip[2])

            arrow.points = [p_start, p_end]
            arrow.scale.x = 0.005  # shaft diameter
            arrow.scale.y = 0.010  # head diameter
            arrow.scale.z = 0.012  # head length
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = 0.95
            marker_array.markers.append(arrow)
            m_id += 1

            # C. Floating Text Tag (Displays exact coordinates and tilt angle)
            tag = Marker()
            tag.header.frame_id = frame_id
            tag.header.stamp = stamp
            tag.ns = 'teat_labels'
            tag.id = m_id
            tag.type = Marker.TEXT_VIEW_FACING
            tag.action = Marker.ADD
            tag.pose.position.x = float(tip[0])
            tag.pose.position.y = float(tip[1])
            tag.pose.position.z = float(tip[2]) - 0.025
            tag.pose.orientation.w = 1.0
            tag.scale.z = 0.018
            tag.color.r = 1.0
            tag.color.g = 1.0
            tag.color.b = 1.0
            tag.color.a = 1.0
            tag.text = f"{name} Tip [{tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}] ({res['tilt_deg']:.1f}°)"
            marker_array.markers.append(tag)
            m_id += 1

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = QuadTeat6DOFEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
