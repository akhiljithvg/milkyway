#!/usr/bin/env python3
"""
Phase 1: Overhead Cow Body Detection & Imaginary Udder Localization Node.

Subscribes to PointCloud2 from the overhead RGB-D camera mounted above the cow's spine.
1. Isolates the cow body point cloud above the floor.
2. Computes the cow body bounding box and spine center (X, Y, Z in world frame).
3. Publishes virtual vertical + horizontal crosshair lines and bounding box markers.
4. Estimates the imaginary udder position using known cow anatomical offsets from spine center.
5. Locks and publishes the udder target waypoint once stable for N consecutive frames.

Publishes:
  - /estimated_udder_position (geometry_msgs/PointStamped, frame_id='world')
  - /udder_position_locked (std_msgs/Bool)
  - /overhead_markers (visualization_msgs/MarkerArray)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2


class OverheadCowDetector(Node):
    """Detects cow spine center from overhead depth and locks estimated udder position."""

    def __init__(self):
        super().__init__('overhead_detector')

        # Overhead camera static pose in world frame (matches vision_rig.xacro)
        # origin xyz="0.35 0.0 2.30" rpy="0 pi/2 0"
        self.declare_parameter('cam_world_x', 0.35)
        self.declare_parameter('cam_world_y', 0.0)
        self.declare_parameter('cam_world_z', 2.30)

        # Cow height band in world Z (filters out floor Z=0 and ceiling)
        # Tight Y bounds exclude the side-mounted robot gantry
        self.declare_parameter('cow_z_min', 0.70)
        self.declare_parameter('cow_z_max', 1.80)
        self.declare_parameter('cow_x_min', -1.20)
        self.declare_parameter('cow_x_max', 1.60)
        self.declare_parameter('cow_y_min', -0.25)
        self.declare_parameter('cow_y_max', 0.25)

        # Known anatomical offset from cow spine center to udder base (meters)
        self.declare_parameter('udder_offset_x', 0.16)
        self.declare_parameter('udder_offset_y', 0.0)
        self.declare_parameter('udder_offset_z', -0.94)

        # Lock stability parameters
        self.declare_parameter('lock_frames_required', 10)
        self.declare_parameter('lock_tolerance_m', 0.02)

        self.sub = self.create_subscription(
            PointCloud2,
            '/overhead_camera/depth/points',
            self.pointcloud_callback,
            10,
        )

        self.udder_pub = self.create_publisher(PointStamped, '/estimated_udder_position', 10)
        self.locked_pub = self.create_publisher(Bool, '/udder_position_locked', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/overhead_markers', 10)

        self.is_locked = False
        self.locked_pos = None
        self.stable_count = 0
        self.prev_estimate = None

        self.get_logger().info('OverheadCowDetector initialized. Waiting for /overhead_camera/depth/points...')

    def camera_to_world(self, pts_cam: np.ndarray) -> np.ndarray:
        """Transform points from overhead_camera_link (rpy=0, pi/2, 0) to world frame."""
        cx = self.get_parameter('cam_world_x').value
        cy = self.get_parameter('cam_world_y').value
        cz = self.get_parameter('cam_world_z').value

        # Rotation rpy=(0, +pi/2, 0):
        # world_x = cam_x_origin + pt_z
        # world_y = cam_y_origin + pt_y
        # world_z = cam_z_origin - pt_x
        pts_world = np.empty_like(pts_cam)
        pts_world[:, 0] = cx + pts_cam[:, 2]
        pts_world[:, 1] = cy + pts_cam[:, 1]
        pts_world[:, 2] = cz - pts_cam[:, 0]
        return pts_world

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            pts_struct = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        except Exception as e:
            self.get_logger().error(f'Failed to parse overhead PointCloud2: {e}')
            return

        xyz_cam = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=-1)
        valid_cam = xyz_cam[np.isfinite(xyz_cam).all(axis=1)]
        if len(valid_cam) < 50:
            return

        pts_world = self.camera_to_world(valid_cam)

        # Filter to cow body bounding volume
        z_min = self.get_parameter('cow_z_min').value
        z_max = self.get_parameter('cow_z_max').value
        x_min = self.get_parameter('cow_x_min').value
        x_max = self.get_parameter('cow_x_max').value
        y_min = self.get_parameter('cow_y_min').value
        y_max = self.get_parameter('cow_y_max').value

        mask = (
            (pts_world[:, 0] >= x_min) & (pts_world[:, 0] <= x_max) &
            (pts_world[:, 1] >= y_min) & (pts_world[:, 1] <= y_max) &
            (pts_world[:, 2] >= z_min) & (pts_world[:, 2] <= z_max)
        )
        cow_pts = pts_world[mask]
        if len(cow_pts) < 100:
            return

        # Cow bounding box and spine center
        bb_min = np.min(cow_pts, axis=0)
        bb_max = np.max(cow_pts, axis=0)
        cow_center = np.mean(cow_pts, axis=0)
        spine_z = float(np.percentile(cow_pts[:, 2], 90))

        # Apply anatomical offset from spine center to estimate udder position
        off_x = self.get_parameter('udder_offset_x').value
        off_y = self.get_parameter('udder_offset_y').value
        off_z = self.get_parameter('udder_offset_z').value

        udder_est = np.array([
            cow_center[0] + off_x,
            cow_center[1] + off_y,
            spine_z + off_z,
        ], dtype=float)

        # Lock logic: confirm stability over N frames
        if not self.is_locked:
            tol = self.get_parameter('lock_tolerance_m').value
            req = self.get_parameter('lock_frames_required').value
            if self.prev_estimate is not None and np.linalg.norm(udder_est - self.prev_estimate) < tol:
                self.stable_count += 1
            else:
                self.stable_count = 0
            self.prev_estimate = udder_est.copy()

            if self.stable_count >= req:
                self.is_locked = True
                self.locked_pos = udder_est.copy()
                self.get_logger().info(
                    f'UDDER POSITION LOCKED at world [{self.locked_pos[0]:.3f}, '
                    f'{self.locked_pos[1]:.3f}, {self.locked_pos[2]:.3f}]'
                )

        target_pos = self.locked_pos if self.is_locked else udder_est

        # Publish estimated udder position in world frame
        pt_msg = PointStamped()
        pt_msg.header.stamp = msg.header.stamp
        pt_msg.header.frame_id = 'world'
        pt_msg.point.x = float(target_pos[0])
        pt_msg.point.y = float(target_pos[1])
        pt_msg.point.z = float(target_pos[2])
        self.udder_pub.publish(pt_msg)

        # Publish lock flag
        lock_msg = Bool()
        lock_msg.data = self.is_locked
        self.locked_pub.publish(lock_msg)

        # Publish RViz crosshair + bounding box + udder estimate markers
        self.publish_markers(msg.header.stamp, cow_center, spine_z, bb_min, bb_max, target_pos)

        status = 'LOCKED' if self.is_locked else f'TRACKING ({self.stable_count})'
        self.get_logger().info(
            f'Overhead [{status}] CowCenter=[{cow_center[0]:.3f}, {cow_center[1]:.3f}, {spine_z:.3f}] '
            f'-> UdderEst=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]',
            throttle_duration_sec=1.5,
        )

    def publish_markers(self, stamp, cow_center, spine_z, bb_min, bb_max, udder_pos):
        ma = MarkerArray()

        # 1. Horizontal & Vertical Crosshair Lines intersecting at cow spine center
        crosshair = Marker()
        crosshair.header.frame_id = 'world'
        crosshair.header.stamp = stamp
        crosshair.ns = 'overhead_crosshair'
        crosshair.id = 0
        crosshair.type = Marker.LINE_LIST
        crosshair.action = Marker.ADD
        crosshair.scale.x = 0.015
        crosshair.color.r = 0.0
        crosshair.color.g = 1.0
        crosshair.color.b = 0.2
        crosshair.color.a = 0.95

        z_line = float(spine_z + 0.05)
        cx, cy = float(cow_center[0]), float(cow_center[1])
        # Longitudinal line (spine axis)
        crosshair.points.extend([
            Point(x=float(bb_min[0] - 0.15), y=cy, z=z_line),
            Point(x=float(bb_max[0] + 0.15), y=cy, z=z_line),
            # Transverse line (perpendicular across back)
            Point(x=cx, y=float(bb_min[1] - 0.25), z=z_line),
            Point(x=cx, y=float(bb_max[1] + 0.25), z=z_line),
            # Drop line from spine center down to imaginary udder position
            Point(x=cx, y=cy, z=z_line),
            Point(x=float(udder_pos[0]), y=float(udder_pos[1]), z=float(udder_pos[2])),
        ])
        ma.markers.append(crosshair)

        # 2. Cow body bounding box
        bbox = Marker()
        bbox.header.frame_id = 'world'
        bbox.header.stamp = stamp
        bbox.ns = 'cow_bbox'
        bbox.id = 1
        bbox.type = Marker.CUBE
        bbox.action = Marker.ADD
        bbox.pose.position.x = float((bb_min[0] + bb_max[0]) * 0.5)
        bbox.pose.position.y = float((bb_min[1] + bb_max[1]) * 0.5)
        bbox.pose.position.z = float((bb_min[2] + bb_max[2]) * 0.5)
        bbox.pose.orientation.w = 1.0
        bbox.scale.x = float(max(bb_max[0] - bb_min[0], 0.1))
        bbox.scale.y = float(max(bb_max[1] - bb_min[1], 0.1))
        bbox.scale.z = float(max(bb_max[2] - bb_min[2], 0.1))
        bbox.color.r = 0.2
        bbox.color.g = 0.8
        bbox.color.b = 1.0
        bbox.color.a = 0.18
        ma.markers.append(bbox)

        # 3. Imaginary Udder Target Sphere
        udder_sphere = Marker()
        udder_sphere.header.frame_id = 'world'
        udder_sphere.header.stamp = stamp
        udder_sphere.ns = 'estimated_udder'
        udder_sphere.id = 2
        udder_sphere.type = Marker.SPHERE
        udder_sphere.action = Marker.ADD
        udder_sphere.pose.position.x = float(udder_pos[0])
        udder_sphere.pose.position.y = float(udder_pos[1])
        udder_sphere.pose.position.z = float(udder_pos[2])
        udder_sphere.pose.orientation.w = 1.0
        udder_sphere.scale.x = 0.10
        udder_sphere.scale.y = 0.10
        udder_sphere.scale.z = 0.10
        if self.is_locked:
            udder_sphere.color.r, udder_sphere.color.g, udder_sphere.color.b = 1.0, 0.2, 0.2
        else:
            udder_sphere.color.r, udder_sphere.color.g, udder_sphere.color.b = 1.0, 0.8, 0.0
        udder_sphere.color.a = 0.85
        ma.markers.append(udder_sphere)

        # 4. Status Label
        label = Marker()
        label.header.frame_id = 'world'
        label.header.stamp = stamp
        label.ns = 'overhead_status'
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = cx
        label.pose.position.y = cy
        label.pose.position.z = z_line + 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.08
        label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
        status_str = 'LOCKED' if self.is_locked else 'SCANNING'
        label.text = f'OVERHEAD [{status_str}] Udder=({udder_pos[0]:.2f}, {udder_pos[1]:.2f}, {udder_pos[2]:.2f})'
        ma.markers.append(label)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = OverheadCowDetector()
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
