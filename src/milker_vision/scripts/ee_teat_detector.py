#!/usr/bin/env python3
"""
Phase 2: Arm-Mounted Close-Range Teat Tip Perception Node.

Subscribes to PointCloud2 from the end-effector RGB-D camera (ee_camera_link)
mounted on the milker cluster (part_2_3) looking upward at the udder from below.

At close range, the 4 teats protrude downward toward the camera, so they appear as
the 4 closest point clusters (smallest X depth in ee_camera_link).
Transforms detected teat tips into the 'world' frame using TF2.

Publishes:
  - /ee_teat_poses (geometry_msgs/PoseArray, frame_id='world')
  - /ee_teat_fl/pose, /ee_teat_fr/pose, /ee_teat_rl/pose, /ee_teat_rr/pose (PoseStamped, 'world')
  - /ee_teat_markers (visualization_msgs/MarkerArray, frame_id='world')
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PointStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener, TransformException


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion [x, y, z, w] to a 3x3 rotation matrix."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


class EndEffectorTeatDetector(Node):
    """Detects 4 teats at close range from the arm-mounted upward-looking depth camera."""

    def __init__(self):
        super().__init__('ee_teat_detector')

        # Depth window in ee_camera_link (+X points straight UP toward udder)
        self.declare_parameter('depth_min', 0.015)
        self.declare_parameter('depth_max', 0.12)
        self.declare_parameter('protrusion_slice', 0.028)
        self.declare_parameter('lateral_bound', 0.16)
        self.declare_parameter('min_cluster_points', 8)
        self.declare_parameter('confirm_frames_required', 8)
        self.declare_parameter('confirm_tolerance_m', 0.012)
        self.declare_parameter('always_active', False)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arm_state = 'IDLE'
        self.was_active = False
        self.confirm_count = 0
        self.prev_tips = None
        self.tips_confirmed = False
        self.state_sub = self.create_subscription(String, '/arm_state', self.state_callback, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/ee_camera/depth/points',
            self.pointcloud_callback,
            10,
        )

        self.poses_pub = self.create_publisher(PoseArray, '/ee_teat_poses', 10)
        self.pub_fl = self.create_publisher(PoseStamped, '/ee_teat_fl/pose', 10)
        self.pub_fr = self.create_publisher(PoseStamped, '/ee_teat_fr/pose', 10)
        self.pub_rl = self.create_publisher(PoseStamped, '/ee_teat_rl/pose', 10)
        self.pub_rr = self.create_publisher(PoseStamped, '/ee_teat_rr/pose', 10)
        self.confirmed_pub = self.create_publisher(Bool, '/teat_tips_confirmed', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ee_teat_markers', 10)

        self.get_logger().info('EndEffectorTeatDetector initialized on /ee_camera/depth/points.')

    def state_callback(self, msg: String):
        self.arm_state = msg.data

    def pointcloud_callback(self, msg: PointCloud2):
        always_active = self.get_parameter('always_active').value
        active_states = {'POSITIONED', 'FINE_ALIGN', 'DOCKING_FL', 'DOCKING_FR', 'DOCKING_RL', 'DOCKING_RR', 'MILKING'}
        is_active = always_active or (self.arm_state in active_states)
        if not is_active:
            self.was_active = False
            return

        if not self.was_active:
            self.was_active = True
            self.get_logger().info(f'[Phase 2] EE Camera ACTIVATED in state {self.arm_state}! Scanning udder from below...')

        try:
            pts_struct = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        except Exception as e:
            self.get_logger().error(f'Failed to parse EE PointCloud2: {e}')
            return

        xyz = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=-1)
        valid = xyz[np.isfinite(xyz).all(axis=1)]
        if len(valid) < 20:
            return

        d_min = self.get_parameter('depth_min').value
        d_max = self.get_parameter('depth_max').value
        lat = self.get_parameter('lateral_bound').value
        slice_thick = self.get_parameter('protrusion_slice').value
        min_pts = self.get_parameter('min_cluster_points').value

        # Isolate close-range ROI in front of EE camera
        roi_mask = (
            (valid[:, 0] >= d_min) & (valid[:, 0] <= d_max) &
            (np.abs(valid[:, 1]) <= lat) &
            (np.abs(valid[:, 2]) <= lat)
        )
        roi_pts = valid[roi_mask]
        if len(roi_pts) < min_pts * 2:
            return

        # Teats are the closest protruding features (smallest X in ee_camera_link)
        closest_x = float(np.percentile(roi_pts[:, 0], 2.0))
        teat_pts_cam = roi_pts[roi_pts[:, 0] <= closest_x + slice_thick]
        if len(teat_pts_cam) < min_pts * 2:
            return

        # Transform teat points to world frame using TF2
        frame_id = msg.header.frame_id if msg.header.frame_id else 'ee_camera_link'
        try:
            tf = self.tf_buffer.lookup_transform(
                'world',
                frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup world <- {frame_id} failed: {ex}',
                throttle_duration_sec=2.0,
            )
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        R = quat_to_rot_matrix(r.x, r.y, r.z, r.w)
        trans = np.array([t.x, t.y, t.z], dtype=float)
        teat_pts_world = (teat_pts_cam @ R.T) + trans

        # Partition into 4 anatomical quadrants in world frame using bbox midpoint
        # (invariant to point-density differences between teats)
        # Cow faces -X (fore = smaller world X, rear = larger world X); Left = +Y, Right = -Y
        x_min, x_max = float(np.min(teat_pts_world[:, 0])), float(np.max(teat_pts_world[:, 0]))
        y_min, y_max = float(np.min(teat_pts_world[:, 1])), float(np.max(teat_pts_world[:, 1]))
        # Require at least 6cm span in X and Y so a single teat cannot be mis-split into 4
        if (x_max - x_min) < 0.06 or (y_max - y_min) < 0.06:
            return
        cx = 0.5 * (x_min + x_max)
        cy = 0.5 * (y_min + y_max)

        clusters = {
            'FL': teat_pts_world[(teat_pts_world[:, 0] < cx) & (teat_pts_world[:, 1] >= cy)],
            'FR': teat_pts_world[(teat_pts_world[:, 0] < cx) & (teat_pts_world[:, 1] < cy)],
            'RL': teat_pts_world[(teat_pts_world[:, 0] >= cx) & (teat_pts_world[:, 1] >= cy)],
            'RR': teat_pts_world[(teat_pts_world[:, 0] >= cx) & (teat_pts_world[:, 1] < cy)],
        }

        tips = {}
        for name, pts in clusters.items():
            if len(pts) < min_pts:
                continue
            # Lowest world Z points correspond to the distal teat tip apex
            z_low = float(np.percentile(pts[:, 2], 10.0))
            apex_pts = pts[pts[:, 2] <= z_low + 0.006]
            tips[name] = np.mean(apex_pts, axis=0)

        if not tips:
            return

        stamp = msg.header.stamp
        pose_pubs = {
            'FL': self.pub_fl,
            'FR': self.pub_fr,
            'RL': self.pub_rl,
            'RR': self.pub_rr,
        }

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = 'world'

        for name in ['FL', 'FR', 'RL', 'RR']:
            tip = tips.get(name)
            if tip is None:
                continue
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'world'
            ps.pose.position.x = float(tip[0])
            ps.pose.position.y = float(tip[1])
            ps.pose.position.z = float(tip[2])
            ps.pose.orientation.w = 1.0
            pose_pubs[name].publish(ps)

            p = Pose()
            p.position = ps.pose.position
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self.poses_pub.publish(pose_array)
        self.publish_markers(stamp, tips)

        # Confirm all 4 teat tips are stable over N consecutive frames
        if len(tips) == 4:
            req_frames = self.get_parameter('confirm_frames_required').value
            tol = self.get_parameter('confirm_tolerance_m').value
            if self.prev_tips is not None and all(
                np.linalg.norm(tips[k] - self.prev_tips[k]) < tol for k in ['FL', 'FR', 'RL', 'RR']
            ):
                self.confirm_count += 1
            else:
                self.confirm_count = 0
            self.prev_tips = {k: tips[k].copy() for k in ['FL', 'FR', 'RL', 'RR']}

            if self.confirm_count >= req_frames and not self.tips_confirmed:
                self.tips_confirmed = True
                self.get_logger().info(
                    f'[Phase 2] ALL 4 TEAT TIPS CONFIRMED ({req_frames} stable frames): '
                    + ' | '.join(f'{k}:[{tips[k][0]:.3f},{tips[k][1]:.3f},{tips[k][2]:.3f}]' for k in ['FL', 'FR', 'RL', 'RR'])
                )
        else:
            self.confirm_count = 0

        conf_msg = Bool()
        conf_msg.data = self.tips_confirmed
        self.confirmed_pub.publish(conf_msg)

        summary = ' | '.join(
            f'{k}:[{v[0]:.3f},{v[1]:.3f},{v[2]:.3f}]' for k, v in tips.items()
        )
        self.get_logger().info(f'EE Close-Range Teat Tips (world) -> {summary}', throttle_duration_sec=1.5)

    def publish_markers(self, stamp, tips):
        ma = MarkerArray()
        colors = {
            'FL': (0.1, 0.9, 0.1),
            'FR': (0.1, 0.8, 0.9),
            'RL': (0.9, 0.8, 0.1),
            'RR': (0.9, 0.2, 0.2),
        }
        for idx, name in enumerate(['FL', 'FR', 'RL', 'RR']):
            tip = tips.get(name)
            if tip is None:
                continue
            r, g, b = colors[name]
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = stamp
            m.ns = 'ee_teat_tips'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(tip[0])
            m.pose.position.y = float(tip[1])
            m.pose.position.z = float(tip[2])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.016
            m.scale.y = 0.016
            m.scale.z = 0.016
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorTeatDetector()
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
