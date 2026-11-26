#!/usr/bin/env python3
"""
Robotic Milking Arm Controller & Sequential Cup Docking State Machine.

Coordinates the two-phase autonomous milking sequence:
  1. IDLE: Arm parked at cow's side (yaw = 0). Waits for /udder_position_locked.
  2. APPROACHING: Moves gantry (base_to_part_2), swing yaw (part_4_to_part_4__2),
     and vertical rod (part_4__3_to_part_4) to place the milker cluster under
     the locked udder waypoint from the overhead camera.
  3. POSITIONED: Waits for end-effector camera close-range teat detection.
  4. FINE_ALIGN: Refines cluster position using the EE camera teat centroid.
  5. DOCKING_FL -> DOCKING_FR -> DOCKING_RL -> DOCKING_RR:
     Aligns and attaches suction cups sequentially one by one to each teat.
  6. MILKING: Holds all 4 cups docked for milking duration.
  7. RETRACTING: Lowers cluster and swings arm back to parked position.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Float64, String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = [
    'base_to_part_2',
    'part_2_to_part_3',
    'part_3_to_part_4__3',
    'part_4__3_to_part_4',
    'part_4_to_part_4__2',
    'part_4__2_to_part_5',
    'part_5_to_part_2_3',
]

TEAT_SEQUENCE = ['FL', 'FR', 'RL', 'RR']


class ArmController(Node):
    """State-machine controller for 7-DOF milking arm and sequential cup docking."""

    def __init__(self):
        super().__init__('arm_controller')

        # Subscribers
        self.udder_sub = self.create_subscription(
            PointStamped, '/estimated_udder_position', self.udder_callback, 10
        )
        self.locked_sub = self.create_subscription(
            Bool, '/udder_position_locked', self.locked_callback, 10
        )
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.ee_teats = {}
        self.tips_confirmed = False
        self.create_subscription(Bool, '/teat_tips_confirmed', self.tips_confirmed_cb, 10)
        self.create_subscription(PoseStamped, '/ee_teat_fl/pose', lambda m: self._teat_cb('FL', m), 10)
        self.create_subscription(PoseStamped, '/ee_teat_fr/pose', lambda m: self._teat_cb('FR', m), 10)
        self.create_subscription(PoseStamped, '/ee_teat_rl/pose', lambda m: self._teat_cb('RL', m), 10)
        self.create_subscription(PoseStamped, '/ee_teat_rr/pose', lambda m: self._teat_cb('RR', m), 10)

        # Joint position command publishers
        self.cmd_pubs = {
            j: self.create_publisher(Float64, f'/arm/{j}/cmd_pos', 10)
            for j in JOINT_NAMES
        }

        self.state_pub = self.create_publisher(String, '/arm_state', 10)
        self.dock_status_pub = self.create_publisher(String, '/docking_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/docking_markers', 10)

        # Safe clearance below estimated udder base (world Z):
        # udder_base=0.50m, fore teat tips=0.288m, safe_cam_z=0.249m (3.9cm below tips, milk tubes +1.4cm above ground)
        self.declare_parameter('safe_clearance_z', 0.248)
        # Keep part_4 telescoped +0.15m inside part_4__3 so it never separates visually
        self.telescoping_nest_z = 0.15
        # At all other joints=0, part_4__3_to_part_4=0.15, and ee_cam z=0.115, ee_camera_link is at world Z = 0.6172 m
        self.ee_cam_zero_z = 0.4672 + self.telescoping_nest_z

        # Slow & smooth slew-rate limits per 20Hz control tick (0.05s)
        self.max_lin_step = 0.004   # 0.08 m/s
        self.max_ang_step = 0.012   # 0.24 rad/s

        # State variables
        self.state = 'IDLE'
        self.state_enter_time = self.get_clock().now()
        self.udder_locked = False
        self.locked_udder_xyz = None
        self.current_joints = {j: 0.0 for j in JOINT_NAMES}
        self.target_joints = {j: 0.0 for j in JOINT_NAMES}
        self.cmd_joints = {j: 0.0 for j in JOINT_NAMES}
        self.cmd_joints['part_4__3_to_part_4'] = self.telescoping_nest_z
        self.docked_cups = {t: False for t in TEAT_SEQUENCE}
        self.dock_targets = {}
        self.dock_subphase = 'ALIGN'  # 'ALIGN' -> 'LIFT' -> 'CONFIRM'

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('ArmController initialized in IDLE state. Waiting for /udder_position_locked...')

    def tips_confirmed_cb(self, msg: Bool):
        if msg.data and not self.tips_confirmed:
            self.tips_confirmed = True
            self.get_logger().info('[Phase 2] ArmController received 4-teat tip confirmation!')

    def _teat_cb(self, name: str, msg: PoseStamped):
        self.ee_teats[name] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)

    def udder_callback(self, msg: PointStamped):
        if self.locked_udder_xyz is None or not self.udder_locked:
            self.locked_udder_xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

    def locked_callback(self, msg: Bool):
        if msg.data and not self.udder_locked:
            self.udder_locked = True
            self.get_logger().info('Received udder position LOCK! Starting Phase 1 -> Arm Approach.')

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joints:
                self.current_joints[name] = pos

    def transition(self, new_state: str):
        self.get_logger().info(f'State transition: {self.state} -> {new_state}')
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

    def elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def joints_near_target(self, tol: float = 0.03) -> bool:
        for j in ['base_to_part_2', 'part_2_to_part_3', 'part_4_to_part_4__2']:
            if abs(self.current_joints[j] - self.target_joints[j]) > tol:
                return False
        return True

    def compute_gantry_and_yaw_for_world_xy(self, target_x: float, target_y: float):
        """
        Exact analytical 2-DOF planar IK for ee_camera_link (center of 4-cup cluster).
        Pivot in world at gantry=0 is (0.100, -0.408).
        Camera offset from pivot at yaw=0 is (0.5178, -0.0325) -> L=0.5188m, phi0=-0.0627 rad.
        """
        L = 0.5188
        phi0 = -0.0627
        sin_arg = np.clip((target_y - (-0.408)) / L, -0.95, 0.95)
        yaw = float(np.arcsin(sin_arg) - phi0)
        gantry_y = float(np.clip(target_x - 0.100 - L * np.cos(phi0 + yaw), -0.80, 0.60))
        return gantry_y, yaw

    def compute_carriage_z_joint(self, desired_cam_z: float) -> float:
        """Compute part_2_to_part_3 carriage target so part_4 stays nested in part_4__3."""
        return float(np.clip(desired_cam_z - self.ee_cam_zero_z, -0.38, 0.25))

    def control_loop(self):
        elapsed = self.elapsed_in_state()
        safe_clear = self.get_parameter('safe_clearance_z').value

        if self.state == 'IDLE':
            # Park at side with part_4 nested inside part_4__3
            self.target_joints = {j: 0.0 for j in JOINT_NAMES}
            self.target_joints['part_4__3_to_part_4'] = self.telescoping_nest_z
            if self.udder_locked and self.locked_udder_xyz is not None:
                gantry_y, yaw = self.compute_gantry_and_yaw_for_world_xy(
                    self.locked_udder_xyz[0], self.locked_udder_xyz[1]
                )
                safe_cam_z = self.locked_udder_xyz[2] - safe_clear
                self.target_joints['base_to_part_2'] = gantry_y
                self.target_joints['part_4_to_part_4__2'] = yaw
                self.target_joints['part_2_to_part_3'] = self.compute_carriage_z_joint(safe_cam_z)
                self.target_joints['part_4__3_to_part_4'] = self.telescoping_nest_z
                self.get_logger().info(
                    f'[Phase 1->2] Moving arm below udder at safe Z={safe_cam_z:.3f}m '
                    f'({safe_clear:.2f}m clearance below udder)'
                )
                self.transition('APPROACHING')

        elif self.state == 'APPROACHING':
            if (self.joints_near_target(tol=0.03) and elapsed > 2.5) or elapsed > 12.0:
                self.transition('POSITIONED')

        elif self.state == 'POSITIONED':
            # Wait for EE camera to confirm all 4 teat tips stably
            if (self.tips_confirmed and len(self.ee_teats) == 4 and elapsed > 2.0) or elapsed > 8.0:
                self.transition('FINE_ALIGN')

        elif self.state == 'FINE_ALIGN':
            if self.ee_teats:
                centroid = np.mean(list(self.ee_teats.values()), axis=0)
                gantry_y, yaw = self.compute_gantry_and_yaw_for_world_xy(centroid[0], centroid[1])
                self.target_joints['base_to_part_2'] = gantry_y
                self.target_joints['part_4_to_part_4__2'] = yaw
            if (self.joints_near_target(tol=0.025) and elapsed > 2.0) or elapsed > 5.0:
                self.dock_targets = dict(self.ee_teats)
                self.dock_subphase = 'ALIGN'
                self.transition('DOCKING_FL')

        elif self.state.startswith('DOCKING_'):
            teat_name = self.state.split('_')[1]
            safe_cam_z = (self.locked_udder_xyz[2] - safe_clear) if self.locked_udder_xyz is not None else 0.255
            safe_z_joint = self.compute_carriage_z_joint(safe_cam_z)

            t_pos = self.dock_targets.get(teat_name, self.ee_teats.get(teat_name))
            if t_pos is not None:
                gantry_y, yaw = self.compute_gantry_and_yaw_for_world_xy(t_pos[0], t_pos[1])
                self.target_joints['base_to_part_2'] = gantry_y
                self.target_joints['part_4_to_part_4__2'] = yaw
                dock_z_joint = self.compute_carriage_z_joint(t_pos[2] - 0.015)
            else:
                dock_z_joint = self.compute_carriage_z_joint(0.28)

            if self.dock_subphase == 'ALIGN':
                self.target_joints['part_2_to_part_3'] = safe_z_joint
                if (self.joints_near_target(tol=0.025) and elapsed > 1.5) or elapsed > 4.0:
                    self.dock_subphase = 'LIFT'
            elif self.dock_subphase == 'LIFT':
                self.target_joints['part_2_to_part_3'] = dock_z_joint
                if (abs(self.current_joints['part_2_to_part_3'] - dock_z_joint) < 0.015 and elapsed > 2.5) or elapsed > 6.0:
                    self.docked_cups[teat_name] = True
                    self.get_logger().info(f'[Phase 3] Suction cup DOCKED on teat {teat_name}!')
                    idx = TEAT_SEQUENCE.index(teat_name)
                    if idx + 1 < len(TEAT_SEQUENCE):
                        self.target_joints['part_2_to_part_3'] = safe_z_joint
                        self.dock_subphase = 'ALIGN'
                        self.transition(f'DOCKING_{TEAT_SEQUENCE[idx + 1]}')
                    else:
                        self.transition('MILKING')

        elif self.state == 'MILKING':
            if elapsed > 5.0:
                safe_cam_z = (self.locked_udder_xyz[2] - safe_clear) if self.locked_udder_xyz is not None else 0.255
                self.target_joints['part_2_to_part_3'] = self.compute_carriage_z_joint(safe_cam_z)
                self.transition('RETRACTING')

        elif self.state == 'RETRACTING':
            self.target_joints = {j: 0.0 for j in JOINT_NAMES}
            self.target_joints['part_4__3_to_part_4'] = self.telescoping_nest_z
            if elapsed > 6.0:
                self.get_logger().info('Milking cycle complete. Arm parked.')

        # Smooth slew-rate ramp for slow, accurate arm motion
        prismatic_joints = {'base_to_part_2', 'part_2_to_part_3', 'part_4__3_to_part_4'}
        for j, tgt in self.target_joints.items():
            step = self.max_lin_step if j in prismatic_joints else self.max_ang_step
            diff = tgt - self.cmd_joints[j]
            if abs(diff) <= step:
                self.cmd_joints[j] = tgt
            else:
                self.cmd_joints[j] += np.sign(diff) * step

        # Publish smoothed joint commands
        for j, val in self.cmd_joints.items():
            msg = Float64()
            msg.data = float(val)
            self.cmd_pubs[j].publish(msg)

        # Publish state & docking status
        st_msg = String()
        st_msg.data = self.state
        self.state_pub.publish(st_msg)

        dock_msg = String()
        docked_list = [k for k, v in self.docked_cups.items() if v]
        dock_msg.data = f'State={self.state} | Docked={docked_list}'
        self.dock_status_pub.publish(dock_msg)

        self.publish_docking_markers()

    def publish_docking_markers(self):
        targets = self.dock_targets if self.dock_targets else self.ee_teats
        if not targets:
            return
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, name in enumerate(TEAT_SEQUENCE):
            pos = targets.get(name)
            if pos is None:
                continue
            docked = self.docked_cups[name]
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = stamp
            m.ns = 'suction_cups'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2] - 0.02)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.032
            m.scale.y = 0.032
            m.scale.z = 0.045
            if docked:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.95, 0.2, 0.95
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.7, 0.7, 0.45
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
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
