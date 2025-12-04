"""Seguidor de trayectoria simple (Pure Pursuit liviano) con replan."""

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__('controller_node')
        defaults = [
            ('use_sim_time', False),
            ('max_linear_speed', 0.2),
            ('max_angular_speed', 1.2),
            ('lookahead_distance', 0.35),
            ('goal_tolerance', 0.12),
            ('angle_tolerance', 0.25),
            ('obstacle_stop_distance', 0.25),
            ('obstacle_clear_distance', 0.3),
            ('obstacle_turn_speed', 0.4),
            ('obstacle_front_angle', 0.8),
            ('replan_distance_threshold', 0.25),
            ('replan_angle_threshold', 0.6),
            ('k_angular', 1.6),
            ('k_linear', 0.6),
            ('control_rate', 20.0),
            ('path_topic', '/plan'),
            ('pose_topic', '/amcl_pose'),
            ('cmd_topic', '/cmd_vel'),
            ('scan_topic', '/scan'),
            ('replan_topic', '/replan'),
        ]
        for name, default in defaults:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value
        )
        self.angle_tolerance = float(
            self.get_parameter('angle_tolerance').value
        )
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value
        )
        self.obstacle_clear_distance = float(
            self.get_parameter('obstacle_clear_distance').value
        )
        self.obstacle_turn_speed = float(
            self.get_parameter('obstacle_turn_speed').value
        )
        self.obstacle_front_angle = float(
            self.get_parameter('obstacle_front_angle').value
        )
        self.replan_distance_threshold = float(
            self.get_parameter('replan_distance_threshold').value
        )
        self.replan_angle_threshold = float(
            self.get_parameter('replan_angle_threshold').value
        )
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.control_period = 1.0 / float(
            self.get_parameter('control_rate').value
        )
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.replan_topic = str(self.get_parameter('replan_topic').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.replan_pub = self.create_publisher(Bool, self.replan_topic, 1)
        self.create_subscription(Path, self.path_topic, self.path_callback, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.create_timer(self.control_period, self.control_loop)

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.path_points: List[Tuple[float, float, float]] = []
        self.last_min_range: Optional[float] = None
        self.last_min_front: Optional[float] = None
        self.last_replan_msg_time = self.get_clock().now()
        self.replan_cooldown = Duration(seconds=1.0)
        self.state = 'IDLE'  # IDLE, FOLLOWING, OBSTACLE, AT_GOAL
        self.obstacle_turn_dir = 1.0
        self.pending_replan_after_clear = False

        self.get_logger().info('Controller listo, esperando plan global.')

    # ------------------------------------------------------------------
    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def path_callback(self, msg: Path) -> None:
        self.path_points = []
        for pose in msg.poses:
            p = pose.pose.position
            yaw = yaw_from_quaternion(pose.pose.orientation)
            self.path_points.append((p.x, p.y, yaw))
        if self.path_points:
            self.get_logger().info(
                f'Recibido plan con {len(self.path_points)} puntos.'
            )
            if self.state != 'OBSTACLE':
                self.state = 'FOLLOWING'

    def scan_callback(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r)]
        self.last_min_range = min(valid) if valid else None

        # Solo consideramos el cono frontal para evitar frenar por paredes laterales.
        half_angle = self.obstacle_front_angle * 0.5
        front_vals = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if -half_angle <= angle <= half_angle:
                front_vals.append(r)
        self.last_min_front = min(front_vals) if front_vals else None

    # ------------------------------------------------------------------
    def control_loop(self) -> None:
        if self.current_pose is None:
            return
        if not self.path_points:
            self.cmd_pub.publish(Twist())
            self.state = 'IDLE'
            return

        x, y, yaw = self.current_pose
        goal_x, goal_y, goal_yaw = self.path_points[-1]
        dist_goal = math.hypot(goal_x - x, goal_y - y)
        heading_goal = wrap_angle(goal_yaw - yaw)

        if dist_goal < self.goal_tolerance and abs(heading_goal) < self.angle_tolerance:
            self.cmd_pub.publish(Twist())
            self.state = 'AT_GOAL'
            return

        nearest_idx, nearest_dist = self.find_nearest_point(x, y)
        target_idx = nearest_idx
        for i in range(nearest_idx, len(self.path_points)):
            px, py, _ = self.path_points[i]
            if math.hypot(px - x, py - y) >= self.lookahead_distance:
                target_idx = i
                break
        tx, ty, _ = self.path_points[target_idx]
        heading = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        target_dist = math.hypot(tx - x, ty - y)

        obstacle_close = (
            self.last_min_front is not None
            and self.last_min_front < self.obstacle_stop_distance
        )
        obstacle_blocking = (
            self.last_min_front is not None
            and self.last_min_front < self.obstacle_clear_distance
        )
        obstacle_cleared = (
            self.last_min_front is None
            or self.last_min_front > self.obstacle_clear_distance
        )

        if obstacle_close or (self.state == 'OBSTACLE' and obstacle_blocking):
            if self.state != 'OBSTACLE':
                self.state = 'OBSTACLE'
                self.obstacle_turn_dir *= -1.0  # alterna sentido
                self.trigger_replan('Obstaculo cerca')
            # mientras está bloqueado, replan con cooldown
            self.pending_replan_after_clear = True
            self.trigger_replan('Obstaculo persiste')
            cmd = Twist()
            cmd.angular.z = self.obstacle_turn_speed * self.obstacle_turn_dir
            self.cmd_pub.publish(cmd)
            return
        if self.state == 'OBSTACLE' and obstacle_cleared:
            # al despejar, forzar un replan fresco
            if self.pending_replan_after_clear:
                self.trigger_replan('Obstaculo despejado, replan')
            self.pending_replan_after_clear = False
            self.state = 'FOLLOWING'

        if nearest_dist > self.replan_distance_threshold or abs(heading) > self.replan_angle_threshold:
            self.trigger_replan('Desvio respecto al plan')

        cmd = Twist()
        cmd.linear.x = min(
            self.max_linear_speed,
            self.k_linear * target_dist * max(0.0, math.cos(heading)),
        )
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, self.k_angular * heading),
        )
        if dist_goal < self.goal_tolerance:
            cmd.linear.x *= 0.3
        self.cmd_pub.publish(cmd)

    def find_nearest_point(self, x: float, y: float) -> Tuple[int, float]:
        best_idx = 0
        best_dist = float('inf')
        for i, (px, py, _) in enumerate(self.path_points):
            d = math.hypot(px - x, py - y)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx, best_dist

    def trigger_replan(self, reason: str) -> None:
        now = self.get_clock().now()
        if now - self.last_replan_msg_time < self.replan_cooldown:
            return
        self.last_replan_msg_time = now
        self.get_logger().info(f'Solicitando replan: {reason}')
        self.replan_pub.publish(Bool(data=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
