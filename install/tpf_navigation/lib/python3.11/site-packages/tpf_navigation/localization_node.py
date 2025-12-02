"""Localizacion en mapa conocido usando filtro de particulas."""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw: float):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass
class Particle:
    x: float
    y: float
    theta: float
    weight: float


class ParticleLocalization(Node):
    """Filtro de particulas tipo AMCL ligero."""

    def __init__(self) -> None:
        super().__init__('localization_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_sim_time', False),
                ('num_particles', 250),
                ('motion_alphas', [0.15, 0.05, 0.1, 0.05]),
                ('scan_step', 8),
                ('hit_bonus', 2.0),
                ('miss_penalty', 1.0),
                ('weight_scale', 0.35),
                ('min_weight', 1.0e-6),
                ('occ_threshold', 50),
                ('initial_std_xy', 0.15),
                ('initial_std_theta', 0.35),
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_footprint'),
                ('scan_topic', 'scan'),
                ('odom_topic', 'calc_odom'),
                ('map_topic', 'map'),
            ],
        )

        self.num_particles = int(self.get_parameter('num_particles').value)
        self.motion_alphas = list(self.get_parameter('motion_alphas').value)
        self.scan_step = int(self.get_parameter('scan_step').value)
        self.hit_bonus = float(self.get_parameter('hit_bonus').value)
        self.miss_penalty = float(self.get_parameter('miss_penalty').value)
        self.weight_scale = float(self.get_parameter('weight_scale').value)
        self.min_weight = float(self.get_parameter('min_weight').value)
        self.occ_threshold = int(self.get_parameter('occ_threshold').value)
        self.initial_std_xy = float(self.get_parameter('initial_std_xy').value)
        self.initial_std_theta = float(
            self.get_parameter('initial_std_theta').value
        )
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 5
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 50
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initialpose_callback,
            5,
        )

        self.particles: List[Particle] = []
        self.map_info: Optional[Tuple[int, int, float, float, float]] = None
        self.map_data: Optional[np.ndarray] = None
        self.current_odom: Optional[Tuple[float, float, float]] = None
        self.prev_scan_odom: Optional[Tuple[float, float, float]] = None

        self.rng = np.random.default_rng()
        self.get_logger().info('Localization node listo, esperando initialpose.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_info = (
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
        )
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.get_logger().info(
            f'Recibido mapa {msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f}'
        )

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.current_odom = (pose.position.x, pose.position.y, yaw)

    def initialpose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.initialize_particles(pose.position.x, pose.position.y, yaw)
        self.prev_scan_odom = self.current_odom
        self.get_logger().info(
            f'Inicializacion de particulas cerca de '
            f'({pose.position.x:.2f}, {pose.position.y:.2f})'
        )

    def scan_callback(self, scan: LaserScan) -> None:
        if (
            self.map_data is None
            or not self.particles
            or self.current_odom is None
        ):
            return

        if self.prev_scan_odom is None:
            self.prev_scan_odom = self.current_odom
            return

        delta_rot1, delta_trans, delta_rot2 = self.compute_motion_delta()
        self.motion_update(delta_rot1, delta_trans, delta_rot2)
        self.measurement_update(scan)
        self.resample_particles()

        estimate = self.estimate_pose()
        if estimate is None:
            return

        stamp = scan.header.stamp
        self.publish_pose(estimate, stamp)
        self.publish_tf(estimate, stamp)
        self.prev_scan_odom = self.current_odom

    # ------------------------------------------------------------------
    # Algoritmo PF
    # ------------------------------------------------------------------
    def initialize_particles(self, x: float, y: float, theta: float) -> None:
        self.particles = []
        for _ in range(self.num_particles):
            px = self.rng.normal(x, self.initial_std_xy)
            py = self.rng.normal(y, self.initial_std_xy)
            pt = self.rng.normal(theta, self.initial_std_theta)
            self.particles.append(
                Particle(px, py, wrap_angle(pt), 1.0 / self.num_particles)
            )

    def compute_motion_delta(self) -> Tuple[float, float, float]:
        assert self.current_odom is not None
        assert self.prev_scan_odom is not None
        x, y, theta = self.current_odom
        x_prev, y_prev, theta_prev = self.prev_scan_odom

        dx = x - x_prev
        dy = y - y_prev
        delta_trans = math.hypot(dx, dy)
        delta_rot1 = wrap_angle(math.atan2(dy, dx) - theta_prev)
        delta_rot2 = wrap_angle(theta - theta_prev - delta_rot1)

        if delta_trans < 1e-4:
            delta_rot1 = 0.0
            delta_rot2 = wrap_angle(theta - theta_prev)

        return delta_rot1, delta_trans, delta_rot2

    def motion_update(self, dr1: float, dt: float, dr2: float) -> None:
        a1, a2, a3, a4 = self.motion_alphas
        for p in self.particles:
            dr1_n = dr1 + self.rng.normal(0.0, a1 * abs(dr1) + a2 * dt)
            dt_n = dt + self.rng.normal(0.0, a3 * dt + a4 * (abs(dr1) + abs(dr2)))
            dr2_n = dr2 + self.rng.normal(0.0, a1 * abs(dr2) + a2 * dt)

            p.x += dt_n * math.cos(p.theta + dr1_n)
            p.y += dt_n * math.sin(p.theta + dr1_n)
            p.theta = wrap_angle(p.theta + dr1_n + dr2_n)

    def measurement_update(self, scan: LaserScan) -> None:
        if self.map_info is None or self.map_data is None:
            return
        width, height, resolution, ox, oy = self.map_info

        indices = range(0, len(scan.ranges), max(1, self.scan_step))
        total = 0.0
        for p in self.particles:
            score = 0.0
            for i in indices:
                r = scan.ranges[i]
                if not math.isfinite(r):
                    continue
                if r < scan.range_min or r > scan.range_max:
                    continue
                angle = p.theta + scan.angle_min + i * scan.angle_increment
                wx = p.x + r * math.cos(angle)
                wy = p.y + r * math.sin(angle)
                mx = int((wx - ox) / resolution)
                my = int((wy - oy) / resolution)
                if mx < 0 or mx >= width or my < 0 or my >= height:
                    continue
                idx = my * width + mx
                cell = self.map_data[idx]
                if cell >= self.occ_threshold:
                    score += self.hit_bonus
                elif cell == -1:
                    score -= 0.2 * self.miss_penalty
                else:
                    score -= self.miss_penalty
            p.weight = max(
                self.min_weight, math.exp(score * self.weight_scale)
            )
            total += p.weight

        if total > 0.0:
            for p in self.particles:
                p.weight /= total
        else:
            # Reseteo suave en caso degenerado
            for p in self.particles:
                p.weight = 1.0 / self.num_particles

    def resample_particles(self) -> None:
        weights = [p.weight for p in self.particles]
        if not weights:
            return
        step = 1.0 / self.num_particles
        r = self.rng.random() * step
        c = weights[0]
        i = 0
        new_particles: List[Particle] = []
        for _ in range(self.num_particles):
            U = r + _ * step
            while U > c and i < self.num_particles - 1:
                i += 1
                c += weights[i]
            src = self.particles[i]
            new_particles.append(
                Particle(src.x, src.y, src.theta, 1.0 / self.num_particles)
            )
        self.particles = new_particles

    def estimate_pose(self) -> Optional[Tuple[float, float, float]]:
        if not self.particles:
            return None
        sum_w = sum(p.weight for p in self.particles)
        if sum_w <= 0.0:
            return None
        x = sum(p.x * p.weight for p in self.particles) / sum_w
        y = sum(p.y * p.weight for p in self.particles) / sum_w
        c = sum(math.cos(p.theta) * p.weight for p in self.particles) / sum_w
        s = sum(math.sin(p.theta) * p.weight for p in self.particles) / sum_w
        theta = math.atan2(s, c)
        return (x, y, theta)

    # ------------------------------------------------------------------
    # Publicacion
    # ------------------------------------------------------------------
    def publish_pose(
        self, pose: Tuple[float, float, float], stamp
    ) -> None:
        x, y, theta = pose
        q = quaternion_from_yaw(theta)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q
        # Covarianza sencilla basada en dispersión inicial
        cov = [
            self.initial_std_xy**2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.initial_std_xy**2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.initial_std_theta**2,
        ]
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

    def publish_tf(
        self, pose: Tuple[float, float, float], stamp
    ) -> None:
        if self.current_odom is None:
            return
        x_m, y_m, theta_m = pose
        x_o, y_o, theta_o = self.current_odom

        yaw_map_to_odom = wrap_angle(theta_m - theta_o)
        cos_y = math.cos(yaw_map_to_odom)
        sin_y = math.sin(yaw_map_to_odom)
        tx = x_m - (cos_y * x_o - sin_y * y_o)
        ty = y_m - (sin_y * x_o + cos_y * y_o)

        q = quaternion_from_yaw(yaw_map_to_odom)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParticleLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
