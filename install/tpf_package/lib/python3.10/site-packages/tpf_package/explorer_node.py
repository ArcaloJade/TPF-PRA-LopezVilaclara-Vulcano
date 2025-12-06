"""Nodo simple de exploracion para recorrer el laberinto."""

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimpleExplorer(Node):
    """Explorador reactivo tipo wall-follow que publica /cmd_vel."""

    def __init__(self) -> None:
        super().__init__('explorer_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_sim_time', False),
                ('linear_speed', 0.15),
                ('angular_speed', 0.8),
                ('obstacle_distance', 0.5),
                ('side_angle', 1.0),
            ],
        )
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.obstacle_distance = float(
            self.get_parameter('obstacle_distance').value
        )
        self.side_angle = float(self.get_parameter('side_angle').value)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan) -> None:
        front = self.min_distance(msg, -0.3, 0.3)
        left = self.min_distance(msg, 0.3, self.side_angle)
        right = self.min_distance(msg, -self.side_angle, -0.3)

        twist = Twist()
        if front < self.obstacle_distance:
            twist.angular.z = self.angular_speed
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.linear_speed
            if left > right + 0.1:
                twist.angular.z = -0.2 * self.angular_speed
            elif right > left + 0.1:
                twist.angular.z = 0.2 * self.angular_speed
        self.cmd_pub.publish(twist)

    def min_distance(
        self, scan: LaserScan, angle_min: float, angle_max: float
    ) -> float:
        """Devuelve la minima distancia en el intervalo de angulos pedido."""
        indices = self._indices_for_angles(scan, angle_min, angle_max)
        values: List[float] = []
        for idx in indices:
            if idx < 0 or idx >= len(scan.ranges):
                continue
            distance = scan.ranges[idx]
            if math.isfinite(distance):
                values.append(distance)
        return min(values) if values else float('inf')

    def _indices_for_angles(
        self, scan: LaserScan, angle_min: float, angle_max: float
    ) -> List[int]:
        start = int(
            (angle_min - scan.angle_min) / scan.angle_increment
        )
        end = int(
            (angle_max - scan.angle_min) / scan.angle_increment
        )
        if start > end:
            start, end = end, start
        start = max(0, min(start, len(scan.ranges) - 1))
        end = max(0, min(end, len(scan.ranges) - 1))
        return list(range(start, end + 1))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
