"""Planificador global basado en A* sobre el OccupancyGrid."""

import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import Bool


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class AStarPlanner(Node):
    def __init__(self) -> None:
        super().__init__('planner_node')
        defaults = [
            ('use_sim_time', False),
            ('obstacle_threshold', 50),
            ('inflation_radius', 0.15),
            ('connect_8', True),
            ('max_search_m', 25.0),
            ('plan_rate', 1.0),
            ('pose_topic', '/amcl_pose'),
            ('goal_topic', '/goal_pose'),
            ('map_topic', '/map'),
            ('path_topic', '/plan'),
            ('replan_topic', '/replan'),
            ('dynamic_obstacle_radius', 0.25),
            ('dynamic_obstacle_timeout', 5.0),
            ('dynamic_map_topic', 'dynamic_obstacle_map'),
        ]
        for name, default in defaults:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self.obstacle_threshold = int(
            self.get_parameter('obstacle_threshold').value
        )
        self.inflation_radius = float(
            self.get_parameter('inflation_radius').value
        )
        self.connect_8 = bool(self.get_parameter('connect_8').value)
        self.max_search_m = float(self.get_parameter('max_search_m').value)
        self.plan_period = 1.0 / float(self.get_parameter('plan_rate').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.replan_topic = str(self.get_parameter('replan_topic').value)

        self.map_data: Optional[np.ndarray] = None
        self.map_info: Optional[Tuple[int, int, float, float, float, str]] = None
        self.current_pose: Optional[PoseWithCovarianceStamped] = None
        self.goal_pose: Optional[PoseStamped] = None
        self.need_plan = False
        self.dynamic_obstacles: List[Tuple[float, float, float]] = []  # x, y, expiry
        self.dynamic_map_dirty = False

        self.path_pub = self.create_publisher(Path, self.path_topic, 1)
        self.dynamic_map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter('dynamic_map_topic').value),
            1,
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1
        )
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )
        self.create_subscription(
            PoseStamped, self.goal_topic, self.goal_callback, 10
        )
        self.create_subscription(Bool, self.replan_topic, self.replan_callback, 1)
        self.create_subscription(
            PointStamped, 'dynamic_obstacle', self.dynamic_obstacle_callback, 10
        )
        self.create_timer(self.plan_period, self.timer_callback)

        self.get_logger().info('Planner A* listo; esperando /goal_pose.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.map_info = (
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.header.frame_id or 'map',
        )
        self.get_logger().info(
            f'Mapa recibido ({msg.info.width}x{msg.info.height})'
        )
        self.need_plan = True
        self.dynamic_map_dirty = True  # asegura publicar overlay aunque este vacio

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.current_pose = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg
        self.need_plan = True
        self.get_logger().info(
            f'Nueva meta: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )

    def replan_callback(self, msg: Bool) -> None:
        if msg.data:
            self.need_plan = True

    def dynamic_obstacle_callback(self, msg: PointStamped) -> None:
        radius = float(self.get_parameter('dynamic_obstacle_radius').value)
        timeout = float(self.get_parameter('dynamic_obstacle_timeout').value)
        expiry = self.get_clock().now().nanoseconds / 1e9 + timeout
        self.dynamic_obstacles.append((msg.point.x, msg.point.y, expiry))
        # mantengo lista corta
        self._prune_dynamic_obstacles()
        self.dynamic_map_dirty = True
        # forzar replan inmediato al incorporar obstaculo dinamico
        self.need_plan = True
        self.publish_dynamic_map()

    def timer_callback(self) -> None:
        if self.dynamic_map_dirty:
            self.publish_dynamic_map()
        if not self.need_plan:
            return
        if (
            self.map_data is None
            or self.map_info is None
            or self.current_pose is None
            or self.goal_pose is None
        ):
            return

        path = self.plan()
        if path is None:
            self.get_logger().warn('No se pudo generar plan.')
            return
        self.path_pub.publish(path)
        self.need_plan = False
        self.get_logger().info(
            f'Plan publicado con {len(path.poses)} poses.'
        )

    # ------------------------------------------------------------------
    # A*
    # ------------------------------------------------------------------
    def plan(self) -> Optional[Path]:
        assert self.map_info is not None
        width, height, res, ox, oy, frame_id = self.map_info
        start = self.world_to_map(
            self.current_pose.pose.pose.position.x,
            self.current_pose.pose.pose.position.y,
        )
        goal = self.world_to_map(
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y,
        )
        if start is None or goal is None:
            self.get_logger().warn('Start o goal fuera del mapa.')
            return None
        if not self.is_cell_free(start[0], start[1]):
            self.get_logger().warn('Start ocupado.')
            return None
        if not self.is_cell_free(goal[0], goal[1]):
            self.get_logger().warn('Goal ocupado.')
            return None

        max_cells = int(self.max_search_m / res)
        frontier: List[Tuple[float, int]] = []
        start_idx = self.idx(start[0], start[1])
        goal_idx = self.idx(goal[0], goal[1])
        heapq.heappush(frontier, (0.0, start_idx))

        g_cost: Dict[int, float] = {start_idx: 0.0}
        came_from: Dict[int, int] = {}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_idx:
                path_cells = self.reconstruct_path(came_from, current)
                return self.cells_to_path(path_cells, frame_id, res, ox, oy)

            cx = current % width
            cy = current // width
            for nx, ny, step_cost in self.neighbors(cx, cy, width, height):
                if not self.is_cell_free(nx, ny):
                    continue
                n_idx = self.idx(nx, ny)
                tentative = g_cost[current] + step_cost
                if tentative > max_cells:
                    continue
                if tentative < g_cost.get(n_idx, math.inf):
                    g_cost[n_idx] = tentative
                    priority = tentative + self.heuristic(
                        nx, ny, goal[0], goal[1]
                    )
                    heapq.heappush(frontier, (priority, n_idx))
                    came_from[n_idx] = current
        return None

    def neighbors(
        self, x: int, y: int, width: int, height: int
    ) -> List[Tuple[int, int, float]]:
        moves = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
        ]
        if self.connect_8:
            diag = math.sqrt(2.0)
            moves.extend(
                [
                    (1, 1, diag),
                    (1, -1, diag),
                    (-1, 1, diag),
                    (-1, -1, diag),
                ]
            )
        result: List[Tuple[int, int, float]] = []
        for dx, dy, cost in moves:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                result.append((nx, ny, cost))
        return result

    def heuristic(self, x: int, y: int, gx: int, gy: int) -> float:
        return math.hypot(gx - x, gy - y)

    def reconstruct_path(
        self, came_from: Dict[int, int], current: int
    ) -> List[int]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def cells_to_path(
        self,
        cells: List[int],
        frame_id: str,
        res: float,
        ox: float,
        oy: float,
    ) -> Path:
        width = self.map_info[0] if self.map_info else 0
        poses: List[PoseStamped] = []
        yaw_prev = 0.0
        for i, idx in enumerate(cells):
            x_cell = idx % width
            y_cell = idx // width
            wx = ox + (x_cell + 0.5) * res
            wy = oy + (y_cell + 0.5) * res

            yaw = yaw_prev
            if i + 1 < len(cells):
                nx = cells[i + 1] % width
                ny = cells[i + 1] // width
                yaw = math.atan2(
                    oy + (ny + 0.5) * res - wy,
                    ox + (nx + 0.5) * res - wx,
                )
                yaw_prev = yaw
            elif self.goal_pose is not None:
                yaw = yaw_from_quaternion(self.goal_pose.pose.orientation)

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation = quaternion_from_yaw(yaw)
            poses.append(pose)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        path.poses = poses
        return path

    # ------------------------------------------------------------------
    # Utiles
    # ------------------------------------------------------------------
    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.map_info is None:
            return None
        width, height, res, ox, oy, _ = self.map_info
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or mx >= width or my < 0 or my >= height:
            return None
        return mx, my

    def is_cell_free(self, mx: int, my: int) -> bool:
        if self.map_info is None or self.map_data is None:
            return False
        self._prune_dynamic_obstacles()
        width, height, res, _, _, _ = self.map_info
        if mx < 0 or mx >= width or my < 0 or my >= height:
            return False
        idx = self.idx(mx, my)
        cell = self.map_data[idx]
        # Consider unknown (-1) as free to allow plan start/goal in zonas no mapeadas.
        if cell >= self.obstacle_threshold:
            return False
        if self.inflation_radius <= 0.0:
            return True
        rad_cells = int(math.ceil(self.inflation_radius / res))
        if rad_cells <= 0:
            return True
        for dx in range(-rad_cells, rad_cells + 1):
            for dy in range(-rad_cells, rad_cells + 1):
                if dx * dx + dy * dy > rad_cells * rad_cells:
                    continue
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                if self.map_data[self.idx(nx, ny)] >= self.obstacle_threshold:
                    return False
        # Check dynamic obstacles
        if self.dynamic_obstacles:
            radius = float(self.get_parameter('dynamic_obstacle_radius').value)
            cx = self.map_info[3] + (mx + 0.5) * res
            cy = self.map_info[4] + (my + 0.5) * res
            for ox, oy, _ in self.dynamic_obstacles:
                if (cx - ox) ** 2 + (cy - oy) ** 2 <= radius * radius:
                    return False
        return True

    def _prune_dynamic_obstacles(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        new_list = [(x, y, t) for (x, y, t) in self.dynamic_obstacles if t > now_s]
        if len(new_list) != len(self.dynamic_obstacles):
            self.dynamic_map_dirty = True
        self.dynamic_obstacles = new_list

    def publish_dynamic_map(self) -> None:
        if self.map_info is None or self.map_data is None:
            return
        self.dynamic_map_dirty = False
        width, height, res, ox, oy, frame_id = self.map_info
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = frame_id
        grid.info.width = width
        grid.info.height = height
        grid.info.resolution = res
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation = Quaternion(w=1.0)
        data = np.full(width * height, -1, dtype=np.int8)
        if self.dynamic_obstacles:
            radius = float(self.get_parameter('dynamic_obstacle_radius').value)
            rad_cells = max(1, int(math.ceil(radius / res)))
            for oxp, oyp, _ in self.dynamic_obstacles:
                mx = int((oxp - ox) / res)
                my = int((oyp - oy) / res)
                for dx in range(-rad_cells, rad_cells + 1):
                    for dy in range(-rad_cells, rad_cells + 1):
                        if dx * dx + dy * dy > rad_cells * rad_cells:
                            continue
                        cx, cy = mx + dx, my + dy
                        if cx < 0 or cy < 0 or cx >= width or cy >= height:
                            continue
                        data[cy * width + cx] = 100
        grid.data = data.tolist()
        self.dynamic_map_pub.publish(grid)

    def idx(self, x: int, y: int) -> int:
        width = self.map_info[0] if self.map_info else 0
        return y * width + x


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
