"""Servicio para guardar el OccupancyGrid publicado en /map a disco."""

from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_srvs.srv import Empty


class MapSaver(Node):
    """Escucha /map y expone el servicio save_map para volcarlo a disco."""

    def __init__(self) -> None:
        super().__init__('map_saver_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_sim_time', False),
                ('map_path', '~/ros2_ws/maps/generated_map'),
            ],
        )
        self.map_path = str(self.get_parameter('map_path').value)
        self._latest_map: Optional[OccupancyGrid] = None

        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_service(Empty, 'save_map', self.handle_save_map)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def handle_save_map(
        self, request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        if self._latest_map is None:
            self.get_logger().warn('Aun no hay mapa para guardar.')
            return response

        base_path = Path(self.map_path).expanduser()
        base_path.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = base_path.with_suffix('.pgm')
        yaml_path = base_path.with_suffix('.yaml')

        self.write_pgm(pgm_path, self._latest_map)
        self.write_yaml(yaml_path, pgm_path.name, self._latest_map)
        self.get_logger().info(
            f'Mapa guardado en {pgm_path} y {yaml_path}'
        )
        return response

    def write_pgm(self, path: Path, map_msg: OccupancyGrid) -> None:
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        lines = ['P2', f'{width} {height}', '255']

        for y in range(height):
            row = []
            for x in range(width):
                value = data[y * width + x]
                if value < 0:
                    cell = 205
                else:
                    cell = int(255 - (value / 100.0) * 255)
                row.append(str(cell))
            lines.append(' '.join(row))
        path.write_text('\n'.join(lines))

    def write_yaml(
        self, path: Path, image_name: str, map_msg: OccupancyGrid
    ) -> None:
        info = map_msg.info
        origin = info.origin
        lines = [
            f'image: {image_name}',
            'mode: trinary',
            f'resolution: {info.resolution}',
            f'origin: [{origin.position.x}, {origin.position.y}, {origin.position.z}]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.196',
        ]
        path.write_text('\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
