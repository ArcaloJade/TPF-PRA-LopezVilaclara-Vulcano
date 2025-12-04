"""Cargador simple de mapas PGM/YAML y publicador de OccupancyGrid."""

from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


DEFAULT_MAP_YAML = str(
    Path.home() / 'TPF-PRA-LopezVilaclara-Vulcano' / 'maps' / 'generated_map.yaml'
)


class MapLoader(Node):
    def __init__(self) -> None:
        super().__init__('map_loader_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_sim_time', False),
                ('map_yaml', DEFAULT_MAP_YAML),
                ('frame_id', 'map'),
                ('publish_rate', 1.0),
            ],
        )

        self.map_yaml_path = Path(
            str(self.get_parameter('map_yaml').value)
        ).expanduser()
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_period = 1.0 / float(
            self.get_parameter('publish_rate').value
        )

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.map_msg: Optional[OccupancyGrid] = None

        self.load_map()
        self.create_timer(publish_period, self.publish_map)

    def load_map(self) -> None:
        if not self.map_yaml_path.exists():
            self.get_logger().error(
                f'No existe el archivo de mapa: {self.map_yaml_path}'
            )
            return
        try:
            data = yaml.safe_load(self.map_yaml_path.read_text())
        except Exception as exc:
            self.get_logger().error(f'Error leyendo YAML del mapa: {exc}')
            return

        image_path = Path(data['image'])
        if not image_path.is_absolute():
            image_path = (self.map_yaml_path.parent / image_path).resolve()
        resolution = float(data.get('resolution', 0.05))
        origin = data.get('origin', [0.0, 0.0, 0.0])

        pgm = self.read_pgm(image_path)
        if pgm is None:
            return
        pixels, width, height = pgm

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.orientation = Quaternion(w=1.0)

        msg.data = pixels
        self.map_msg = msg
        self.get_logger().info(
            f'Mapa cargado desde {image_path} ({width}x{height})'
        )

    def read_pgm(
        self, path: Path
    ) -> Optional[tuple[list[int], int, int]]:
        if not path.exists():
            self.get_logger().error(f'No se encontro el PGM: {path}')
            return None
        try:
            tokens: List[str] = []
            for line in path.read_text().splitlines():
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                tokens.extend(line.split())
        except Exception as exc:
            self.get_logger().error(f'No se pudo leer PGM: {exc}')
            return None
        if not tokens or tokens[0] != 'P2':
            self.get_logger().error('Formato PGM no soportado (se espera P2).')
            return None
        try:
            width = int(tokens[1])
            height = int(tokens[2])
            max_val = float(tokens[3])
            raw_vals = list(map(int, tokens[4:]))
            if len(raw_vals) < width * height:
                raise ValueError('Datos PGM incompletos')
        except Exception as exc:
            self.get_logger().error(f'Error parseando PGM: {exc}')
            return None

        pixels: List[int] = []
        for v in raw_vals[: width * height]:
            if v >= 205:
                pixels.append(-1)
            else:
                occ = int((1.0 - (v / max_val)) * 100)
                pixels.append(occ)
        return pixels, width, height

    def publish_map(self) -> None:
        if self.map_msg is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
