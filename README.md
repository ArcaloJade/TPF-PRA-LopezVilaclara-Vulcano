# TPF-PRA-LopezVilaclara-Vulcano

## Parte A (SLAM)

Terminal 1:

```
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
```

Terminal 2 (SLAM + mapa):

```
source install/setup.bash
ros2 launch tpf_package slam_launch.py
```

Terminal 3 (teleop para explorar):

```
source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## Parte B (Navegación con PF + A*)

1. Asegurate de tener un mapa guardado (por defecto `~/TPF-PRA-LopezVilaclara-Vulcano/maps/generated_map.yaml` generado con `map_saver_node`).
2. Lanzar la simulación igual que en la Parte A (Terminal 1).
3. Lanzar la navegación:

```
source install/setup.bash
ros2 launch tpf_navigation navigation_launch.py map_yaml:=~/TPF-PRA-LopezVilaclara-Vulcano/maps/generated_map.yaml enable_rviz:=true
```

En RViz: usar `2D Pose Estimate` para la pose inicial y `2D Goal Pose` para fijar el objetivo. El paquete levanta:
- `map_loader_node`: publica `/map` desde el YAML/PGM guardado.
- `localization_node`: filtro de partículas con `/scan` + `calc_odom`.
- `planner_node`: planeo global A* sobre `/map`.
- `controller_node`: seguimiento del `Path` y pedido de replanning ante desvíos/obstáculos.
