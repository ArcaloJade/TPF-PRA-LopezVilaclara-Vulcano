# TPF-PRA-LopezVilaclara-Vulcano

Para ejecutar el trabajo, correr:

(hacer)


Comandos para correr:

Terminal 1:


colcon build
source install/setup.bash
export export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py


Terminal 2:

source install/setup.bash
ros2 launch tpf_package slam_launch.py rviz_config:=/home/facuvulcano/TPF-PRA-LopezVilaclara-Vulcano/ros2_ws/src/tpf_package/rviz/slam_config.rviz


Terminal 3:

source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
