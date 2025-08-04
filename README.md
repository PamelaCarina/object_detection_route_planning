# object_detection_route_planning

READ ME en mantención, agradezco la paciencia! 

Install packages
colcon build --symlink-install
source install/setup.bash

Run nodes

Client UDP
ros2 run yolov5_slam_nav listener_yolov5

Data transformer
ros2 run yolov5_slam_nav map_updater
