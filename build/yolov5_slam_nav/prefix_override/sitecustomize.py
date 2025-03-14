import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetson/object_detection_route_planning/install/yolov5_slam_nav'
