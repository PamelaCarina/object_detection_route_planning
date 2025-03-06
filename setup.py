from setuptools import setup, find_packages

package_name = 'yolov5_slam_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@example.com',
    description='Paquete de ROS 2 para integración de YOLOv5 con SLAM Toolbox y Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener_yolov5 = yolov5_slam_nav.listener_yolov5:main',
            'map_updater = yolov5_slam_nav.map_updater:main',
        ],
    },
)

