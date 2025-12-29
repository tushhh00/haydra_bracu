from setuptools import setup
import os
from glob import glob

package_name = 'yolo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peru0002',
    maintainer_email='user@example.com',
    description='YOLOv11 Object Detection ROS2 Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo.yolo_detector_node:main',
            'yolo_webcam = yolo.yolo_webcam_node:main',
            'object_3d_detector = yolo.object_3d_detector_node:main',
        ],
    },
)
