from glob import glob
import os

from setuptools import setup

package_name = 'rgbd_object_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        package_name + '.core',
        package_name + '.nodes',
        package_name + '.utils',
    ],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamil',
    maintainer_email='jamil@example.com',
    description='ROS2 RGB-D object perception using RealSense, YOLO, and 3D localization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_yolo_node = rgbd_object_perception.nodes.image_yolo_node:main',
            'gif_recorder_node = rgbd_object_perception.nodes.gif_recorder_node:main',
            'rgbd_localization_node = rgbd_object_perception.nodes.rgbd_localization_node:main',
            'rgbd_localization_gif_recorder_node = rgbd_object_perception.nodes.rgbd_localization_gif_recorder_node:main',
	    'pointcloud_crop_node = rgbd_object_perception.nodes.pointcloud_crop_node:main',
        ],
    },
)
