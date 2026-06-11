from setuptools import setup, find_packages

package_name = 'rgbd_object_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kazi Abdul Jamil',
    maintainer_email='kaziabduljamil.xxx01@universitadipavia.it',
    description='ROS2 RGB-D object detection and 3D localization pipeline for robotic perception.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_yolo_node = rgbd_object_perception.nodes.image_yolo_node:main',
	    'gif_recorder_node = rgbd_object_perception.nodes.gif_recorder_node:main',  
           'rgbd_localization_node = rgbd_object_perception.nodes.rgbd_localization_node:main',
           'rgbd_localization_gif_recorder_node = rgbd_object_perception.nodes.rgbd_localization_gif_recorder_node:main',	   
        ],
    },
)
