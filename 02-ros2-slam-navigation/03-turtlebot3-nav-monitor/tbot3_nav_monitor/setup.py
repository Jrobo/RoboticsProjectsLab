from setuptools import setup
import os
from glob import glob

package_name = 'tbot3_nav_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamil',
    maintainer_email='jamil@todo.todo',
    description='TurtleBot3 navigation performance monitoring and adaptive behavior package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_monitor_node = tbot3_nav_monitor.nav_monitor_node:main',
            'nav_goal_metrics_node = tbot3_nav_monitor.nav_goal_metrics_node:main',
            'live_dashboard_node = tbot3_nav_monitor.live_dashboard_node:main',
        ],
    },
)
