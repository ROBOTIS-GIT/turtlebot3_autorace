from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'turtlebot3_autorace_detect'

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/param/lane', glob('param/lane/*.yaml')),
        ('share/' + package_name + '/param/level', glob('param/level/*.yaml')),
        ('share/' + package_name + '/param/traffic_light', glob('param/traffic_light/*.yaml')),
        ('share/' + package_name + '/image', glob('image/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyu',
    maintainer_email='kimhg@robotis.com',
    description='ROS 2 packages for turtlebot3_autorace_detect',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_construction_sign = turtlebot3_autorace_detect.detect_construction_sign:main',
            'detect_intersection_sign = turtlebot3_autorace_detect.detect_intersection_sign:main',
            'detect_lane = turtlebot3_autorace_detect.detect_lane:main',
            'detect_level_crossing = turtlebot3_autorace_detect.detect_level_crossing:main',
            'detect_level_crossing_sign = turtlebot3_autorace_detect.detect_level_crossing_sign:main',
            'detect_parking_sign = turtlebot3_autorace_detect.detect_parking_sign:main',
            'detect_traffic_light = turtlebot3_autorace_detect.detect_traffic_light:main',
            'detect_tunnel_sign = turtlebot3_autorace_detect.detect_tunnel_sign:main',
        ],
    },
)
