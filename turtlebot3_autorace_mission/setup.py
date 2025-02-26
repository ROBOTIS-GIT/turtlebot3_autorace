from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'turtlebot3_autorace_mission'

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/param', glob('param/*.yaml')),
        ('share/' + package_name + '/map', glob('map/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyu',
    maintainer_email='kimhg@robotis.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_construction = turtlebot3_autorace_mission.avoid_construction:main',
            'control_lane = turtlebot3_autorace_mission.control_lane:main',
            'mission_tunnel = turtlebot3_autorace_mission.mission_tunnel:main',
        ],
    },
)
