from setuptools import find_packages, setup

package_name = 'turtlebot3_autorace'

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyu',
    maintainer_email='kimhg@robotis.com',
    description='ROS 2 meta packages for turtlebot3_autorace',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
