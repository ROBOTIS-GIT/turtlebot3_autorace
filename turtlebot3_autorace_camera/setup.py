from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'turtlebot3_autorace_camera'

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/calibration/extrinsic_calibration', glob('calibration/extrinsic_calibration/*.yaml')),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='ROS 2 packages for camera calibration and image processing in TurtleBot AutoRace',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_compensation = turtlebot3_autorace_camera.image_compensation:main',
            'image_projection = turtlebot3_autorace_camera.image_projection:main'
        ],
    },
)
