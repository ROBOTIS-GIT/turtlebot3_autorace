from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_autorace_camera'
authors_info = [
    ('Gilbert', 'kkjong@robotis.com'),
    ('Leon Jung', 'N/A'),
    ('Hyungyu Kim', 'kimhg@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/calibration/extrinsic_calibration',
            glob('calibration/extrinsic_calibration/*.yaml')),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
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
