from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_autorace_detect'
authors_info = [
    ('Gilbert', 'kkjong@robotis.com'),
    ('Leon Jung', 'N/A'),
    ('Hyungyu Kim', 'kimhg@robotis.com'),
    ('ChanHyeong Lee', 'dddoggi1207@gmail.com'),
    ('Jun', 'junyeong4321@gmail.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='ROS 2 packages for turtlebot3_autorace_detect',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tracker = turtlebot3_autorace_detect.aruco_tracker:main',
        ],
    },
)
