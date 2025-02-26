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
    author='Gilbert, Leon Jung, Hyungyu Kim, ChanHyeong Lee, Jun',
    author_email='kkjong@robotis.com, , kimhg@robotis.com, dddoggi1207@gmail.com, junyeong4321@gmail.com',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='ROS 2 meta packages for turtlebot3_autorace',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
