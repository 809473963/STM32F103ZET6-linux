from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wheeltec_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='luo',
    maintainer_email='luo@todo.todo',
    description='WHEELTEC 履带小车串口桥接节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheeltec_serial   = wheeltec_bridge.serial_node:main',
            'wheeltec_keyboard = wheeltec_bridge.keyboard_node:main',
            'wheeltec_chassis_marker = wheeltec_bridge.chassis_marker_node:main',
        ],
    },
)
