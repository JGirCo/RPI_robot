from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robotcar_pkg_g00'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi4master',
    maintainer_email='rpi4master@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotcar_node_v1_g01 = robotcar_pkg_g00.robotcar_node_v1_g01:main',
            'bm_controller_g01 = robotcar_pkg_g00.bm_controller_g01:main',
            'serial_talker_g01 = robotcar_pkg_g00.serial_talker_g01:main',
            'laser_node = robotcar_pkg_g00.laser_node:main',
            'FTG_node = robotcar_pkg_g00.FTG_node:main',
            'pidWF_node = robotcar_pkg_g00.pidWF_node:main',
            'pidFTG_node = robotcar_pkg_g00.pidFTG_node:main',
            'stop_node = robotcar_pkg_g00.stop_node:main',
        ],
    },
)
