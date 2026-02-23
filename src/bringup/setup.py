from setuptools import find_packages, setup
import os
from glob import glob # Importa glob

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        (os.path.join('share', package_name, 'config', 'robot_description'), glob(os.path.join('config', 'robot_description', '*'))),
        (os.path.join('share', package_name, 'config', 'ros2_control'), glob(os.path.join('config', 'ros2_control', '*'))),
        (os.path.join('share', package_name, 'config', 'navigation'), glob(os.path.join('config', 'navigation', '*'))),
        (os.path.join('share', package_name, 'config', 'rviz'), glob(os.path.join('config', 'rviz', '*'))),
        (os.path.join('share', package_name, 'config', 'slam'), glob(os.path.join('config', 'slam', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edoardo',
    maintainer_email='edoardo@example.com',
    description='Launch files and configuration for the autonomous differential drive robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)