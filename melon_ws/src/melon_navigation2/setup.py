from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'melon_navigation2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/map', glob('map/*.yaml') + glob('map/*.png') + glob('map/*.pgm')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSatoya',
    maintainer_email='c0b230860d@edu.teu.ac.jp',
    description='Navigation2 configuration and nodes for the Melon robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'melon_navigation_node = melon_navigation2.melon_navigation_node:main'
        ],
    },
)
