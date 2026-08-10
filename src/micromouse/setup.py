import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'micromouse'

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
        (os.path.join('share', package_name, 'description'),
            glob('description/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='mms flood-fill solver mirrored cell-by-cell by a Gazebo robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_maze = micromouse.generate_maze:main',
            'cell_motion_controller = micromouse.cell_motion_controller:main',
            'gazebo_sync_brain = micromouse.gazebo_sync_brain:main',
            'explorer_brain = micromouse.explorer_brain:main',
            'raycast_lidar = micromouse.raycast_lidar:main',
            'brain_viz = micromouse.brain_viz:main',
        ],
    },
)
