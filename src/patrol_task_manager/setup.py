import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'patrol_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aswanth-v-b',
    maintainer_email='aswanthvb009@gmail.com',
    description='Patrol state machine and event logging for the patrol robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_manager = patrol_task_manager.patrol_manager:main',
        ],
    },
)
