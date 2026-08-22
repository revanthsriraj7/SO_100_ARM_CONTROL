import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so101_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        # PySide6, pyqtgraph, and the REST API stack are not available as
        # rosdep/apt keys for Humble; install them with pip (see README)
        # before running.
        'PySide6>=6.5',
        'pyqtgraph>=0.13',
        'PyYAML>=6.0',
        'fastapi>=0.100',
        'uvicorn>=0.23',
        'pydantic>=2',
    ],
    zip_safe=True,
    maintainer='Revanth',
    maintainer_email='revanth@example.com',
    description='PySide6 control, telemetry, waypoint, trajectory-recording, and rosbag dashboard for the SO-101/SO-100 arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = so101_dashboard.main:main',
        ],
    },
)
