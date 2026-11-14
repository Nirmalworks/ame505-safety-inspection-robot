from glob import glob
from setuptools import setup

package_name = 'ame505_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nirmal Kumar Jagannathan',
    maintainer_email='njaganna@usc.edu',
    description='AME 505 JetAuto bringup, teleop, mapping and navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'xbox_teleop = ame505_bringup.xbox_teleop:main',
            'checkpoint_recorder = ame505_bringup.checkpoint_recorder:main',
            'patrol = ame505_bringup.patrol:main',
            'save_3d_map = ame505_bringup.save_3d_map:main',
            'dashboard = ame505_bringup.dashboard:main',
            'motor_stop = ame505_bringup.motor_stop:main',
            'reconstruct_3d = ame505_bringup.reconstruct_3d:main',
            'frame_recorder = ame505_bringup.frame_recorder:main',
            'gtsam_slam = ame505_bringup.gtsam_slam:main',
            'gtsam_map_check = ame505_bringup.gtsam_map_check:main',
            'depth_obstacles = ame505_bringup.depth_obstacles:main',
            'capture_dataset = ame505_bringup.capture_dataset:main',
            'lifecycle_guard = ame505_bringup.lifecycle_guard:main',
            'inspection = ame505_bringup.inspection:main',
        ],
    },
)
