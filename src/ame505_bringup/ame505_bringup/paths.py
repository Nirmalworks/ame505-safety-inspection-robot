"""Shared paths. Large data (RTAB-Map databases, bags, checkpoints) lives on the
SSD bind-mount so it survives container rebuilds. Override with $AME505_DATA."""
import os

DEFAULT_DATA_DIR = '/home/ubuntu/share/tmp/ame505_data'
HIWONDER_SRC = '/home/ubuntu/ros2_ws/src'
_HW_SRC_DIRS = {
    'peripherals': 'peripherals',
    'controller': 'driver/controller',
    'ros_robot_controller': 'driver/ros_robot_controller',
    'servo_controller': 'driver/servo_controller',
    'navigation': 'navigation',
}


def data_dir(sub=''):
    path = os.path.join(os.environ.get('AME505_DATA', DEFAULT_DATA_DIR), sub)
    os.makedirs(path, exist_ok=True)
    return path


def map_db(map_name):
    return os.path.join(data_dir('maps'), map_name + '.db')


def checkpoints_file(map_name):
    return os.path.join(data_dir('checkpoints'), map_name + '.yaml')


def hiwonder_pkg(pkg):
    """Locate a Hiwonder package the same way Hiwonder's own launch files do."""
    if os.environ.get('need_compile', 'False') == 'True':
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    return os.path.join(HIWONDER_SRC, _HW_SRC_DIRS[pkg])


def map_yaml(map_name):
    """2D occupancy map (slam_toolbox / map_saver) used by AMCL navigation."""
    return os.path.join(data_dir('maps_2d'), map_name + '.yaml')
