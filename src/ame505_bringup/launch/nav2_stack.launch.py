"""Nav2 for the JetAuto with OUR parameters (included by navigation.launch.py).

Replaces Hiwonder's navigation bringup, which hard-codes its own controller file
(no sideways motion, weak obstacle avoidance) and the default behaviour tree (spins on the
spot whenever a plan fails). Everything runs in one component container to save RAM.

    controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
               -> collision_monitor -> /cmd_vel -> motors (odom_publisher)

map_yaml:=<file> also runs 2D localization in the same container: map_server (/map) +
AMCL (lidar particle filter -> TF map -> odom), started first by their own lifecycle manager.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Nav2 resets EVERY server when one misses its 4 s heartbeat. On the loaded Jetson Nano that
# happened every ~1 min ("CRITICAL FAILURE: SERVER planner_server IS DOWN") and any goal in
# flight was lost. All servers share one process here, so a real crash is visible anyway.
BOND = {'bond_timeout': 30.0, 'attempt_respawn_reconnection': True}

LIFECYCLE = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor']


def setup(context):
    me = get_package_share_directory('ame505_bringup')
    params = LaunchConfiguration('params_file').perform(context)
    bt = os.path.join(me, 'config', 'navigate_no_spin.xml')
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    pose = {f'initial_pose.{k}': float(LaunchConfiguration(f'initial_{k}').perform(context))
            for k in ('x', 'y', 'yaw')}

    def comp(pkg, plugin, name, extra_params=None, remaps=()):
        return ComposableNode(package=pkg, plugin=plugin, name=name,
                              parameters=[params] + ([extra_params] if extra_params else []),
                              remappings=tf + list(remaps))

    localization = []
    if map_yaml:
        localization = [
            comp('nav2_map_server', 'nav2_map_server::MapServer', 'map_server',
                 {'yaml_filename': map_yaml}),
            comp('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl', pose),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': False, 'autostart': True,
                             'node_names': ['map_server', 'amcl'], **BOND}]),
        ]

    return [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        ComposableNodeContainer(
            name='nav2_container', namespace='', package='rclcpp_components',
            executable='component_container_isolated', output='screen',
            # The costmaps are sub-nodes created INSIDE the components; they only see
            # parameters given to the container process itself.
            parameters=[params, {'autostart': True}],
            composable_node_descriptions=localization + [
                comp('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server',
                     remaps=[('cmd_vel', 'cmd_vel_nav')]),
                comp('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server'),
                comp('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
                comp('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
                comp('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator',
                     {'default_nav_to_pose_bt_xml': bt}),
                comp('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower',
                     'waypoint_follower'),
                comp('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
                     'velocity_smoother',
                     remaps=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
                comp('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor',
                     'collision_monitor'),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{'use_sim_time': False, 'autostart': True,
                                 'node_names': LIFECYCLE, **BOND}]),
            ]),
    ]


def generate_launch_description():
    me = get_package_share_directory('ame505_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(me, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='2D map -> also run map_server + AMCL'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
