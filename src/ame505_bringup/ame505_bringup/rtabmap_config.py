"""RTAB-Map node definitions shared by mapping and navigation launches.

Inputs: lidar /scan, EKF odometry /odom, and the Astra camera via astra_bridge.py
(colour + depth already registered to colour, so rtabmap subscribes to them directly).
Values follow jetauto_toolkit/scripts/rtabmap/rtabmap.yaml, which is tested on this robot.
"""
from launch.conditions import IfCondition
from launch_ros.actions import Node

TOPIC_REMAPS = [
    ('rgb/image', '/depth_cam/rgb/image_raw'),
    ('rgb/camera_info', '/depth_cam/rgb/camera_info'),
    ('depth/image', '/depth_cam/depth/image_raw'),
    ('scan', '/scan'),
]


def base_params(database_path, with_camera: bool, map_3d: bool):
    p = {
        'frame_id': 'base_footprint',
        # Odometry from TF (odom -> base_footprint, published by the EKF) instead of the /odom
        # topic: TF keeps ~10 s of history, so late camera frames still get their exact pose.
        # With the /odom topic in the sync group (30 Hz -> only ~2 s buffered) every late
        # camera frame was dropped -> "Did not receive data" and very few map nodes.
        'odom_frame_id': 'odom',
        'wait_for_transform': 0.5,
        'map_frame_id': 'map',
        'database_path': database_path,
        'subscribe_rgb': with_camera,
        'subscribe_depth': with_camera,
        'subscribe_rgbd': False,
        'subscribe_scan': True,
        'approx_sync': True,              # camera, lidar and odom arrive at different times
        # Camera frames reach ROS 2 late (ROS 1 bridge on a busy Nano: ~1-2 s). With a short
        # queue the matching lidar scan (12 Hz) is already gone -> "Did not receive data".
        # 60 msgs = ~5 s of lidar history, so late camera frames still find their scan.
        # rtabmap_ros 0.21.4 (installed) only knows 'queue_size'; newer versions use the
        # sync_/topic_ names. Set all so it works on either (its log prints "queue_size = N").
        'queue_size': 60,
        'sync_queue_size': 60,
        'topic_queue_size': 30,
        'qos_image': 2,                   # best effort: works with any publisher
        'qos_camera_info': 2,
        'qos_scan': 2,
        'qos_odom': 2,
        'use_action_for_goal': True,
        'publish_tf': True,
        # --- RTAB-Map core params (strings) ---
        'Rtabmap/DetectionRate': '1',     # new nodes per second (Jetson Nano limit)
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/NeighborLinkRefining': 'true',   # lidar scan matching corrects odometry
        'RGBD/ProximityBySpace': 'true',
        'Reg/Strategy': '1',              # lidar ICP
        'Reg/Force3DoF': 'true',          # ground robot: x, y, yaw
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'false',      # 2D lidar
        'Vis/MinInliers': '12',
        'Kp/MaxFeatures': '400',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.2',           # ignore lidar hits on the robot body
        'Optimizer/GravitySigma': '0',
    }
    # 2D navigation grid from the LIDAR only, ray traced (free floor + walls).
    # Grid/Sensor 2 (lidar + depth) was used before: lab_map_new.db ended up with 2617
    # obstacle cells, 1 free cell and phantom obstacles around the start pose, because the
    # ROS 1 camera bridge stamps frames ~1-2 s late, so depth points were placed at the wrong
    # pose. Nav2 still uses the depth camera LIVE (depth_obstacles.py -> costmaps), and the
    # realistic 3D scene comes from frame_recorder + reconstruct_3d.
    p.update({'Grid/Sensor': '0', 'Grid/3D': 'false', 'Grid/RayTracing': 'true',
              'Grid/RangeMax': '6.0', 'Grid/RangeMin': '0.2'})
    return p


def rtabmap_nodes(params, extra_args, rtabmap_viz):
    return [
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[params], remappings=TOPIC_REMAPS, arguments=extra_args),
        Node(package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
             parameters=[params], remappings=TOPIC_REMAPS, condition=IfCondition(rtabmap_viz)),
    ]
