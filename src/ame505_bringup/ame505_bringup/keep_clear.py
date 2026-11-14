"""Non-ML baseline from the proposal: is something standing in the keep-clear zone?

Takes one depth image (16-bit, millimetres, aligned to RGB, as capture_dataset saves it), turns
every pixel into a 3D point in the robot frame (x forward, y left, z up from the floor) and adds
up the frontal area of the points that are inside the zone and above the floor. More area than
`min_area_m2` -> hazard. No ROS imports: used by the inspection node and by ml/depth_baseline.py.

Covers blocked exit and obstructed walkway. It cannot see an open door (nothing is IN the zone).
"""
import numpy as np

# Astra at 320x240 (half of its 640x480 calibration, fx ~570). The inspection node replaces these
# with /depth_cam/rgb/camera_info when it is published. Mount from ame505_description: the camera
# looks straight ahead, 0.029 m in front of base_footprint and 0.207 m above the floor.
DEFAULT_CAMERA = {'fx': 285.0, 'fy': 285.0, 'cx': 160.0, 'cy': 120.0,
                  'x': 0.029, 'height': 0.207, 'pitch_deg': 0.0}
# Zone in front of the robot's taped checkpoint position. 0.8 m wide: OSHA 1910.37 wants exit
# routes >= 0.71 m. Starts at 0.5 m because the camera sees no floor closer than that.
DEFAULT_ZONE = {'x_min': 0.5, 'x_max': 1.5, 'half_width': 0.4, 'z_min': 0.06, 'z_max': 0.60}
DEFAULT_MIN_AREA_M2 = 0.01        # 10 cm x 10 cm of object facing the camera


def points_in_robot_frame(depth_mm, camera):
    """HxW uint16 depth -> (x, y, z, pixel_area_m2) arrays for the valid pixels."""
    z = depth_mm.astype(np.float32) / 1000.0               # distance along the optical axis
    v, u = np.nonzero(z > 0)
    z = z[v, u]
    right = (u - camera['cx']) / camera['fx'] * z
    down = (v - camera['cy']) / camera['fy'] * z
    p = np.radians(camera.get('pitch_deg', 0.0))            # > 0 = camera tilted down
    fwd = z * np.cos(p) - down * np.sin(p)
    drop = z * np.sin(p) + down * np.cos(p)
    area = (z / camera['fx']) * (z / camera['fy'])          # what one pixel covers at that distance
    return camera['x'] + fwd, -right, camera['height'] - drop, area


def check(depth_mm, zone=None, camera=None, min_area_m2=DEFAULT_MIN_AREA_M2):
    """Return {'hazard', 'area_m2', 'points', 'nearest_m', 'valid_pct'} for one depth image."""
    zone = {**DEFAULT_ZONE, **(zone or {})}
    camera = {**DEFAULT_CAMERA, **(camera or {})}
    x, y, z, area = points_in_robot_frame(depth_mm, camera)
    inside = ((x >= zone['x_min']) & (x <= zone['x_max']) & (np.abs(y) <= zone['half_width'])
              & (z >= zone['z_min']) & (z <= zone['z_max']))
    total = float(area[inside].sum())
    return {'hazard': total >= min_area_m2,
            'area_m2': round(total, 4),
            'points': int(inside.sum()),
            'nearest_m': round(float(x[inside].min()), 2) if inside.any() else None,
            'valid_pct': round(100.0 * len(x) / depth_mm.size, 1)}


def load_config(path):
    """Read config/inspection.yaml."""
    import yaml
    with open(path) as f:
        return yaml.safe_load(f) or {}


def check_checkpoint(depth_mm, config, checkpoint, camera=None):
    """check() with the zone for this checkpoint; `camera` (from camera_info) overrides the defaults."""
    zone = {**(config.get('zone') or {}),
            **((config.get('checkpoints') or {}).get(checkpoint) or {}).get('zone', {})}
    cam = {**(camera or {}), **(config.get('camera') or {})}
    return check(depth_mm, zone, cam, config.get('min_area_m2', DEFAULT_MIN_AREA_M2))
