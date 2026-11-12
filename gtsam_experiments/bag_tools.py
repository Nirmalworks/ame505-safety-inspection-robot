"""Inspect and export ROS bags offline without a ROS installation."""

import argparse
import csv
from contextlib import ExitStack
import json
import math
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def stamp_ns(msg):
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite(values):
    # LaserScan commonly contains +inf for rays with no return; JSON uses null.
    return [float(x) if math.isfinite(float(x)) else None for x in values]


def bag_path(path):
    path = Path(path).resolve()
    if path.suffix == '.db3':
        # Read the entire bag directory, including all split files and metadata.
        path = path.parent
    if not path.exists():
        raise ValueError(f'Bag does not exist: {path}')
    return path


def inspect(reader):
    return {
        'duration_s': reader.duration / 1e9,
        'messages': reader.message_count,
        'topics': [
            {'topic': name, 'type': info.msgtype, 'count': info.msgcount}
            for name, info in sorted(reader.topics.items())
        ],
    }


def export(reader, directory, scan_topic='/scan', imu_topic='/imu'):
    """Preserve raw measurements; do not interpret command odometry as ground truth."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    selected = [c for c in reader.connections if
                (c.topic == scan_topic and c.msgtype == 'sensor_msgs/msg/LaserScan') or
                (c.topic == imu_topic and c.msgtype == 'sensor_msgs/msg/Imu') or
                c.msgtype in ('nav_msgs/msg/Odometry', 'tf2_msgs/msg/TFMessage')]
    if not any(c.topic == scan_topic for c in selected):
        raise ValueError(f'No LaserScan on {scan_topic}; inspect topics and set --scan-topic')
    if not any(c.topic == imu_topic for c in selected):
        raise ValueError(f'No Imu on {imu_topic}; inspect topics and set --imu-topic')
    counts = {'imu': 0, 'scans': 0, 'odometry': 0, 'transforms': 0}
    with ExitStack() as stack:
        def csv_file(name, fields):
            writer = csv.writer(stack.enter_context((directory / name).open('w', newline='')))
            writer.writerow(fields)
            return writer
        imu = csv_file('imu.csv', ('bag_ns', 'stamp_ns', 'frame', 'wx', 'wy', 'wz',
                                  'ax', 'ay', 'az', 'qx', 'qy', 'qz', 'qw',
                                  'orientation_covariance', 'angular_velocity_covariance',
                                  'linear_acceleration_covariance'))
        odom = csv_file('odometry.csv', ('topic', 'bag_ns', 'stamp_ns', 'frame', 'child',
                                       'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                                       'vx', 'vy', 'vz', 'wx', 'wy', 'wz'))
        tf = csv_file('transforms.csv', ('topic', 'bag_ns', 'stamp_ns', 'parent', 'child',
                                       'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
        scans = stack.enter_context((directory / 'scans.jsonl').open('w'))
        for connection, timestamp, raw in reader.messages(connections=selected):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.msgtype == 'sensor_msgs/msg/Imu':
                w, a, q = msg.angular_velocity, msg.linear_acceleration, msg.orientation
                imu.writerow((timestamp, stamp_ns(msg), msg.header.frame_id,
                              w.x, w.y, w.z, a.x, a.y, a.z, q.x, q.y, q.z, q.w,
                              json.dumps(finite(msg.orientation_covariance)),
                              json.dumps(finite(msg.angular_velocity_covariance)),
                              json.dumps(finite(msg.linear_acceleration_covariance))))
                counts['imu'] += 1
            elif connection.msgtype == 'sensor_msgs/msg/LaserScan':
                row = {'bag_ns': timestamp, 'stamp_ns': stamp_ns(msg),
                       'frame': msg.header.frame_id, 'topic': connection.topic}
                for key in ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
                            'scan_time', 'range_min', 'range_max'):
                    value = float(getattr(msg, key))
                    row[key] = value if math.isfinite(value) else None
                row.update(ranges=finite(msg.ranges), intensities=finite(msg.intensities))
                scans.write(json.dumps(row, allow_nan=False) + '\n')
                counts['scans'] += 1
            elif connection.msgtype == 'nav_msgs/msg/Odometry':
                p, q, v, w = (msg.pose.pose.position, msg.pose.pose.orientation,
                              msg.twist.twist.linear, msg.twist.twist.angular)
                odom.writerow((connection.topic, timestamp, stamp_ns(msg),
                               msg.header.frame_id, msg.child_frame_id,
                               p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                               v.x, v.y, v.z, w.x, w.y, w.z))
                counts['odometry'] += 1
            else:
                for tr in msg.transforms:
                    p, q = tr.transform.translation, tr.transform.rotation
                    tf.writerow((connection.topic, timestamp, stamp_ns(tr),
                                 tr.header.frame_id, tr.child_frame_id,
                                 p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                    counts['transforms'] += 1
    summary = inspect(reader)
    summary.update(exported=counts, scan_topic=scan_topic, imu_topic=imu_topic,
                   note='Raw export only. No scan matching, fusion, or ground truth.')
    (directory / 'bag_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('inspect', 'export'))
    parser.add_argument('bag', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--imu-topic', default='/imu')
    args = parser.parse_args()
    if args.action == 'export' and args.out is None:
        parser.error('export requires --out')
    try:
        with AnyReader([bag_path(args.bag)],
                       default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
            summary = (inspect(reader) if args.action == 'inspect' else
                       export(reader, args.out, args.scan_topic, args.imu_topic))
            print(json.dumps(summary, indent=2))
    except (ValueError, OSError) as exc:
        parser.exit(2, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
