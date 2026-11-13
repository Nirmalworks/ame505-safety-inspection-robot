#!/usr/bin/env python3
"""Dense, photo-coloured 3D reconstruction from an RTAB-Map database.

Every map node stores the camera's colour image + depth image. This back-projects every
pixel into 3D at the node's OPTIMIZED pose (loop-closure corrected), merges all nodes and
voxel-filters the result -> a realistic coloured point cloud (.ply).
No running ROS or rtabmap-export needed: reads the .db directly.

    ros2 run ame505_bringup reconstruct_3d --map lab_map
    ros2 run ame505_bringup reconstruct_3d --map lab_map --voxel 0.01 --max-depth 3.5
-> $AME505_DATA/maps_3d/<map>_dense.ply   coloured points (1 cm)
-> $AME505_DATA/maps_3d/<map>_mesh.ply    coloured SURFACES: each camera view's depth image
                                           triangulated (realistic look; open in the 3D viewer)
"""
import argparse
import math
import os
import sqlite3
import struct
import sys
import zlib

import cv2
import numpy as np

from ame505_bringup.paths import data_dir, map_db
from ame505_bringup.save_3d_map import write_ply


def mat_uncompress(blob):
    """RTAB-Map compressData2: zlib(data) + int32 rows, cols, cv_type at the end."""
    rows, cols, typ = struct.unpack('<3i', blob[-12:])
    dtype = {0: np.uint8, 1: np.int8, 2: np.uint16, 3: np.int16, 4: np.int32,
             5: np.float32, 6: np.float64}[typ & 7]
    return np.frombuffer(zlib.decompress(blob[:-12]), dtype=dtype).reshape(rows, cols)


def to_T(v12):
    T = np.eye(4)
    T[:3, :4] = np.asarray(v12, dtype=np.float64).reshape(3, 4)
    return T


def parse_calibration(blob):
    """CameraModel (RTAB-Map >= 0.18): 11 int header, K D R P (float64), localTransform (12 float32)."""
    h = struct.unpack('<11i', blob[:44])
    w, hgt, nk, nd, nr, npp, nlt = h[4], h[5], h[6], h[7], h[8], h[9], h[10]
    off = 44
    K = np.frombuffer(blob, np.float64, nk, off).reshape(3, 3)
    off += 8 * (nk + nd + nr + npp)
    lt = to_T(np.frombuffer(blob, np.float32, nlt, off)) if nlt == 12 else np.eye(4)
    return w, hgt, K, lt


def load_poses(con):
    """Optimized poses saved by RTAB-Map on close; fall back to odometry poses."""
    poses = {}
    row = con.execute('select opt_ids, opt_poses from Admin').fetchone()
    if row and row[0] and row[1]:
        ids = mat_uncompress(row[0]).ravel()
        P = mat_uncompress(row[1]).reshape(-1, 12)
        poses = {int(i): to_T(p) for i, p in zip(ids, P)}
    src = 'optimized' if poses else 'odometry'
    for nid, blob in con.execute('select id, pose from Node'):
        if nid not in poses and blob and len(blob) >= 48:
            poses[nid] = to_T(np.frombuffer(blob, np.float32, 12))
    return poses, src


def _view_mesh(z, rgb, fx, fy, cx, cy, T, min_depth, max_depth, max_z, step):
    """Triangulate one depth image: neighbouring pixels -> 2 triangles per cell, skipping
    cells that span a depth jump (object edges) so separate objects are not glued together."""
    zs, cs = z[::step, ::step], rgb[::step, ::step]
    H, W = zs.shape
    v, u = np.mgrid[0:H, 0:W]
    u, v = u * step, v * step
    pc = np.stack([(u - cx) / fx * zs, (v - cy) / fy * zs, zs, np.ones_like(zs)]).reshape(4, -1)
    pw = (T @ pc)[:3].T.reshape(H, W, 3)
    valid = (zs > min_depth) & (zs < max_depth) & (pw[..., 2] < max_z)
    idx = -np.ones((H, W), np.int64)
    idx[valid] = np.arange(valid.sum())
    a, b, c, d = idx[:-1, :-1], idx[:-1, 1:], idx[1:, :-1], idx[1:, 1:]
    za, zb, zc, zd = zs[:-1, :-1], zs[:-1, 1:], zs[1:, :-1], zs[1:, 1:]
    tol = 0.04 + 0.04 * za                                   # depth-jump tolerance grows with range
    t1 = (a >= 0) & (b >= 0) & (c >= 0) & (abs(za - zb) < tol) & (abs(za - zc) < tol) & (abs(zb - zc) < tol)
    t2 = (b >= 0) & (d >= 0) & (c >= 0) & (abs(zb - zd) < tol) & (abs(zc - zd) < tol) & (abs(zb - zc) < tol)
    faces = np.concatenate([np.stack([a[t1], c[t1], b[t1]], 1), np.stack([b[t2], c[t2], d[t2]], 1)])
    return pw[valid].astype(np.float32), cs[valid][:, ::-1].copy(), faces


def write_mesh_ply(path, verts, cols, faces):
    n, m = len(verts), len(faces)
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\n'
              f'element face {m}\nproperty list uchar int vertex_indices\nend_header\n')
    vd = np.empty(n, [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    vd['x'], vd['y'], vd['z'] = verts[:, 0], verts[:, 1], verts[:, 2]
    vd['r'], vd['g'], vd['b'] = cols[:, 0], cols[:, 1], cols[:, 2]
    fd = np.empty(m, [('n', 'u1'), ('i', '<i4', (3,))])
    fd['n'], fd['i'] = 3, faces
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(vd.tobytes())
        f.write(fd.tobytes())


def merge_meshes(views):
    vs, cs, fs, off = [], [], [], 0
    for v, c, f in views:
        vs.append(v); cs.append(c); fs.append(f + off); off += len(v)
    return np.concatenate(vs), np.concatenate(cs), np.concatenate(fs).astype(np.int32)


def reconstruct(db, voxel, max_depth, min_depth, decimate, max_z, mesh_views=None, mesh_step=2):
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    poses, src = load_poses(con)
    pts_all, col_all, used = [], [], 0
    for nid, img_b, dep_b, cal_b in con.execute(
            'select id, image, depth, calibration from Data where image is not null and depth is not null'):
        if nid not in poses or not cal_b:
            continue
        rgb = cv2.imdecode(np.frombuffer(img_b, np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(np.frombuffer(dep_b, np.uint8), cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None:
            continue
        w, h, K, local = parse_calibration(cal_b)
        if depth.dtype == np.float32:
            z = depth.astype(np.float32)
        else:
            z = depth.astype(np.float32) / 1000.0           # 16UC1 millimetres
        if rgb.shape[:2] != z.shape[:2]:
            rgb = cv2.resize(rgb, (z.shape[1], z.shape[0]))
        sx, sy = z.shape[1] / w, z.shape[0] / h
        fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sy, K[0, 2] * sx, K[1, 2] * sy
        if mesh_views is not None:
            mesh_views.append(_view_mesh(z, rgb, fx, fy, cx, cy, poses[nid] @ local,
                                         min_depth, max_depth, max_z, mesh_step))
        v, u = np.mgrid[0:z.shape[0]:decimate, 0:z.shape[1]:decimate]
        zz = z[v, u]
        ok = (zz > min_depth) & (zz < max_depth)
        u, v, zz = u[ok], v[ok], zz[ok]
        pc = np.stack([(u - cx) / fx * zz, (v - cy) / fy * zz, zz, np.ones_like(zz)])
        pw = (poses[nid] @ local @ pc)[:3].T                  # map <- base <- camera optical
        c = rgb[v, u][:, ::-1]                                # BGR -> RGB
        keep = pw[:, 2] < max_z
        pts_all.append(pw[keep].astype(np.float32))
        col_all.append(c[keep])
        used += 1
    if not pts_all:
        return None, None, used, src
    pts, cols = voxel_filter(np.concatenate(pts_all), np.concatenate(col_all), voxel)
    return pts, cols, used, src


def voxel_filter(pts, cols, voxel):
    """One point per voxel (mean position + colour)."""
    if voxel <= 0:
        return pts, cols
    key = np.floor(pts / voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    P = np.zeros((cnt.size, 3)); C = np.zeros((cnt.size, 3))
    np.add.at(P, inv, pts); np.add.at(C, inv, cols.astype(np.float64))
    return (P / cnt[:, None]).astype(np.float32), np.clip(C / cnt[:, None], 0, 255).astype(np.uint8)


def node_odom_poses(con):
    """RTAB-Map Node table: id -> (stamp, odometry pose)."""
    out = {}
    for nid, stamp, blob in con.execute('select id, stamp, pose from Node'):
        if blob and len(blob) >= 48:
            out[nid] = (stamp, to_T(np.frombuffer(blob, np.float32, 12)))
    return out


def reconstruct_frames(db, fdir, voxel, max_depth, min_depth, max_z, mesh_views, mesh_step,
                       key_dist=0.25, key_angle=0.21, max_frames=4000):
    """Use ALL frames saved by frame_recorder. Each frame's odometry pose is corrected with the
    nearest RTAB-Map node:  T_map = T_opt(node) * inv(T_odom(node)) * T_odom(frame)."""
    import csv
    import yaml
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    opt, src = load_poses(con)
    nodes = node_odom_poses(con)
    nids = [n for n in nodes if n in opt]
    if not nids:
        return None, None, 0, 'none'
    n_st = np.array([nodes[n][0] for n in nids])
    corr = {n: opt[n] @ np.linalg.inv(nodes[n][1]) for n in nids}
    cam = yaml.safe_load(open(os.path.join(fdir, 'camera.yaml')))
    fx, fy, cx, cy = cam['fx'], cam['fy'], cam['cx'], cam['cy']
    b2c = to_T(cam['base_to_camera'])
    rows = list(csv.DictReader(open(os.path.join(fdir, 'frames.csv'))))[:max_frames]
    pts_all, col_all, used, keyposes = [], [], 0, []
    for r in rows:
        st = float(r['stamp'])
        n = nids[int(np.argmin(abs(n_st - st)))]
        T = corr[n] @ to_T([float(r[f'o{i}']) for i in range(12)]) @ b2c
        i = int(r['idx'])
        rgb = cv2.imread(os.path.join(fdir, 'rgb', f'{i:06d}.jpg'))
        dep = cv2.imread(os.path.join(fdir, 'depth', f'{i:06d}.png'), cv2.IMREAD_UNCHANGED)
        if rgb is None or dep is None:
            continue
        z = dep.astype(np.float32) / 1000.0
        # points from every frame (every 2nd pixel keeps memory sane on the Nano)
        v, u = np.mgrid[0:z.shape[0]:2, 0:z.shape[1]:2]
        zz = z[v, u]
        ok = (zz > min_depth) & (zz < max_depth)
        u, v, zz = u[ok], v[ok], zz[ok]
        pw = (T @ np.stack([(u - cx) / fx * zz, (v - cy) / fy * zz, zz, np.ones_like(zz)]))[:3].T
        keep = pw[:, 2] < max_z
        pts_all.append(pw[keep].astype(np.float32))
        col_all.append(rgb[v, u][:, ::-1][keep])
        used += 1
        if len(pts_all) >= 40:                                 # fold in periodically: bounded memory
            p, c = voxel_filter(np.concatenate(pts_all), np.concatenate(col_all), voxel)
            pts_all, col_all = [p], [c]
        # mesh only "key" views (new viewpoint), otherwise the mesh repeats the same wall 50x
        if mesh_views is not None:
            yaw = math.atan2(T[1, 0], T[0, 0])
            if all(np.linalg.norm(T[:2, 3] - kp[:2]) > key_dist or
                   abs((yaw - kp[2] + math.pi) % (2 * math.pi) - math.pi) > key_angle for kp in keyposes):
                keyposes.append((T[0, 3], T[1, 3], yaw))
                mesh_views.append(_view_mesh(z, rgb, fx, fy, cx, cy, T, min_depth, max_depth, max_z, mesh_step))
    if not pts_all:
        return None, None, used, src
    pts, cols = voxel_filter(np.concatenate(pts_all), np.concatenate(col_all), voxel)
    return pts, cols, used, f'{src} + {len(rows)} recorded frames ({len(keyposes)} key views meshed)'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='lab_map')
    ap.add_argument('--db', default='', help='explicit .db path (overrides --map)')
    ap.add_argument('--voxel', type=float, default=0.01, help='m; 0 = keep every point')
    ap.add_argument('--max-depth', type=float, default=4.0, help='m; Astra is noisy beyond ~4 m')
    ap.add_argument('--min-depth', type=float, default=0.3)
    ap.add_argument('--decimate', type=int, default=1, help='use every Nth pixel')
    ap.add_argument('--max-z', type=float, default=2.2, help='m; drop points above (ceiling)')
    ap.add_argument('--no-mesh', action='store_true', help='skip the surface mesh')
    ap.add_argument('--mesh-step', type=int, default=1, help='pixel step for the mesh (1 = finest)')
    ap.add_argument('--nodes-only', action='store_true',
                    help='ignore recorded frames; use only RTAB-Map nodes')
    args = ap.parse_args([a for a in sys.argv[1:] if not a.startswith('--ros-args')])

    db = args.db or map_db(args.map)
    if not os.path.exists(db):
        sys.exit(f'No database {db}')
    views = None if args.no_mesh else []
    name = os.path.splitext(os.path.basename(db))[0]
    fdir = os.path.join(data_dir('frames'), name)
    if not args.nodes_only and os.path.exists(os.path.join(fdir, 'frames.csv')) \
            and os.path.exists(os.path.join(fdir, 'camera.yaml')):
        pts, cols, used, src = reconstruct_frames(db, fdir, args.voxel, args.max_depth, args.min_depth,
                                                  args.max_z, views, args.mesh_step)
    else:
        pts, cols, used, src = reconstruct(db, args.voxel, args.max_depth, args.min_depth,
                                           args.decimate, args.max_z, views, args.mesh_step)
    if pts is None:
        sys.exit('No usable RGB-D nodes in the database (was the camera running while mapping?)')
    out = os.path.join(data_dir('maps_3d'), f'{name}_dense.ply')
    write_ply(out, pts, cols)
    print(f'{used} camera views, {src} poses -> {len(pts):,} points -> {out}')
    if views:
        v, c, f = merge_meshes(views)
        mout = os.path.join(data_dir('maps_3d'), f'{name}_mesh.ply')
        write_mesh_ply(mout, v, c, f)
        print(f'mesh: {len(v):,} vertices, {len(f):,} triangles -> {mout}')


if __name__ == '__main__':
    main()
