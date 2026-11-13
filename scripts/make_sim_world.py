#!/usr/bin/env python3
"""Generate the simulation floor plan: a 20 m x 9 m hallway with six rooms.

    python3 scripts/make_sim_world.py        # rewrites src/ame505_bringup/sim/

Writes two occupancy images (same size, same origin, 5 cm cells):
  sim_hall.pgm        the SAVED MAP for AMCL / Nav2 (walls + fixed furniture)
  sim_hall_world.pgm  the TRUE WORLD the simulated lidar sees = map + 2 boxes that are
                      NOT on the map (so the costmaps and collision monitor have work to do)
The map frame origin (0,0) is the robot's start, in the hallway 2 m from the west wall.
Replace these with a real map later: copy the robot's maps_2d/<name>.pgm/.yaml here.
"""
import os

import cv2
import numpy as np

RES = 0.05                 # m per cell, same as the real maps
W, H = 20.0, 9.0           # building size (m)
MARGIN = 0.5               # unknown border around the building (m)
HOME = (2.0, 4.5)          # robot start in building coordinates -> map origin
T = 0.10                   # wall thickness (m)
FREE, OCC, UNKNOWN = 254, 0, 205

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'ame505_bringup', 'sim')


def rect(img, x0, y0, x1, y1, value):
    """Fill building-coordinate rectangle (metres, y up) with a cell value."""
    c0, c1 = round((x0 + MARGIN) / RES), round((x1 + MARGIN) / RES)
    r0, r1 = img.shape[0] - round((y1 + MARGIN) / RES), img.shape[0] - round((y0 + MARGIN) / RES)
    img[r0:r1, c0:c1] = value


def build():
    img = np.full((round((H + 2 * MARGIN) / RES), round((W + 2 * MARGIN) / RES)), UNKNOWN, np.uint8)
    rect(img, 0, 0, W, H, FREE)
    for x0, y0, x1, y1 in [(0, 0, W, T), (0, H - T, W, H), (0, 0, T, H), (W - T, 0, W, H)]:
        rect(img, x0, y0, x1, y1, OCC)                      # outer walls
    # hallway y = 3.5 .. 5.5 with 1 m doorways into the rooms
    rect(img, 0, 3.5 - T, W, 3.5, OCC)
    rect(img, 0, 5.5, W, 5.5 + T, OCC)
    for x in (4.0, 9.0, 15.5):
        rect(img, x, 3.5 - T, x + 1.0, 3.5, FREE)           # doors to the south rooms
    for x in (3.0, 9.5, 16.0):
        rect(img, x, 5.5, x + 1.0, 5.5 + T, FREE)           # doors to the north rooms
    for x in (6.5, 13.0):
        rect(img, x, 0, x + T, 3.5, OCC)                    # south room dividers
    for x in (7.0, 13.5):
        rect(img, x, 5.5, x + T, H, OCC)                    # north room dividers
    for box in [(1.0, 7.5, 2.5, 8.3), (9.0, 7.8, 11.0, 8.5), (17.5, 6.5, 18.5, 8.0),   # tables
                (1.0, 0.8, 3.0, 1.6), (8.0, 1.0, 8.8, 2.5), (16.0, 0.6, 18.5, 1.2),
                (12.0, 5.1, 13.0, 5.5), (6.0, 3.5, 6.6, 3.9)]:                          # cabinets in the hallway
        rect(img, *box, OCC)
    world = img.copy()
    for box in [(14.0, 4.9, 14.4, 5.3), (9.8, 2.2, 10.2, 2.6)]:   # boxes that are NOT on the map
        rect(world, *box, OCC)
    return img, world


def main():
    os.makedirs(OUT, exist_ok=True)
    img, world = build()
    origin = [-(HOME[0] + MARGIN), -(HOME[1] + MARGIN), 0]
    for name, data in (('sim_hall', img), ('sim_hall_world', world)):
        cv2.imwrite(os.path.join(OUT, name + '.pgm'), data)
        with open(os.path.join(OUT, name + '.yaml'), 'w') as f:
            f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: {origin}\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'wrote {img.shape[1]}x{img.shape[0]} cells to {os.path.normpath(OUT)}')


if __name__ == '__main__':
    main()
