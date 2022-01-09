# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import numpy as np
import transforms3d


def distance2(v1, v2):
    return math.sqrt((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2)


def distance3(v1, v2):
    return math.sqrt((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2 + (v1[2] - v2[2]) ** 2)


def rotate_along_z(axis_and_angle):
    q = transforms3d.quaternions.axangle2quat([axis_and_angle[0], axis_and_angle[1], axis_and_angle[2]], axis_and_angle[3])
    rz = [0, 0, 0, 1]
    r = transforms3d.quaternions.qmult(rz, q)
    v, a = transforms3d.quaternions.quat2axangle(r)
    return [v[0], v[1], v[2], a]


def area(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)


def point_in_triangle(m, a, b, c):
    abc = area(a[0], a[1], b[0], b[1], c[0], c[1])
    mbc = area(m[0], m[1], b[0], b[1], c[0], c[1])
    amc = area(a[0], a[1], m[0], m[1], c[0], c[1])
    amb = area(a[0], a[1], b[0], b[1], m[0], m[1])
    if abc == mbc + amc + amb:
        return True
    else:
        return False


def aabb_circle_collision(aabb, x, y, radius):
    if x + radius < aabb[0]:
        return False
    if x - radius > aabb[2]:
        return False
    if y + radius < aabb[1]:
        return False
    if y - radius > aabb[3]:
        return False
    return True


def segment_circle_collision(p1, p2, center, radius):
    len = distance2(p1, p2)
    dx = (p2[0] - p1[0]) / len
    dy = (p2[1] - p1[1]) / len
    t = dx * (center[0] - p1[0]) + dy * (center[1] - p1[1])
    e = [t * dx + p1[0], t * dy + p1[1]]  # projection of circle center onto the (p1, p2) line
    if distance2(e, center) > radius:  # circle is too far away from (p1 p2) line
        return False
    if t >= 0 and t <= len:  # E is on the [p1, p2] segment
        return True
    if distance2(p1, center) < radius:
        return True
    if distance2(p2, center) < radius:
        return True
    return False


def triangle_circle_collision(p1, p2, p3, center, radius):
    if distance2(p1, center) < radius or distance2(p2, center) < radius or distance2(p3, center) < radius:
        return True
    if point_in_triangle(center, p1, p2, p3):
        return True
    return segment_circle_collision(p1, p2, center, radius) \
        or segment_circle_collision(p1, p3, center, radius) \
        or segment_circle_collision(p2, p3, center, radius)


def point_inside_polygon(point, polygon):
    n = len(polygon)
    inside = False
    p2x = 0.0
    p2y = 0.0
    xints = 0.0
    p1x, p1y = polygon[0]
    x, y, _ = point
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def polygon_circle_collision(polygon, center, radius):
    # show_polygon(polygon)  # uncomment this to display convex hull polygons for debugging purposes
    # 1. there is collision if one point of the polygon is inside the circle
    for point in polygon:
        if distance2(point, center) <= radius:
            return True
    # 2. there is collision if one segment of the polygon collide with the circle
    for i in range(len(polygon) - 1):
        if segment_circle_collision(polygon[i], polygon[i+1], center, radius):
            return True
    # 3. there is collision if the circle center is inside the polygon
    if point_inside_polygon(center, polygon):
        return True
    return False


def update_aabb(aabb, position):
    if aabb is None:
        aabb = np.array([position[0], position[1], position[0], position[1]])
    else:
        if position[0] < aabb[0]:
            aabb[0] = position[0]
        elif position[0] > aabb[2]:
            aabb[2] = position[0]
        if position[1] < aabb[1]:
            aabb[1] = position[1]
        elif position[1] > aabb[3]:
            aabb[3] = position[1]
    return aabb
