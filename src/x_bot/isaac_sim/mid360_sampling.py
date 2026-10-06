"""ROS/Isaac-independent MID-360-like sampling and packet assembly.

1000 rays are sampled simultaneously per 5 ms physics step. Offsets describe
those real sample instants, not a fabricated 5 us firing schedule.
"""
import math
import struct
import numpy as np

POINT = struct.Struct('<ffffIBBxx')  # xyz, intensity, offset_time(ns), line, tag
PERIOD_NS = 100_000_000
POINT_DTYPE = np.dtype([('x','<f4'), ('y','<f4'), ('z','<f4'),
                       ('intensity','<f4'), ('offset_time','<u4'),
                       ('line','u1'), ('tag','u1'), ('padding','u1',(2,))])


def settled_at_rest(linear, angular, gyro, acceleration, pose_speed):
    """Require stationary poses as well as bounded compliant-solver velocity."""
    vectors = (linear, angular, gyro, acceleration)
    if not math.isfinite(pose_speed) or not all(math.isfinite(v) for row in vectors for v in row):
        return False
    norms = [math.sqrt(sum(v*v for v in row)) for row in vectors]
    return (pose_speed < .01 and norms[0] < .03 and norms[1] < .02
            and norms[2] < .02 and abs(norms[3]-9.81) < .5)


def directions(first, count=1000):
    """Low-discrepancy nonrepeating coverage, NOT Livox's proprietary pattern."""
    for i in range(first, first + count):
        az = 2 * math.pi * ((i * 0.6180339887498949) % 1)
        el = math.radians(-7 + 59 * ((i * 0.4142135623730951) % 1))
        yield (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


def direction_array(first, count=1000):
    """Vectorized equivalent of directions; no repeating scan pattern."""
    indices = np.arange(first, first + count, dtype=np.float64)
    az = 2 * np.pi * ((indices * 0.6180339887498949) % 1)
    el = np.deg2rad(-7 + 59 * ((indices * 0.4142135623730951) % 1))
    return np.column_stack((np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)))


def ray_matches_surface(point, direction, distance, incidence):
    """Bound ray-pattern error and surface error without grazing amplification."""
    point, direction = np.asarray(point), np.asarray(direction)
    if not (np.isfinite(point).all() and np.isfinite(direction).all()
            and math.isfinite(distance) and math.isfinite(incidence)):
        return False
    direction_error = np.linalg.norm(point - direction * np.dot(point, direction))
    surface_error = np.linalg.norm(point - direction * distance) * abs(incidence)
    return direction_error <= .001 and surface_error <= .010


def native_hit_mask(points, paths, robot_prefix='/World/x_bot/'):
    """Validate geometric hits; SDK triangle hits may retain max-range depth."""
    points = np.asarray(points)
    ranges = np.linalg.norm(points, axis=1)
    return (np.isfinite(points).all(axis=1) & (ranges >= .1) & (ranges < 40.0)
            & np.fromiter((bool(p) and not str(p).startswith(robot_prefix) for p in paths), bool, count=len(points)))


def period_for_rate(hz):
    """Frame boundaries must coincide with real 5 ms samples (no fake stamps)."""
    if (isinstance(hz, bool) or not isinstance(hz, (int, float))
            or not math.isfinite(hz) or not 10 <= hz <= 40
            or hz != int(hz) or 200 % int(hz)):
        raise ValueError('MID360 publish_hz must be 10, 20, 25 or 40')
    return 1_000_000_000 // int(hz)


class Packet:
    def __init__(self, period_ns=PERIOD_NS):
        if (not isinstance(period_ns, int) or period_ns <= 0
                or period_ns > PERIOD_NS or period_ns % 5_000_000):
            raise ValueError('Packet period must align with 5 ms samples, at most 100 ms')
        self.period_ns = period_ns
        self.reset()

    def reset(self):
        self.start = None
        self.last = None
        self.data = bytearray()

    def _advance(self, stamp_ns):
        if self.last is not None and stamp_ns <= self.last:
            self.reset()
        if self.start is None:
            self.start = stamp_ns
        completed = None
        if stamp_ns - self.start >= self.period_ns:
            completed = (self.start, bytes(self.data))
            self.start, self.data = stamp_ns, bytearray()
        self.last = stamp_ns
        return completed

    def add(self, stamp_ns, points):
        completed = self._advance(stamp_ns)
        for x, y, z, intensity, line in points:
            if all(math.isfinite(v) for v in (x, y, z, intensity)):
                self.data.extend(POINT.pack(x, y, z, intensity, stamp_ns - self.start, line, 0x10))
        return completed

    def add_arrays(self, stamp_ns, xyz, lines):
        """Pack a batched sensor sample with the same 24-byte ROS wire layout."""
        completed = self._advance(stamp_ns)
        xyz = np.asarray(xyz)
        valid = np.isfinite(xyz).all(axis=1)
        data = np.zeros(int(valid.sum()), dtype=POINT_DTYPE)
        for column, name in enumerate(('x', 'y', 'z')):
            data[name] = xyz[valid, column]
        data['intensity'] = 100.0
        data['offset_time'] = stamp_ns - self.start
        data['line'] = np.asarray(lines)[valid]
        data['tag'] = 0x10
        self.data.extend(data.tobytes())
        return completed
