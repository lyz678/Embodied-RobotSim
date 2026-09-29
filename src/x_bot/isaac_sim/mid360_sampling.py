"""ROS/Isaac-independent MID-360-like sampling and packet assembly.

1000 rays are sampled simultaneously per 5 ms physics step. Offsets describe
those real sample instants, not a fabricated 5 us firing schedule.
"""
import math
import struct

POINT = struct.Struct('<ffffIBBxx')  # xyz, intensity, offset_time(ns), line, tag
PERIOD_NS = 100_000_000


def directions(first, count=1000):
    """Low-discrepancy nonrepeating coverage, NOT Livox's proprietary pattern."""
    for i in range(first, first + count):
        az = 2 * math.pi * ((i * 0.6180339887498949) % 1)
        el = math.radians(-7 + 59 * ((i * 0.4142135623730951) % 1))
        yield (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


class Packet:
    def __init__(self):
        self.reset()

    def reset(self):
        self.start = None
        self.last = None
        self.data = bytearray()

    def add(self, stamp_ns, points):
        if self.last is not None and stamp_ns <= self.last:
            self.reset()
        if self.start is None:
            self.start = stamp_ns
        completed = None
        if stamp_ns - self.start >= PERIOD_NS:
            completed = (self.start, bytes(self.data))
            self.start, self.data = stamp_ns, bytearray()
        self.last = stamp_ns
        for x, y, z, intensity, line in points:
            if all(math.isfinite(v) for v in (x, y, z, intensity)):
                self.data.extend(POINT.pack(x, y, z, intensity, stamp_ns - self.start, line, 0x10))
        return completed
