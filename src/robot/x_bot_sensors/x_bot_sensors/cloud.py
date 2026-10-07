"""Numerical contracts; usable without ROS or simulation SDKs."""
import math
import struct

def decode_cloud(msg):
    """Validate PointCloud2 layout, endian, padding, finite values and ns offsets."""
    required = {'x': 7, 'y': 7, 'z': 7, 'intensity': 7, 'offset_time': 6, 'line': 2, 'tag': 2}
    fields = {f.name: f for f in msg.fields}
    if msg.header.frame_id != 'mid360_link':
        raise ValueError('Expected mid360_link')
    for name, typ in required.items():
        f = fields.get(name)
        size = 4 if typ in (6, 7) else 1
        if f is None or f.datatype != typ or f.count != 1 or f.offset < 0 or f.offset + size > msg.point_step:
            raise ValueError('Invalid point field: ' + name)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.height * msg.row_step:
        raise ValueError('Invalid cloud dimensions')
    endian = '>' if msg.is_bigendian else '<'
    unpackers = {n: struct.Struct(endian + ('f' if t == 7 else 'I' if t == 6 else 'B')) for n, t in required.items()}
    points = []
    for row in range(msg.height):
        for col in range(msg.width):
            base = row * msg.row_step + col * msg.point_step
            p = {n: u.unpack_from(msg.data, base + fields[n].offset)[0] for n, u in unpackers.items()}
            if not all(math.isfinite(p[n]) for n in ('x', 'y', 'z', 'intensity')):
                continue
            if p['offset_time'] >= 100_000_000 or p['line'] >= 4 or p['tag'] & 0x30 not in (0, 0x10):
                raise ValueError('Invalid MID360 timing/return metadata')
            p['intensity'] = max(0, min(255, round(p['intensity'])))
            points.append(p)
    return sorted(points, key=lambda p: p['offset_time'])
