"""Numerical contracts; usable without ROS or simulation SDKs."""
import math
import json
import os
import tempfile
from pathlib import Path
import numpy as np

def ray_cells(start, end):
    """Integer Bresenham including endpoints."""
    x, y = start
    ex, ey = end
    dx, dy = abs(ex-x), -abs(ey-y)
    sx, sy = (1 if x < ex else -1), (1 if y < ey else -1)
    err = dx + dy
    while True:
        yield x, y
        if (x, y) == (ex, ey):
            return
        twice = 2 * err
        if twice >= dy:
            err += dy
            x += sx
        if twice <= dx:
            err += dx
            y += sy


class Grid:
    def __init__(self, resolution=.05, min_height=.1, max_height=1.8):
        self.resolution, self.min_height, self.max_height = resolution, min_height, max_height
        self.cells = {}

    def cell(self, point):
        return tuple(math.floor(float(v) / self.resolution) for v in point[:2])

    def insert(self, origin, points):
        start = self.cell(origin)
        free, occupied = set(), set()
        for p in points:
            # Clear only rays passing through the navigation obstacle slab.
            # Ceiling rays cannot establish traversability below a table.
            if not self.min_height <= p[2] <= self.max_height:
                continue
            end = self.cell(p)
            occupied.add(end)
            free.update(list(ray_cells(start, end))[:-1])
        for cell in free - occupied:
            self.cells[cell] = max(-5, self.cells.get(cell, 0) - 1)
        for cell in occupied:
            self.cells[cell] = min(5, self.cells.get(cell, 0) + 3)

    def image(self):
        if not self.cells:
            raise ValueError('No observed occupancy cells')
        xs, ys = zip(*self.cells)
        lower = min(xs), min(ys)
        width, height = max(xs)-lower[0]+1, max(ys)-lower[1]+1
        if width * height > 16_000_000:
            raise ValueError('Map exceeds 16 million cells')
        data = np.full((height, width), -1, dtype=np.int8)
        for (x, y), v in self.cells.items():
            data[y-lower[1], x-lower[0]] = 100 if v > 0 else 0 if v < 0 else -1
        return data, [lower[0]*self.resolution, lower[1]*self.resolution, 0.]


def save_bundle(output, grid, points, initial):
    """Save paired maps atomically; never overwrite an existing bundle."""
    output = Path(output)
    if output.exists():
        raise ValueError('Refusing to overwrite existing bundle: ' + str(output))
    image, origin = grid.image()
    points = list(points)
    if len(points) < 100:
        raise ValueError('Insufficient 3D map points')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.map-', dir=output.parent) as tmp:
        folder = Path(tmp)
        with (folder/'map.pcd').open('w') as stream:
            stream.write(f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
            for p in points:
                stream.write(' '.join(f'{v:.6f}' for v in p)+'\n')
        pixels = np.where(image < 0, 205, np.where(image == 0, 254, 0)).astype('uint8')
        h, w = image.shape
        (folder/'map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode() + pixels[::-1].tobytes())
        (folder/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {grid.resolution}\norigin: {origin}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        (folder/'bundle.json').write_text(json.dumps({'version':1, 'frame':'map', 'pcd':'map.pcd',
            'occupancy':'map.yaml', 'initial_pose':list(initial), 'resolution':grid.resolution,
            'sensor_frame':'mid360_link', 'mapping':'FAST-LIO (no loop closure)',
            'point_count':len(points)}, indent=2)+'\n')
        # A separate reservation prevents a second cooperating saver racing us.
        lock = output.with_name(output.name + '.saving')
        descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        try:
            if output.exists():
                raise ValueError('Bundle appeared while saving; refusing overwrite')
            os.rename(folder, output)
        finally:
            os.close(descriptor)
            lock.unlink()
