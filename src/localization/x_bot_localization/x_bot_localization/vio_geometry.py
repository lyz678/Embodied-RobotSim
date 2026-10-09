"""Project rectified simulator depth (optical z) without ROS dependencies."""
import numpy as np


def depth_points(depth, k, stride=4, max_range=10.0):
    if depth.ndim != 2 or stride < 1:
        raise ValueError('Expected a depth plane and positive sampling stride')
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        raise ValueError('Invalid camera intrinsics')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > .1) & (z < max_range)
    return np.column_stack(((u[valid]-cx)*z[valid]/fx,
                            (v[valid]-cy)*z[valid]/fy, z[valid]))
