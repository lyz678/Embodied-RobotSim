"""Numerical contracts; usable without ROS or simulation SDKs."""
import numpy as np

def matrix(position, quaternion):
    q = np.asarray(quaternion, dtype=float)  # xyzw
    norm = np.linalg.norm(q)
    if norm < 1e-9 or not np.isfinite(norm):
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = position
    if not np.isfinite(result).all():
        raise ValueError('Nonfinite pose')
    return result


def quaternion(rotation):
    # Symmetric eigenproblem is stable at rotations near pi.
    r = rotation
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    _, v = np.linalg.eigh(k)
    q = v[:, -1]
    return q if q[3] >= 0 else -q


def transform(points, pose):
    return np.asarray(points) @ pose[:3, :3].T + pose[:3, 3]
