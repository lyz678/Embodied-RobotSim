"""Measure contact against the visible tabletop, without collider approximations."""
import numpy as np


def visible_surface_height(points, counts, indices, x, y):
    points = np.asarray(points, dtype=float)
    triangles = []
    offset = 0
    for count in counts:
        face = indices[offset:offset+count]
        triangles.extend((face[0], face[i], face[i+1]) for i in range(1, count-1))
        offset += count
    if not triangles:
        return None
    vertices = points[np.asarray(triangles)]
    a, b, c = vertices[:,0], vertices[:,1], vertices[:,2]
    u, v = b-a, c-a
    det = u[:,0]*v[:,1]-v[:,0]*u[:,1]
    usable = np.abs(det) > 1e-12
    alpha = np.zeros(len(det)); beta = np.zeros(len(det))
    alpha[usable] = ((x-a[usable,0])*v[usable,1]-(y-a[usable,1])*v[usable,0])/det[usable]
    beta[usable] = (u[usable,0]*(y-a[usable,1])-u[usable,1]*(x-a[usable,0]))/det[usable]
    inside = usable & (alpha >= -1e-8) & (beta >= -1e-8) & (alpha+beta <= 1+1e-8)
    if not inside.any():
        return None
    heights = a[:,2]+alpha*u[:,2]+beta*v[:,2]
    return float(heights[inside].max())
