"""Rasterize measured camera-space points into a pinhole image, with a nearest-Z buffer."""
import numpy as np


def register_depth(points_optical, K, shape, minimum=.1, maximum=3., measured_depth=None):
    """No truth/depth filling: each output comes from a measured optical-frame XYZ point."""
    points=np.asarray(points_optical).reshape(-1,3)
    valid=np.isfinite(points).all(axis=1) & (points[:,2]>=minimum) & (points[:,2]<=maximum)
    if measured_depth is not None:
        measured=np.asarray(measured_depth).reshape(-1)
        if len(measured)!=len(points):
            raise ValueError("point/depth dimensions differ")
        valid &= np.isfinite(measured) & (measured>=minimum) & (measured<=maximum)
    else:
        measured=points[:,2]
    depths=measured[valid]
    points=points[valid]
    h,w=shape
    out=np.full(h*w,np.inf,np.float32)
    if len(points):
        projected=points @ np.asarray(K).T
        u=np.rint(projected[:,0]/projected[:,2]).astype(np.int64)
        v=np.rint(projected[:,1]/projected[:,2]).astype(np.int64)
        valid=(u>=0)&(u<w)&(v>=0)&(v<h)
        np.minimum.at(out,v[valid]*w+u[valid],depths[valid])
    out[~np.isfinite(out)]=np.nan
    return out.reshape(h,w)
