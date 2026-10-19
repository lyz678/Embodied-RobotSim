"""ROS/Isaac-independent hand-eye geometry and depth perception. T_a_b maps b into a."""
from pathlib import Path
import json
import cv2
import numpy as np

METHOD = 'Daniilidis'


def transform(rotation=None, translation=None):
    out = np.eye(4)
    if rotation is not None:
        out[:3, :3] = rotation
    if translation is not None:
        out[:3, 3] = translation
    return out


def rigid(T):
    T = np.asarray(T, dtype=float)
    return (T.shape == (4, 4) and np.isfinite(T).all()
            and np.allclose(T[3], [0, 0, 0, 1])
            and np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
            and abs(np.linalg.det(T[:3, :3]) - 1) < 1e-5)


def errors(estimated, actual):
    delta = np.linalg.inv(actual) @ estimated
    return {'translation_mm': float(np.linalg.norm(delta[:3, 3]) * 1000),
            'rotation_deg': float(np.degrees(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1))))}


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'count': 0}
    return dict(count=len(a), mean=float(a.mean()), rmse=float(np.sqrt(np.mean(a*a))),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def layout():
    """4x4 AprilGrid: 30 mm tags, 40 mm pitch; x right, y down, centre origin."""
    offsets = np.array([[-1,-1], [1,-1], [1,1], [-1,1]]) * .015
    return {row*4+col: offsets + [(col-1.5)*.04, (row-1.5)*.04]
            for row in range(4) for col in range(4)}


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def detect(image):
    p = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary(), p).detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary(), parameters=p)
    return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.flatten())}


def board_pose(detections, K, D, max_reprojection=2.0):
    target = layout()
    keys = sorted(set(target) & set(detections))
    if not keys:
        raise ValueError('no_target_tags')
    obj = np.concatenate([np.c_[target[k], np.zeros(4)] for k in keys]).astype(np.float64)
    img = np.concatenate([detections[k] for k in keys]).astype(np.float64)
    ok, r, t, inliers = cv2.solvePnPRansac(obj, img, K, D, reprojectionError=max_reprojection,
                                         iterationsCount=100, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inliers is None or len(inliers) < max(4, int(.75*len(obj))):
        raise ValueError('pnp_insufficient_inliers')
    use = inliers.flatten()
    r, t = cv2.solvePnPRefineLM(obj[use], img[use], K, D, r, t)
    projected, _ = cv2.projectPoints(obj[use], r, t, K, D)
    rmse = float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-img[use])**2, axis=1))))
    if t[2, 0] <= 0 or not np.isfinite(rmse) or rmse > max_reprojection:
        raise ValueError('pnp_bad_reprojection_or_cheirality')
    return transform(cv2.Rodrigues(r)[0], t.flatten()), rmse


def solve(samples):
    A = [np.asarray(s['T_base_hand']) for s in samples]
    B = [np.asarray(s['T_camera_target']) for s in samples]
    if len(A) < 6 or not all(rigid(T) for T in A+B):
        raise ValueError('insufficient_or_invalid_samples')
    rotations = np.array([cv2.Rodrigues(A[0][:3,:3].T @ a[:3,:3])[0].flatten() for a in A[1:]])
    if np.linalg.svd(rotations, compute_uv=False)[1] < .1:
        raise ValueError('degenerate_rotation_axes')
    R, t = cv2.calibrateHandEye([a[:3,:3] for a in A], [a[:3,3] for a in A],
                               [b[:3,:3] for b in B], [b[:3,3] for b in B], method=cv2.CALIB_HAND_EYE_DANIILIDIS)
    T = transform(R, t.flatten())
    if not rigid(T):
        raise ValueError('invalid_handeye_solution')
    return T


def fit_depth_grasp(depth, corners, K, max_residual=.004, min_points=40, tcp_backoff=.055):
    """Intersect tag-centre ray with robust top plane. Never uses PnP translation."""
    if depth.ndim != 2 or not np.isfinite(K).all() or min(K[0,0], K[1,1]) <= 0:
        raise ValueError('invalid_depth_geometry')
    corners = np.asarray(corners, dtype=float)
    H = cv2.getPerspectiveTransform(np.array([[0,0],[1,0],[1,1],[0,1]],np.float32),corners.astype(np.float32))
    center = cv2.perspectiveTransform(np.array([[[.5,.5]]],np.float32),H)[0,0]
    polygon = center + .65*(corners-center)  # exclude tag edge / object background
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(polygon).astype(np.int32), 1)
    ys, xs = np.where(mask)
    z = depth[ys, xs]
    valid = np.isfinite(z) & (z > .08) & (z < 3.)
    fraction = float(valid.sum()/max(1, len(z)))
    ys, xs, z = ys[valid], xs[valid], z[valid]
    if len(z) < min_points:
        raise ValueError('insufficient_valid_depth')
    points = np.c_[(xs-K[0,2])*z/K[0,0], (ys-K[1,2])*z/K[1,1], z]
    rng = np.random.default_rng(0)
    best = np.zeros(len(points), bool)
    for _ in range(100):
        triplet = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(triplet[1]-triplet[0], triplet[2]-triplet[0])
        if np.linalg.norm(n) < 1e-10:
            continue
        n /= np.linalg.norm(n)
        keep = np.abs((points-triplet[0]) @ n) < max_residual
        if keep.sum() > best.sum():
            best = keep
    if best.sum() < max(min_points, int(.65*len(points))):
        raise ValueError('depth_plane_outliers')
    centroid = points[best].mean(axis=0)
    _, singular, vh = np.linalg.svd(points[best]-centroid, full_matrices=False)
    if singular[1] < 1e-5:
        raise ValueError('degenerate_depth_plane')
    normal = vh[-1]
    if normal @ centroid > 0:
        normal = -normal  # outward top normal points towards camera
    ray = np.linalg.inv(K) @ [*center, 1.]
    denom = normal @ ray
    if abs(denom) < .2:
        raise ValueError('grazing_depth_plane')
    top = ray * (normal @ centroid) / denom
    if top[2] <= 0 or not np.isfinite(top).all():
        raise ValueError('invalid_plane_intersection')
    # Intersect top-left/top-right rays to recover the closing direction.
    rays = (np.linalg.inv(K) @ np.c_[corners[:2], np.ones(2)].T).T
    edge_points = rays * ((normal @ centroid)/(rays @ normal))[:,None]
    xaxis = edge_points[1]-edge_points[0]
    xaxis /= np.linalg.norm(xaxis)
    # fingers_center: x approach, y closing (see URDF), z completes right-handed frame.
    approach = -normal
    closing = xaxis
    lateral = np.cross(approach, closing)
    lateral /= np.linalg.norm(lateral)
    closing = np.cross(lateral, approach)
    return transform(np.column_stack([approach, closing, lateral]), top + (tcp_backoff-.020)*normal), {
        'valid_fraction': fraction, 'plane_inliers': int(best.sum()),
        'plane_rmse_mm': float(np.sqrt(np.mean(((points[best]-centroid)@normal)**2))*1000)}


def make_assets(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for kind in ('grid', 'cube'):
        extent = .04 if kind == 'cube' else .18
        pixels = 1200 if kind != 'cube' else 400
        image = np.full((pixels, pixels), 255, np.uint8)
        tags = {100: np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*.015} if kind == 'cube' else layout()
        for tag, xy in tags.items():
            lo = np.round((xy.min(axis=0)/extent+.5)*pixels).astype(int)
            hi = np.round((xy.max(axis=0)/extent+.5)*pixels).astype(int)
            n = int(hi[0]-lo[0])
            marker = (cv2.aruco.generateImageMarker(dictionary(), tag, n) if hasattr(cv2.aruco, 'generateImageMarker')
                      else cv2.aruco.drawMarker(dictionary(), tag, n))
            image[lo[1]:hi[1], lo[0]:hi[0]] = marker
        cv2.imwrite(str(directory/f'{kind}.png'), image)
    (directory/'layouts.json').write_text(json.dumps({str(i):v.tolist() for i,v in layout().items()}, indent=2))
