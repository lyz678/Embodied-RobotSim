"""Deterministic explicit fetch intents and fresh, stable grasp candidate selection."""
import math
import re
import numpy as np


def explicit_fetch(text, locations, objects):
    rooms='|'.join(map(re.escape,locations));items='|'.join(map(re.escape,sorted(objects,key=len,reverse=True)))
    match=re.fullmatch(rf'\s*(?:🥤\s*)?(?:请|帮我|请帮我)?\s*(?:去|到|前往)?\s*({rooms})\s*(?:拿|取|抓取|拿取)\s*(?:一?[瓶罐个本])?\s*({items})\s*[。！!，,]?\s*',text)
    return match.groups() if match else None


def stable_grasp(candidates, minimum=3):
    """Return an observed cluster medoid, never average different grasp orientations."""
    valid=[]
    for det in candidates:
        p=det.results[0].pose.pose
        xyz=np.array([p.position.x,p.position.y,p.position.z]);q=np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        norm=np.linalg.norm(q)
        if not np.isfinite(np.r_[xyz,q]).all() or norm<.5 or norm>1.5:continue
        valid.append((det,xyz,q/norm))
    if len(valid)<minimum:return None
    clusters=[]
    for d,p,q in valid:
        neighbors=[v for v in valid if v[0].header.frame_id==d.header.frame_id and np.linalg.norm(v[1]-p)<.035 and abs(np.dot(v[2],q))>math.cos(math.radians(20))]
        clusters.append(neighbors)
    largest=max(clusters,key=len)
    if len(largest)<minimum or len(largest)<len(valid)/2:return None
    return min(largest,key=lambda a:sum(np.linalg.norm(a[1]-b[1]) for b in largest))[0]


def approach_axis(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if norm<1e-6:raise ValueError('Invalid grasp quaternion')
    x,y,z,w=(v/norm for v in (x,y,z,w))
    return np.array([1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)])
