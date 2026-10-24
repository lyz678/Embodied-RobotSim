"""RGB-D target association and bounded position-based visual servo control."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Observation:
    class_id: int
    wall: float
    stamp: float
    position: np.ndarray
    track_id: str = ''


@dataclass(frozen=True)
class Estimate:
    position: np.ndarray
    wall: float
    stamp: float
    observations: int
    track_id: str


@dataclass(frozen=True)
class ServoConfig:
    standoff: float = .69
    distance_min: float = .62
    distance_max: float = .76
    lateral_tolerance: float = .055
    heading_tolerance: float = .08
    arm_radius: float = .80
    max_forward: float = .12
    max_reverse: float = .06
    max_angular: float = .30
    distance_gain: float = .45
    heading_gain: float = 1.2
    align_before_drive: float = .18
    max_age: float = .35
    observation_window: float = .75
    minimum_observations: int = 3
    cluster_radius: float = .07
    association_radius: float = .25
    timeout: float = 90.
    lost_timeout: float = 8.
    search_timeout: float = 45.
    search_speed: float = .18
    settle_seconds: float = .8
    blocked_timeout: float = 5.


class TargetLock:
    def __init__(self, config=None):
        self.config=config or ServoConfig()
        self.last=None

    def update(self, samples, class_id, wall, sim, base_xy, allowed=lambda p:True):
        c=self.config
        rows=[r for r in samples if r.class_id==class_id and 0<=wall-r.wall<=c.observation_window
              and -.1<=sim-r.stamp<=c.observation_window and np.isfinite(r.position).all() and allowed(r.position)]
        choices=[]
        for seed in rows:
            # A lock survives brief occlusion but never jumps to a different can/book.
            if self.last is not None and np.linalg.norm(seed.position-self.last.position)>c.association_radius:continue
            group=[r for r in rows if (not seed.track_id or not r.track_id or seed.track_id==r.track_id)
                   and np.linalg.norm(r.position-seed.position)<=c.cluster_radius]
            unique={}
            for r in group:
                if r.stamp not in unique or r.track_id:unique[r.stamp]=r
            if len(unique)<c.minimum_observations:continue
            latest=max(unique.values(),key=lambda r:r.stamp)
            if wall-latest.wall>c.max_age or sim-latest.stamp>c.max_age:continue
            position=np.median(np.stack([r.position for r in unique.values()]),axis=0)
            if max(np.linalg.norm(r.position-position) for r in unique.values())>c.cluster_radius:continue
            distance=np.linalg.norm(position-self.last.position) if self.last is not None else math.dist(position[:2],base_xy)
            choices.append((distance,-bool(seed.track_id),Estimate(position,latest.wall,latest.stamp,len(unique),seed.track_id)))
        if not choices:return None
        self.last=min(choices,key=lambda row:row[:2])[2]
        return self.last


@dataclass(frozen=True)
class ServoCommand:
    linear: float
    angular: float
    ready: bool
    distance: float
    heading: float
    arm_distance: float
    visible: bool


def command(target_base, target_arm, image_uv, config=None):
    c=config or ServoConfig();x,y,z=map(float,target_base)
    arm_distance=float(np.linalg.norm(target_arm));distance=math.hypot(x,y);heading=math.atan2(y,x)
    visible=image_uv is not None and all(math.isfinite(v) and .04<=v<=.96 for v in image_uv)
    if not np.isfinite([x,y,z,arm_distance]).all():
        return ServoCommand(0.,0.,False,distance,heading,arm_distance,False)
    ready=(visible and x>0 and c.distance_min<=distance<=c.distance_max
           and abs(y)<=c.lateral_tolerance and abs(heading)<=c.heading_tolerance and arm_distance<=c.arm_radius)
    if ready:return ServoCommand(0.,0.,True,distance,heading,arm_distance,visible)
    angular=float(np.clip(c.heading_gain*heading,-c.max_angular,c.max_angular))
    linear=0.
    if visible and abs(heading)<=c.align_before_drive:
        radial_limit=math.sqrt(max(0.,c.arm_radius**2-float(target_arm[2])**2))
        standoff=min(c.standoff,max(c.distance_min,radial_limit-.015))
        linear=float(np.clip(c.distance_gain*(distance-standoff),-c.max_reverse,c.max_forward))
    return ServoCommand(linear,angular,False,distance,heading,arm_distance,visible)
