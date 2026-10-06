"""Conservative separating-contact geometry, independent of ROS."""
import math


def separating_contacts(ranges, angle_min, increment, range_min, range_max,
                        linear, angular, radius=.435, distance=.3):
    """Return only front overlap ray indices safe to ignore during straight retreat.

    Every other return must clear the complete reverse swept disk. Existing
    side/rear contacts, sparse rear visibility, spins and faster commands fail
    closed. Original scans and planning obstacles are never modified.
    """
    if (not all(math.isfinite(v) for v in
                (angle_min, increment, range_min, range_max, linear, angular, radius, distance))
            or increment <= 0 or radius <= 0 or distance <= 0
            or not -.1-1e-6 <= linear < -.001 or abs(angular) > 1e-6):
        return []
    contacts, rear = [], []
    for i, value in enumerate(ranges):
        if not math.isfinite(value) or not range_min <= value <= range_max:
            continue
        angle = angle_min + i*increment
        x, y = value*math.cos(angle), value*math.sin(angle)
        rear_angle = math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi))
        if abs(rear_angle) <= .6:
            rear.append(rear_angle)
        if x*x+y*y < radius*radius:
            if x <= .05:
                return []
            contacts.append(i)
        elif (x+max(0., min(distance, -x)))**2+y*y < radius*radius:
            return []
    rear.sort()
    if (len(rear) < 10 or not rear or rear[0] > -.4 or rear[-1] < .4
            # MID360's height-projected scan is sparse: require both rear
            # sides and reject a missing half-cone, rather than a dense 2D ring.
            or max((b-a for a, b in zip(rear, rear[1:])), default=math.inf) > .6):
        return []
    return contacts
