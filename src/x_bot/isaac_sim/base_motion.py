"""Pure, offline-testable four-wheel skid-steer command conditioning.

All wheel axes are +Y in x_bot.xacro. Positive velocities drive forward.
Joint names and their velocities are always submitted together in this order.
"""
import math

WHEEL_NAMES = ('front_left_wheel_joint', 'front_right_wheel_joint',
               'back_left_wheel_joint', 'back_right_wheel_joint')
RADIUS = .06
TRACK = .45


def wheel_velocities(linear, angular):
    left = (linear - angular * TRACK / 2) / RADIUS
    right = (linear + angular * TRACK / 2) / RADIUS
    return [left, right, left, right]


class BaseMotion:
    """Coupled ramp avoids independent axis clipping changing turn curvature.

Zero/invalid/expired commands stop immediately, bypassing the normal ramp.
Intentional pure rotations are supported; this is not a blanket spin filter.
"""
    def __init__(self):
        self.linear = self.angular = 0.

    def reset(self):
        self.linear = self.angular = 0.
        return [0.] * 4

    def advance(self, linear, angular, dt, enabled=True):
        if not enabled or not all(math.isfinite(v) for v in (linear, angular, dt)) or dt < 0:
            return self.reset()
        if linear == 0 and angular == 0:
            return self.reset()
        # Scale both components uniformly at saturation, preserving curvature.
        ratio = max(1., abs(linear) / (.5 if linear >= 0 else .2), abs(angular))
        linear, angular = linear / ratio, angular / ratio
        dv, dw = linear-self.linear, angular-self.angular
        # Cap elapsed time after stalls: don't jump to a large target on resume.
        dt = min(dt, .05)
        fraction = min(1., .5*dt/abs(dv) if dv else 1., 1.*dt/abs(dw) if dw else 1.)
        self.linear += fraction * dv
        self.angular += fraction * dw
        return wheel_velocities(self.linear, self.angular)
