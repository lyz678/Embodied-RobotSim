"""Pure, offline-testable four-wheel skid-steer command conditioning.

All wheel axes are +Y in x_bot.xacro. Positive velocities drive forward.
Joint names and their velocities are always submitted together in this order.
"""
import math

WHEEL_NAMES = ('front_left_wheel_joint', 'front_right_wheel_joint',
               'back_left_wheel_joint', 'back_right_wheel_joint')
RADIUS = .06
TRACK = .45
MAX_FORWARD = 2.0
MAX_REVERSE = .2
LINEAR_ACCEL = 1.0
LINEAR_DECEL = 2.0


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
        self.yaw_integral = 0.

    def reset(self):
        self.linear = self.angular = 0.
        self.yaw_integral = 0.
        return [0.] * 4

    def advance(self, linear, angular, dt, enabled=True, measured_angular=None):
        if (not enabled or not all(math.isfinite(v) for v in (linear, angular, dt)) or dt < 0
                or (measured_angular is not None and not math.isfinite(measured_angular))):
            return self.reset()
        if linear == 0 and angular == 0:
            return self.reset()
        # Scale both components uniformly at saturation, preserving curvature.
        ratio = max(1., abs(linear) / (MAX_FORWARD if linear >= 0 else MAX_REVERSE), abs(angular))
        linear, angular = linear / ratio, angular / ratio
        dv, dw = linear-self.linear, angular-self.angular
        # Cap elapsed time after stalls: don't jump to a large target on resume.
        dt = min(dt, .05)
        decelerating = linear * self.linear >= 0 and abs(linear) < abs(self.linear)
        acceleration = LINEAR_DECEL if decelerating else LINEAR_ACCEL
        fraction = min(1., acceleration*dt/abs(dv) if dv else 1., 1.*dt/abs(dw) if dw else 1.)
        self.linear += fraction * dv
        self.angular += fraction * dw
        wheel_angular = self.angular
        if measured_angular is not None:
            # Four driven wheels must skid laterally to turn. Regulate body
            # yaw from IMU feedback instead of assuming ideal differential
            # drive kinematics; otherwise Nav2 substantially understeers.
            error = self.angular - measured_angular
            demand = self.angular + 2.0 * error + self.yaw_integral
            if abs(demand) < 3.5 or demand * error < 0:
                self.yaw_integral = max(-2.5, min(2.5, self.yaw_integral + 2.0 * error * dt))
            wheel_angular = max(-3.5, min(3.5, self.angular + 2.0 * error + self.yaw_integral))
        return wheel_velocities(self.linear, wheel_angular)
