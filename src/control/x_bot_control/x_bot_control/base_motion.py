"""Pure, offline-testable four-wheel skid-steer command conditioning.

All wheel axes are +Y in x_bot.xacro. Positive velocities drive forward.
Joint names and their velocities are always submitted together in this order.
"""
import math

WHEEL_NAMES = ('front_left_wheel_joint', 'front_right_wheel_joint',
               'back_left_wheel_joint', 'back_right_wheel_joint')
# One parameter source for both simulation backends; baseline values unchanged.
from pathlib import Path
import os
import yaml

def config_path():
    explicit = os.environ.get('X_BOT_BASE_MOTION_CONFIG')
    if explicit:
        return Path(explicit)
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('x_bot_control')) / 'config/base_motion.yaml'

with config_path().open() as stream:
    _parameters = yaml.safe_load(stream)
for _name, _value in _parameters.items():
    if not math.isfinite(_value) or _value < 0:
        raise ValueError(f'Invalid base motion parameter: {_name}')
    globals()[_name.upper()] = float(_value)
if YAW_INTEGRAL_RELEASE_TIME <= 0 or WHEEL_YAW_ACCEL <= 0:
    raise ValueError('Yaw release time and wheel acceleration must be positive')


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
        self.wheel_angular = 0.

    def reset(self):
        self.linear = self.angular = 0.
        self.yaw_integral = 0.
        self.wheel_angular = 0.
        return [0.] * 4

    def advance(self, linear, angular, dt, enabled=True, measured_angular=None):
        if (not enabled or not all(math.isfinite(v) for v in (linear, angular, dt)) or dt < 0
                or (measured_angular is not None and not math.isfinite(measured_angular))):
            return self.reset()
        if linear == 0 and angular == 0:
            return self.reset()
        # Scale both components uniformly at saturation, preserving curvature.
        ratio = max(1., abs(linear) / (MAX_FORWARD if linear >= 0 else MAX_REVERSE), abs(angular)/MAX_ANGULAR)
        linear, angular = linear / ratio, angular / ratio
        dv, dw = linear-self.linear, angular-self.angular
        # Cap elapsed time after stalls: don't jump to a large target on resume.
        dt = min(dt, .05)
        decelerating = linear * self.linear >= 0 and abs(linear) < abs(self.linear)
        acceleration = LINEAR_DECEL if decelerating else LINEAR_ACCEL
        braking_yaw = angular*self.angular < 0 or abs(angular) < abs(self.angular)
        # MPPI changes its command every cycle. Small reductions are normal
        # tracking corrections, not a reason to discard steady skid compensation.
        # Unload it continuously only for an actual stop, reversal, decisive
        # slowdown, or body overspeed. Emergency zero still resets above.
        unloading = (abs(angular) < .02 or angular*self.angular < 0
                     or abs(angular) < .5*abs(self.angular)
                     or (braking_yaw and measured_angular is not None
                         and measured_angular*self.angular > 0
                         and abs(measured_angular) > abs(self.angular) + .1))
        if unloading:
            self.yaw_integral *= math.exp(-dt/YAW_INTEGRAL_RELEASE_TIME)
        angular_acceleration = ANGULAR_DECEL if braking_yaw else ANGULAR_ACCEL
        fraction = min(1., acceleration*dt/abs(dv) if dv else 1., angular_acceleration*dt/abs(dw) if dw else 1.)
        self.linear += fraction * dv
        self.angular += fraction * dw
        wheel_angular = self.angular
        if measured_angular is not None:
            # Four driven wheels must skid laterally to turn. Regulate body
            # yaw from IMU feedback instead of assuming ideal differential
            # drive kinematics; otherwise Nav2 substantially understeers.
            error = self.angular - measured_angular
            demand = YAW_FEED_FORWARD*self.angular + YAW_KP*error + self.yaw_integral
            if not unloading and abs(angular)>=.02 and (abs(demand) < MAX_WHEEL_YAW_DEMAND or demand*error < 0):
                self.yaw_integral = max(-YAW_INTEGRAL_LIMIT, min(YAW_INTEGRAL_LIMIT,
                    self.yaw_integral + YAW_KI*error*dt))
            wheel_angular = max(-MAX_WHEEL_YAW_DEMAND, min(MAX_WHEEL_YAW_DEMAND,
                YAW_FEED_FORWARD*self.angular + YAW_KP*error + self.yaw_integral))
            # Bound actuator changes too: the body command ramp alone does not
            # bound PI correction jumps from IMU feedback or integral unloading.
            step = WHEEL_YAW_ACCEL*dt
            wheel_angular = max(self.wheel_angular-step,
                                min(self.wheel_angular+step, wheel_angular))
        self.wheel_angular = wheel_angular
        return wheel_velocities(self.linear, wheel_angular)
