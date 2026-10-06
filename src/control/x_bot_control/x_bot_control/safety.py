"""Numerical contracts; usable without ROS or simulation SDKs."""
import math

class VelocityArbiter:
    """Manual commands take priority; expired manual commands hold a short stop.

    Both inputs have a wall-time deadman. Localization validity is enforced by
    the caller for every selected command, including manual commands.
    """
    def __init__(self, deadman=.5, manual_hold=2.):
        self.deadman, self.manual_hold = deadman, manual_hold
        self.commands = {'manual': (0., 0.), 'navigation': (0., 0.)}
        self.stamps = {'manual': -math.inf, 'navigation': -math.inf}

    def update(self, source, linear, angular, wall):
        self.commands[source] = ((linear, angular) if
            math.isfinite(linear) and math.isfinite(angular) else (0., 0.))
        self.stamps[source] = wall

    def select(self, wall):
        if 0 <= wall-self.stamps['manual'] < self.manual_hold:
            source = 'manual'
        else:
            source = 'navigation'
        if 0 <= wall-self.stamps[source] < self.deadman:
            return self.commands[source], source
        return (0., 0.), 'manual_stop' if source == 'manual' else 'idle'


class Health:
    """Wall-clock heartbeat AND simulation-age checks; clock rewind latches fault."""
    def __init__(self):
        self.wall = -math.inf
        self.stamp = -math.inf
        self.valid = False
        self.fault = False

    def update(self, valid, stamp, wall):
        if stamp < self.stamp:
            self.fault = True
        self.valid, self.stamp, self.wall = valid, stamp, wall

    def ready(self, sim, wall, wall_timeout=2., sim_timeout=.5):
        return (not self.fault and self.valid and 0 <= sim-self.stamp <= sim_timeout
                and 0 <= wall-self.wall <= wall_timeout)
