"""Physics-step MID-360 approximation. Requires Isaac Sim 6.1, not Livox SDK.

Uses native C++ batched collision queries, not RTX material optics.
"""
import numpy as np
import math
import time
import carb
import omni.graph.core as og
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from isaacsim.core.experimental.prims import RigidPrim
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
from isaacsim.sensors.experimental.physics import IMU, IMUSensor, Raycast, RaycastSensor
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, Vt
from robot_importer import find_prim, ROBOT_PRIM_PATH
from mid360_sampling import Packet, POINT, directions, direction_array, settled_at_rest, ray_matches_surface, native_hit_mask
from base_motion import BaseMotion


def stamp(ns):
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


class Mid360:
    def __init__(self, stage, backend='native'):
        path = str(find_prim(stage, 'mid360_link').GetPath())
        self.body = RigidPrim(path)
        self.imu = IMUSensor(IMU.create(path + '/imu_sensor'))
        self.backend = backend
        self.ray_sensor = None
        self.native_checks = 10
        if backend == 'native':
            self.ray_directions = direction_array(0)
            rays = Raycast.create(path + '/lidar_sensor', min_range=0.001, max_range=39.9,
                                  ray_origins=self.ray_directions * .1,
                                  ray_directions=self.ray_directions,
                                  ray_time_offsets=np.zeros(1000), output_frame='SENSOR',
                                  report_hit_prim_paths=True)
            self.ray_sensor = RaycastSensor(rays)
            self.ray_prim = rays.prims[0]
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init(args=[])
        self.node = rclpy.create_node('isaac_mid360')
        self.command = (0., 0.)
        self.motion = BaseMotion()
        self.measured_angular = 0.
        self.command_sim_time = None
        self.command_time = -math.inf
        self.command_sub = self.node.create_subscription(Twist, '/x_bot/cmd_vel_safe', self.receive_command, 1)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/x_bot/mid360/points', qos_profile_sensor_data)
        # Upstream FAST-LIO uses a reliable IMU subscription (not SensorDataQoS).
        self.imu_pub = self.node.create_publisher(Imu, '/livox/imu', 1000)
        self.query = get_physx_scene_query_interface()
        self.packet = Packet()
        self.index = 0
        self.last_imu = None
        self.ready = False
        self.stable_time = 0.0
        self.start_time = None
        self.previous_position = None
        self.error = None
        self.control_callback = SimulationManager.register_callback(self.control_step, SimulationEvent.PHYSICS_PRE_STEP, order=-100)
        self.callback = SimulationManager.register_callback(self.step, SimulationEvent.PHYSICS_POST_STEP, order=100)
        self.reset_callback = SimulationManager.register_callback(self.reset, SimulationEvent.SIMULATION_STOPPED)

    def reset(self, *args):
        self.motion.reset()
        self.measured_angular = 0.
        self.command_sim_time = None
        self.packet.reset()
        self.index = 0
        self.last_imu = None
        self.ready = False
        self.stable_time = 0.0
        self.start_time = None
        self.previous_position = None
        self.command = (0., 0.)
        self.command_time = -math.inf

    def receive_command(self, msg):
        x, z = msg.linear.x, msg.angular.z
        self.command = (x, z) if math.isfinite(x) and math.isfinite(z) else (0.,0.)
        self.command_time = time.monotonic()

    def update(self):
        playing = app_utils.is_playing()
        if not playing:
            self.reset()
        rclpy.spin_once(self.node, timeout_sec=0.)
        sim_time = SimulationManager.get_simulation_time()
        dt = 0. if self.command_sim_time is None else sim_time-self.command_sim_time
        self.command_sim_time = sim_time
        speeds = self.motion.advance(*self.command, dt,
            enabled=playing and self.ready and time.monotonic()-self.command_time < .5,
            measured_angular=self.measured_angular)
        # One articulation writer applies all four wheel targets atomically.
        og.Controller.set(og.Controller.attribute('/World/ROS2ControlGraph/BaseWheels.inputs:velocityCommand'), speeds)

    def control_step(self, dt, context):
        try:
            self.update()
            if self.ray_sensor is not None:
                self.ray_directions = direction_array(self.index)
                self.ray_prim.GetAttribute('rayDirections').Set(Vt.Vec3fArray.FromNumpy(self.ray_directions.astype(np.float32)))
                self.ray_prim.GetAttribute('rayOrigins').Set(Vt.Vec3fArray.FromNumpy((self.ray_directions * .1).astype(np.float32)))
                # Isaac 6.1 caches ray arrays at sensor creation. Rebind after
                # changing the nonrepeating pattern, before the next step.
                self.ray_sensor.reset()
                self.ray_sensor.get_sensor_reading()
        except Exception as exc:
            self.error = exc

    def step(self, dt, context):
        try:
            self.sample(dt)
        except Exception as exc:
            self.error = exc  # propagate callback errors to main loop (fail closed)

    def sample(self, dt):
        if abs(dt - .005) > 1e-6:
            raise RuntimeError('MID360 requires 200 Hz physics')
        ns = round(SimulationManager.get_simulation_time() * 1e9)
        if self.packet.last is not None and ns <= self.packet.last:
            self.reset()
        reading = self.imu.get_sensor_reading(read_gravity=True)
        if not reading.is_valid:
            return
        imu_data = self.imu.get_data(read_gravity=True)
        self.measured_angular = float(imu_data['angular_velocity'][2])
        positions, orientations = self.body.get_world_poses()
        position = positions.numpy()[0].astype(float)
        if not self.ready:
            if self.start_time is None:
                self.start_time = ns
            linear, angular = self.body.get_velocities()
            pose_speed = (math.inf if self.previous_position is None else
                          float(np.linalg.norm(position-self.previous_position)/dt))
            self.previous_position = position.copy()
            stable = settled_at_rest(linear.numpy()[0], angular.numpy()[0],
                                     imu_data['angular_velocity'], imu_data['linear_acceleration'], pose_speed)
            self.stable_time = self.stable_time + dt if stable else 0.0
            if self.stable_time < .5:
                if ns - self.start_time > 30_000_000_000:
                    raise RuntimeError('Robot did not settle at rest; MID360/FAST-LIO initialization withheld; '
                                       f'position={position.tolist()}, pose_speed={pose_speed:.4f}, '
                                       f'linear={linear.numpy()[0].tolist()}, angular={angular.numpy()[0].tolist()}, '
                                       f'imu_acceleration={np.asarray(imu_data["linear_acceleration"]).tolist()}')
                return
            self.ready = True
            self.node.get_logger().info('Robot settled at rest; starting MID360 lidar/IMU publication')
        if self.ray_sensor is not None:
            reading_rays = self.ray_sensor.get_sensor_reading()
            if not reading_rays.is_valid:
                raise RuntimeError('Native MID360 raycast sensor has no valid physics sample')
            if abs(float(reading_rays.time) - ns / 1e9) > dt / 4:
                raise RuntimeError('Native MID360 raycast timestamp differs from current physics step')
            paths = reading_rays.hit_prim_paths
            # Isaac 6.1 can report max-range depth for an actual triangle-mesh
            # hit while returning the correct hit point/path. Filter on those
            # geometric outputs, including range and self-occlusion checks.
            points = np.asarray(reading_rays.hit_positions)
            valid = native_hit_mask(points, paths)
            xyz = points[valid]
            if self.native_checks:
                self._validate_native_sample(position, orientations, reading_rays, valid)
                self.native_checks -= 1
                if not self.native_checks:
                    self.node.get_logger().info('Native MID360 geometry/pattern verified against reference ray queries')
            lines = (self.index + np.arange(1000))[valid] % 4
            self.index += 1000
            ready = self.packet.add_arrays(ns, xyz, lines)
        else:
            ready = self._sample_python(position, orientations, ns)
        self._publish(ready, reading, imu_data)

    def _validate_native_sample(self, position, orientations, reading, valid):
        """Check changing native patterns at startup against independent queries."""
        q = orientations.numpy()[0].astype(float)
        rotation = Gf.Rotation(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:])))
        points = np.asarray(reading.hit_positions)
        for index in (0, 123, 500, 999):
            local = self.ray_directions[index]
            ray = rotation.TransformDir(Gf.Vec3d(*local))
            start = position + np.asarray(ray) * .1
            hit = self.query.raycast_closest(carb.Float3(*start), carb.Float3(*ray), 39.9)
            expected = hit['hit'] and not any(str(hit.get(k, '')).startswith(ROBOT_PRIM_PATH + '/') for k in ('rigidBody', 'collision'))
            if bool(valid[index]) != bool(expected):
                raise RuntimeError('Native MID360 hit mask differs from reference ray query; '
                                   f'ray={index}, expected={expected}, native_depth={reading.depths[index]}, '
                                   f'native_point={points[index].tolist()}, '
                                   f'reference_distance={hit.get("distance")}, native_hit={reading.hit_prim_paths[index]}, '
                                   f'reference_hit={hit.get("collision", "")}')
            distance = float(hit['distance']) + .1 if expected else 0.0
            # Sensor snapshots and tensor poses can differ by a few mm at
            # compliant contact. Grazing floor rays amplify that into cm of
            # range difference; check the ray direction and surface-normal
            # residual separately, keeping a fixed 10 mm surface tolerance.
            if expected:
                native = points[index]
                normal = np.asarray(tuple(hit['normal']), dtype=float)
                incidence = abs(float(np.dot(np.asarray(ray), normal)))
            if expected and not ray_matches_surface(native, local, distance, incidence):
                raise RuntimeError('Native MID360 geometry differs from reference ray query; '
                                   f'ray={index}, native_point={points[index].tolist()}, '
                                   f'reference_point={(local * distance).tolist()}, '
                                   f'native_hit={reading.hit_prim_paths[index]}, reference_hit={hit.get("collision", "")}')

    def _sample_python(self, position, orientations, ns):
        origin = position
        q = orientations.numpy()[0].astype(float)  # wxyz
        rotation = Gf.Rotation(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:])))
        # RigidPrim tensor poses are current physics poses, not delayed USD poses.
        points = []
        for i, local in enumerate(directions(self.index)):
            ray = rotation.TransformDir(Gf.Vec3d(*local))
            # Start outside the sensor housing; don't discard valid rays because
            # the optical origin lies inside its own box collider.
            start = origin + np.asarray(ray) * .1
            hit = self.query.raycast_closest(carb.Float3(*start), carb.Float3(*ray), 39.9)
            if not hit['hit']:
                continue
            if any(str(hit.get(k, '')).startswith(ROBOT_PRIM_PATH + '/') for k in ('rigidBody', 'collision')):
                continue  # self occlusions are not transparent
            distance = float(hit['distance']) + .1
            if distance < .1 or distance > 40:
                continue
            points.append((*[distance * v for v in local], 100., (self.index + i) % 4))
        self.index += 1000
        return self.packet.add(ns, points)

    def _publish(self, ready, reading, imu_data):
        if ready and ready[1]:
            start_ns, data = ready
            msg = PointCloud2()
            msg.header.stamp, msg.header.frame_id = stamp(start_ns), 'mid360_link'
            msg.height, msg.width = 1, len(data) // POINT.size
            msg.point_step, msg.row_step = POINT.size, len(data)
            msg.is_bigendian, msg.is_dense = False, True
            msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in (
                ('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
                ('offset_time', 16, 6), ('line', 20, 2), ('tag', 21, 2))]
            msg.data = data
            self.cloud_pub.publish(msg)
        if reading.is_valid:
            imu_ns = round(imu_data['time'] * 1e9)
            if imu_ns != self.last_imu:
                self.last_imu = imu_ns
                msg = Imu()
                msg.header.stamp, msg.header.frame_id = stamp(imu_ns), 'mid360_imu_link'
                msg.orientation_covariance[0] = -1.0  # don't leak world orientation
                for attr in ('linear_acceleration', 'angular_velocity'):
                    for axis, value in zip('xyz', imu_data[attr]):
                        setattr(getattr(msg, attr), axis, float(value))
                self.imu_pub.publish(msg)

    def close(self):
        SimulationManager.deregister_callback(self.control_callback)
        SimulationManager.deregister_callback(self.callback)
        SimulationManager.deregister_callback(self.reset_callback)
        if self.ray_sensor is not None:
            self.ray_sensor.reset()
        self.imu.reset()
        self.node.destroy_node()
        if self.owns_context:
            rclpy.shutdown()
