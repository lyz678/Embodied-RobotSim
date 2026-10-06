"""Read-only PhysX ground truth for validating manipulation (never inference input)."""
import json
from pxr import UsdPhysics, UsdGeom, Usd, Gf
from std_msgs.msg import String
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
from robot_importer import find_prim

TARGETS = ('book', 'coffee_mug', 'coke_can', 'water_bottle', 'shoe')

class ManipulationTelemetry:
    def __init__(self, stage, node):
        self.node = node
        self.names, paths = [], []
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI) or not str(prim.GetPath()).startswith('/World/Scene/'):
                continue
            parent = prim
            while parent and parent.GetName() not in TARGETS + ('kitchen_table',):
                parent = parent.GetParent()
            if parent:
                self.names.append(parent.GetName()); paths.append(str(prim.GetPath()))
        for name in ('fr3_leftfinger', 'fr3_rightfinger', 'fr3_hand'):
            prim = find_prim(stage, name)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.names.append(name); paths.append(str(prim.GetPath()))
        if not all(name in self.names for name in TARGETS):
            raise RuntimeError(f'Missing physical grasp objects: {self.names}')
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        self.centers = [cache.ComputeRelativeBound(stage.GetPrimAtPath(path), stage.GetPrimAtPath(path)).ComputeAlignedRange().GetMidpoint() for path in paths]
        bin_prim = next((prim for prim in stage.Traverse() if prim.GetName() == 'trash_bin'), None)
        if bin_prim is None:
            # Migrated model names retain the Gazebo model identifier.
            bin_prim = next((prim for prim in stage.Traverse() if 'trash' in prim.GetName().lower()), None)
        if bin_prim is None:
            raise RuntimeError('Missing physical placement bin')
        bounds = cache.ComputeWorldBound(bin_prim).ComputeAlignedRange()
        self.place_bounds = [list(bounds.GetMin()), list(bounds.GetMax())]
        self.bodies = RigidPrim(paths)
        self.publisher = node.create_publisher(String, '/isaac/debug/object_states', 1)
        self.last = -1.0
        self.callback = SimulationManager.register_callback(self.step, SimulationEvent.PHYSICS_POST_STEP, order=110)
        node.get_logger().info(f'Manipulation ground-truth telemetry: {self.names}')

    def step(self, *args):
        now = SimulationManager.get_simulation_time()
        if now >= self.last and now-self.last < .1:
            return
        self.last = now
        positions, orientations = self.bodies.get_world_poses()
        positions, orientations = positions.numpy(), orientations.numpy()
        linear, angular = self.bodies.get_velocities()
        linear, angular = linear.numpy(), angular.numpy()
        msg = String()
        msg.data = json.dumps({'simulation_time': now, 'frame': 'isaac_world', 'place_bounds': self.place_bounds, 'objects': {
            name: {'position': positions[i].tolist(), 'orientation_wxyz': orientations[i].tolist(), 'linear_velocity': linear[i].tolist(), 'angular_velocity': angular[i].tolist(),
                   'center': list(Gf.Vec3d(*positions[i].tolist()) + Gf.Quatd(float(orientations[i][0]), Gf.Vec3d(*orientations[i][1:].tolist())).Transform(self.centers[i]))}
            for i, name in enumerate(self.names)}})
        self.publisher.publish(msg)

    def close(self):
        SimulationManager.deregister_callback(self.callback)
