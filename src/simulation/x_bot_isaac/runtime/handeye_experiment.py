"""Opt-in hand-eye fixture and SingleViewDepthCameraSensor bridge.

All images are read after one render from the SAME render product. Native virtual
stereo distance is registered to RGB using native XYZ rays and its noisy distance.
Truth is segregated on /handeye/debug; normal perception never subscribes to it.
"""
import json
from pathlib import Path
import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdPhysics
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from isaacsim.core.experimental.utils.backend import use_backend
from isaacsim.sensors.experimental.rtx import SingleViewDepthCameraSensor
from robot_importer import find_prim
from scene_builder import add_box
from depth_registration import register_depth


def matrix(prim):
    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T


def texture_plane(stage, path, size, texture, position):
    mesh = UsdGeom.Mesh.Define(stage, path)
    h = size/2
    mesh.CreatePointsAttr([(-h,h,0), (h,h,0), (h,-h,0), (-h,-h,0)])
    mesh.CreateFaceVertexCountsAttr([4]); mesh.CreateFaceVertexIndicesAttr([0,3,2,1])
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    # USD texture origin is bottom-left; source PNG origin is top-left.
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    uv.Set([(0,1),(1,1),(1,0),(0,0)])
    UsdGeom.XformCommonAPI(mesh).SetTranslate(Gf.Vec3d(*position))
    mat = UsdShade.Material.Define(stage, path+'_material')
    shader = UsdShade.Shader.Define(stage, path+'_material/surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(1.)
    reader = UsdShade.Shader.Define(stage, path+'_material/uv')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    tex = UsdShade.Shader.Define(stage, path+'_material/texture')
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh.GetPrim()


class HandeyeRosRuntime:
    """Clock/control graph supplies simulation time; this experiment needs no lidar."""
    error = None

    def __init__(self):
        import rclpy
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = rclpy.create_node('handeye_sensor_bridge')

    def update(self):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.)

    def close(self):
        self.node.destroy_node()


class HandeyeExperiment:
    def __init__(self, stage, node, config_path, assets):
        self.stage, self.node = stage, node
        self.cfg = yaml.safe_load(Path(config_path).read_text())
        # Freeze the mobile chassis so arm motions cannot contaminate base-frame measurements.
        chassis = find_prim(stage, 'base_link')
        anchor = UsdPhysics.FixedJoint.Define(stage, '/World/Handeye/BaseLock')
        anchor.CreateBody1Rel().SetTargets([chassis.GetPath()])
        world_pose = Gf.Transform(UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        anchor.CreateLocalPos0Attr(Gf.Vec3f(world_pose.GetTranslation()))
        anchor.CreateLocalRot0Attr(Gf.Quatf(world_pose.GetRotation().GetQuat()))
        c = self.cfg
        assets = Path(assets)
        add_box(stage, '/World/Handeye/table', [.65,.65,.04], [.65,0,c['scene']['table_top']-.02])
        self.board = texture_plane(stage, '/World/Handeye/grid', .18,
                                   assets/'grid.png', c['scene']['board_center'])
        cube = add_box(stage, '/World/Handeye/cube', [.04]*3,
                       [.65,0,c['scene']['table_top']+.02], dynamic=True, mass=.06)
        # Parent body is scaled .04: child unit square gives a 40 mm face.
        texture_plane(stage, '/World/Handeye/cube/tag', 1., assets/'cube.png', [0,0,.501])
        material = UsdShade.Material.Define(stage, '/World/Handeye/contact')
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(.9); physics.CreateDynamicFrictionAttr(.8)
        UsdShade.MaterialBindingAPI.Apply(cube).Bind(material, materialPurpose='physics')
        self.cube_prim = cube
        self.hand = find_prim(stage, c['frames']['hand'])
        self.base = find_prim(stage, c['frames']['base'])
        self.camera_frame = find_prim(stage, c['frames']['camera'])
        self.tcp = find_prim(stage, c['frames']['tcp'])
        camera = next(p for p in self.camera_frame.GetChildren() if p.GetTypeName() == 'Camera')
        s = c['sensor']; w,h = s['width'],s['height']
        self.sensor = SingleViewDepthCameraSensor(str(camera.GetPath()), resolution=(h,w),
            annotators=['depth_sensor_distance', 'depth_sensor_point_cloud_position', 'distance_to_image_plane'])
        import omni.replicator.core as rep
        self.rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        self.rgb.attach(str(self.sensor.render_product.GetPath()))
        self.camera_params = rep.AnnotatorRegistry.get_annotator('CameraParams')
        self.camera_params.attach(str(self.sensor.render_product.GetPath()))
        self.sensor.set_enabled_post_processing(True)
        self.sensor.set_sensor_baseline(s['baseline_mm'])
        self.sensor.set_sensor_maximum_disparity(s['maximum_disparity'])
        self.sensor.set_sensor_distance_cutoffs(s['minimum_distance_m'], s['maximum_distance_m'])
        self.sensor.set_sensor_disparity_confidence(s['disparity_confidence'])
        self.sensor.set_sensor_noise_parameters(s['noise_mean'], s['noise_sigma'])
        self.sensor.set_sensor_disparity_noise_downscale(s['disparity_noise_downscale'])
        self.sensor.set_enabled_outlier_removal(s['outlier_removal'])
        camera_schema = UsdGeom.Camera(camera)
        f = camera_schema.GetFocalLengthAttr().Get()
        fx = w*f/camera_schema.GetHorizontalApertureAttr().Get()
        fy = h*f/camera_schema.GetVerticalApertureAttr().Get()
        self.sensor.set_sensor_focal_length(fx)
        self.sensor.set_sensor_size(float(w))
        self.sensor.set_sensor_output_mode(0)
        self.K = np.array([[fx,0,w/2],[0,fy,h/2],[0,0,1.]])
        self.publishers = {key:node.create_publisher(Image, topic, 2) for key,topic in {
            'rgb':'/handeye/rgb/image_raw', 'depth':'/handeye/depth/image_raw',
            'ideal':'/handeye/debug/ideal_depth/image_raw', 'raw':'/handeye/debug/raw_sensor_depth/image_raw'}.items()}
        self.info = node.create_publisher(CameraInfo, '/handeye/camera_info', 2)
        self.truth = node.create_publisher(String, '/handeye/debug/truth', 2)
        self.ack = node.create_publisher(String, '/handeye/scene_ack', 2)
        self.subscription = node.create_subscription(String, '/handeye/scene_command', self.command, 2)
        self.pending = []
        self.completed_commands = {}
        self.body = None
        self.last = -1.
        self.mode = 'calibration'
        node.get_logger().info('Handeye: SingleViewDepthCameraSensor enabled, baseline 50 mm; isolated /handeye topics')

    def initialize(self):
        self.body = RigidPrim(str(self.cube_prim.GetPath()))
        self.views = XformPrim([str(p.GetPath()) for p in (self.base,self.hand,self.camera_frame,self.tcp)])

    def command(self, msg):
        self.pending.append(json.loads(msg.data))

    def update(self):
        for command in self.pending:
            if command['id'] in self.completed_commands:
                self.ack.publish(String(data=json.dumps(self.completed_commands[command['id']])))
                continue
            try:
                if command['mode'] == 'calibration':
                    UsdGeom.Imageable(self.board).GetVisibilityAttr().Set('inherited')
                    self.mode = 'calibration'
                    self.body.set_world_poses(positions=np.array([[.65,.4,1.0]]), orientations=np.array([[1.,0,0,0]]))
                else:
                    UsdGeom.Imageable(self.board).GetVisibilityAttr().Set('invisible')
                    self.mode = 'grasp'
                    x,y,yaw = command['trial']
                    self.body.set_world_poses(positions=np.array([[x,y,self.cfg['scene']['table_top']+.02]]),
                        orientations=np.array([[np.cos(yaw/2),0,0,np.sin(yaw/2)]]))
                self.body.set_velocities(np.zeros((1,3)), np.zeros((1,3)))
                response = {'id':command['id'], 'ok':True}
                self.completed_commands[command['id']] = response
                self.ack.publish(String(data=json.dumps(response)))
            except Exception as error:
                self.ack.publish(String(data=json.dumps({'id':command.get('id'), 'ok':False,'error':str(error)})))
        self.pending.clear()
        now = SimulationManager.get_simulation_time()
        if now <= self.last:
            return
        arrays = {}
        for key, ann in [('rgb','rgb'), ('depth','depth_sensor_distance'), ('ideal','distance_to_image_plane'), ('points','depth_sensor_point_cloud_position')]:
            if key == 'rgb':
                data = self.rgb.get_data()
                if data is None or not data.size:
                    return
                arrays[key] = np.asarray(data).copy()
            else:
                data,_ = self.sensor.get_data(ann)
                if data is None:
                    return
                arrays[key] = data.numpy().copy()
        parameters = self.camera_params.get_data()
        if parameters:
            projection = np.asarray(parameters['cameraProjection']).reshape(4,4)
            w,h = self.cfg['sensor']['width'],self.cfg['sensor']['height']
            actual_K = np.array([[abs(projection[0,0])*w/2,0,w/2], [0,abs(projection[1,1])*h/2,h/2], [0,0,1.]])
            if not np.allclose(actual_K, self.K, atol=.05):
                self.node.get_logger().warning(f'Render projection differs from authored intrinsics: {self.K.tolist()} -> {actual_K.tolist()}')
                self.K = actual_K
                self.sensor.set_sensor_focal_length(float(self.K[0,0]))
        # Native distance is rasterized in the virtual stereo view, NOT the RGB grid.
        # Native XYZ supplies the left-view rays in USD camera axes (+X right,+Y up,-Z forward).
        # Reproject those rays into RGB; use the noisy distance AOV as Z, retaining sensor noise.
        points = arrays.pop('points') * np.array([1.,-1.,-1.])
        arrays['raw'] = arrays['depth'].squeeze().copy()
        arrays['depth'] = register_depth(points, self.K, arrays['raw'].shape,
            self.cfg['sensor']['minimum_distance_m'],self.cfg['sensor']['maximum_distance_m'],
            measured_depth=arrays['raw'])
        self.last = now
        ns = round(now*1e9)
        from builtin_interfaces.msg import Time
        stamp = Time(sec=ns//10**9, nanosec=ns%10**9)
        for key, array in arrays.items():
            rgb = key == 'rgb'
            array = np.ascontiguousarray(array[:,:,:3] if rgb else array.squeeze(), dtype=np.uint8 if rgb else np.float32)
            if not rgb:
                array[~np.isfinite(array) | (array <= 0)] = np.nan
            msg = Image()
            msg.header.stamp = stamp; msg.header.frame_id = self.cfg['frames']['camera']
            msg.height, msg.width = array.shape[:2]
            msg.encoding = 'rgb8' if rgb else '32FC1'
            msg.step = msg.width*(3 if rgb else 4); msg.data = array.tobytes()
            self.publishers[key].publish(msg)
        info = CameraInfo()
        info.header.stamp = stamp; info.header.frame_id = self.cfg['frames']['camera']
        info.width = self.cfg['sensor']['width']; info.height = self.cfg['sensor']['height']
        info.distortion_model = 'plumb_bob'; info.d = [0.]*5; info.k = self.K.flatten().tolist()
        info.r = np.eye(3).flatten().tolist(); info.p = np.c_[self.K,np.zeros(3)].flatten().tolist()
        self.info.publish(info)
        with use_backend('fabric'):
            positions, orientations = self.views.get_world_poses()
        poses = []
        for p,q in zip(positions.numpy(), orientations.numpy()):
            m = np.eye(4)
            m[:3,:3] = np.array(Gf.Matrix3d(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))).T
            m[:3,3] = p
            poses.append(m)
        world_base, world_hand, world_camera, world_tcp = poses
        base_world = np.linalg.inv(world_base)
        hand_camera = np.linalg.inv(world_hand) @ world_camera
        p,q = self.body.get_world_poses()
        p,q = p.numpy()[0],q.numpy()[0]
        cube_world = np.eye(4)
        cube_world[:3,:3] = np.array(Gf.Matrix3d(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))).T
        cube_world[:3,3] = p
        board = np.eye(4); board[:3,:3] = np.diag([1,-1,-1]); board[:3,3] = self.cfg['scene']['board_center']
        self.truth.publish(String(data=json.dumps({'simulation_time':now, 'mode':self.mode,
            'T_hand_camera':hand_camera.tolist(), 'T_base_hand':(base_world@world_hand).tolist(), 'T_base_board':(base_world@board).tolist(),
            'T_base_cube':(base_world@cube_world).tolist(), 'T_base_tcp':(base_world@world_tcp).tolist(),
            'sensor':self.cfg['sensor'], 'depth_semantics':'optical_z_meters', 'registration':'native_left_XYZ_rays_with_noisy_distance_zbuffer'})))

    def close(self):
        # Sensor resources are released with SimulationApp; subscriptions belong to lidar node.
        pass
