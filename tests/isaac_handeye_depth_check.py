#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import json
parser=argparse.ArgumentParser(description='Isaac oblique-plane check: native sensor distance must be registered to RGB, preserving noise.')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/simulation/x_bot_isaac/runtime'))
from depth_registration import register_depth
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
import isaacsim.core.experimental.utils.app as au
au.enable_extension('isaacsim.sensors.experimental.rtx')
import isaacsim.core.experimental.utils.stage as su
from isaacsim.sensors.experimental.rtx import SingleViewDepthCameraSensor
from pxr import UsdGeom, UsdLux, Gf, UsdShade, Sdf
import numpy as np
import cv2
y,x=np.indices((1200,1200));checker=(((x//75+y//75)%2)*255).astype(np.uint8)
cv2.imwrite(str(args.output/'checker.png'),checker)
import omni.timeline
stage=su.create_new_stage()
UsdGeom.SetStageMetersPerUnit(stage,1.)
cam=UsdGeom.Camera.Define(stage,'/World/Camera'); cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(20.955);cam.CreateFocalLengthAttr(11.041)
plane=UsdGeom.Mesh.Define(stage,'/World/plane')
plane.CreatePointsAttr([(-.5,.5,0),(.5,.5,0),(.5,-.5,0),(-.5,-.5,0)])
plane.CreateFaceVertexCountsAttr([4]);plane.CreateFaceVertexIndicesAttr([0,3,2,1]);plane.CreateSubdivisionSchemeAttr('none');plane.CreateDoubleSidedAttr(True)
UsdGeom.PrimvarsAPI(plane).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(0,1),(1,1),(1,0),(0,0)])
mat=UsdShade.Material.Define(stage,'/World/Mat');sh=UsdShade.Shader.Define(stage,'/World/Mat/surface');sh.CreateIdAttr('UsdPreviewSurface')
uv=UsdShade.Shader.Define(stage,'/World/Mat/uv');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
tex=UsdShade.Shader.Define(stage,'/World/Mat/texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(args.output/'checker.png')));tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(mat)
x=UsdGeom.XformCommonAPI(plane);x.SetTranslate(Gf.Vec3d(0,0,-1));x.SetRotate((0.,30.,0.));x.SetScale(Gf.Vec3f(2,2,.01))
UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1000.)
sensor=SingleViewDepthCameraSensor('/World/Camera',resolution=(640,640),annotators=['depth_sensor_distance','depth_sensor_point_cloud_position','distance_to_image_plane'])
sensor.set_enabled_post_processing(True);sensor.set_sensor_baseline(50.);sensor.set_sensor_focal_length(640*11.041/20.955);sensor.set_sensor_size(640.);sensor.set_sensor_disparity_confidence(.99);sensor.set_sensor_noise_parameters(.25,.25);sensor.set_sensor_distance_cutoffs(.1,3.)
omni.timeline.get_timeline_interface().play()
for i in range(35): app.update()
arrays={}
for a in ['depth_sensor_distance','depth_sensor_point_cloud_position','distance_to_image_plane']:
 data,info=sensor.get_data(a); arrays[a]=data.numpy();print(a,arrays[a].shape,arrays[a][320,320])
depth=arrays['depth_sensor_distance'].squeeze()
ideal=arrays['distance_to_image_plane'].squeeze()
points=arrays['depth_sensor_point_cloud_position']*np.array([1.,-1.,-1.])
K=np.array([[640*11.041/20.955,0,320],[0,640*11.041/20.955,320],[0,0,1.]])
registered=register_depth(points,K,depth.shape,measured_depth=depth)
result={}
for name,output in [('raw',depth),('registered',registered)]:
    valid=np.isfinite(output)&(output>.1)&(output<3)&np.isfinite(ideal)&(ideal>.1)&(ideal<3)
    delta=(output[valid]-ideal[valid])*1000
    result[name]={'valid':int(valid.sum()),'rmse_mm':float(np.sqrt(np.mean(delta**2))),'bias_mm':float(delta.mean())}
np.savez_compressed(args.output/'depth.npz',raw=depth,registered=registered,ideal=ideal,points_usd=arrays['depth_sensor_point_cloud_position'])
(args.output/'result.json').write_text(json.dumps(result,indent=2))
app.close()
print(json.dumps(result,indent=2))
assert result['registered']['valid']>1000,'insufficient registered sensor pixels'
assert result['registered']['rmse_mm']<10,'registered depth exceeds 10 mm on the test plane'
assert result['raw']['rmse_mm']>3*result['registered']['rmse_mm'],'registration did not resolve virtual stereo pixel shift'
