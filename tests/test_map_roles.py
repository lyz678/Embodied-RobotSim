"""Isolated ROS/CUDA integration: global semantic paint and local search gating.

Run with ROS_DOMAIN_ID set to an unused domain, after sourcing ROS/workspace.
"""
import struct
import subprocess
import tempfile
import time
import unittest
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_srvs.srv import SetBool
from moveit_msgs.msg import PlanningScene
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter


class MapRolesTests(unittest.TestCase):
    def test_lidar_owns_geometry_and_camera_map_integrates_only_when_enabled(self):
        rclpy.init();node=Node('map_roles_test');broadcaster=StaticTransformBroadcaster(node)
        tf=TransformStamped();tf.header.frame_id='roles_world';tf.child_frame_id='roles_sensor';tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
        raw=node.create_publisher(PointCloud2,'/roles/raw',10)
        semantic=node.create_publisher(PointCloud2,'/roles/semantic',10)
        camera=node.create_publisher(PointCloud2,'/roles/camera',10)
        received={'global':[],'local':[]};subs=[];scenes=[]
        subs.append(node.create_subscription(PlanningScene,'/planning_scene',scenes.append,10))
        from rclpy.qos import QoSProfile,DurabilityPolicy
        for scope in received:
            subs.append(node.create_subscription(PointCloud2,f'/roles/{scope}/voxels',lambda msg,s=scope:received[s].append(msg),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        def cloud(labels=False,x=1.05,confidence=.9):
            msg=PointCloud2();msg.header.frame_id='roles_sensor';msg.header.stamp=node.get_clock().now().to_msg();msg.width=msg.height=1;msg.point_step=20 if labels else 12;msg.row_step=msg.point_step
            msg.fields=[PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))]
            if labels:
                msg.fields.extend([PointField(name='rgb',offset=12,datatype=PointField.UINT32,count=1),PointField(name='confidence',offset=16,datatype=PointField.FLOAT32,count=1)])
                msg.data=struct.pack('<fffIf',x,.05,.55,0xff0000,confidence)
            else:msg.data=struct.pack('<fff',x,.05,.55)
            return msg
        def spin_until(condition,timeout=8.,publish=None):
            end=time.monotonic()+timeout;last=0.
            while time.monotonic()<end:
                if publish and time.monotonic()-last>.12:publish();last=time.monotonic()
                rclpy.spin_once(node,timeout_sec=.03)
                if condition():return True
            return False
        processes=[]
        with tempfile.TemporaryFile(mode='w+') as log:
            try:
                for scope in ('global','local'):
                    args=['ros2','run','semantic_voxel_mapping','semantic_voxel_node','--ros-args','-r',f'__node:={scope}_roles','-r',f'/semantic_map/voxels:=/roles/{scope}/voxels',
                        '-p','use_sim_time:=false','-p','map_frame:=roles_world','-p','sensor_frame:=roles_sensor','-p','self_filter:=false','-p','resolution:=0.1','-p',f'publish_moveit_scene:={"true" if scope=="local" else "false"}','-p','planning_frame:=roles_world',
                        '-p','integration_rate:=20.0','-p','publish_rate:=20.0','-p','gpu_scratch_mib:=16','-p','max_voxels:=10000']
                    if scope=='global':args+=['-p','cloud_topic:=/roles/raw','-p','geometry_only:=true','-p','semantic_cloud_topic:=/roles/semantic']
                    else:args+=['-p','cloud_topic:=/roles/camera','-p','enabled_on_start:=false']
                    processes.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))
                self.assertTrue(spin_until(lambda:bool(received['global']),publish=lambda:(raw.publish(cloud()),camera.publish(cloud(True)))))
                self.assertEqual(received['local'],[],'Local map must remain empty during navigation')
                self.assertEqual(scenes,[],'Global map must not write the MoveIt collision scene')
                def colors(scope):
                    msg=received[scope][-1];offset=next(f.offset for f in msg.fields if f.name=='rgb')
                    return [struct.unpack_from('<I',msg.data,i*msg.point_step+offset)[0] for i in range(msg.width)]
                self.assertEqual(colors('global'),[0xffffff])
                spin_until(lambda:False,timeout=.6,publish=lambda:semantic.publish(cloud(True,confidence=.4)))
                self.assertEqual(colors('global'),[0xffffff],'Old 0.5 gate filters a visible 0.4 detection')
                parameters=AsyncParameterClient(node,'global_roles')
                self.assertTrue(parameters.wait_for_services(timeout_sec=3.))
                future=parameters.set_parameters([Parameter('confidence_threshold',value=.25)])
                self.assertTrue(spin_until(future.done));self.assertTrue(future.result().results[0].successful)
                self.assertTrue(spin_until(lambda:0xff0000 in colors('global'),publish=lambda:semantic.publish(cloud(True,confidence=.4))))
                future=parameters.set_parameters([Parameter('confidence_threshold',value=1.1)])
                self.assertTrue(spin_until(future.done));self.assertFalse(future.result().results[0].successful)
                self.assertEqual(len(colors('global')),1)
                # Semantics at a point absent from LiDAR must not invent an obstacle.
                spin_until(lambda:False,timeout=.6,publish=lambda:semantic.publish(cloud(True,2.05)))
                self.assertEqual(len(colors('global')),1)
                enable=node.create_client(SetBool,'/local_roles/set_enabled')
                self.assertTrue(enable.wait_for_service(timeout_sec=3.))
                future=enable.call_async(SetBool.Request(data=True));self.assertTrue(spin_until(future.done));self.assertTrue(future.result().success)
                def empty_camera():
                    msg=cloud(True);msg.width=0;msg.row_step=0;msg.data=b'';camera.publish(msg)
                spin_until(lambda:False,timeout=.3,publish=empty_camera)
                self.assertEqual(received['local'],[],'Empty sensor inputs must not masquerade as a fresh collision map')
                self.assertTrue(spin_until(lambda:bool(received['local']),publish=lambda:camera.publish(cloud(True))))
                self.assertTrue(scenes and scenes[-1].world.octomap.octomap.data,'Camera map must supply MoveIt collision geometry')
                future=enable.call_async(SetBool.Request(data=False));self.assertTrue(spin_until(future.done))
                spin_until(lambda:False,timeout=.3)
                count=len(received['local'])
                spin_until(lambda:False,timeout=.6,publish=lambda:camera.publish(cloud(True,2.05)))
                self.assertEqual(len(received['local']),count,'Paused local map must retain its collision snapshot')
            except Exception:
                log.seek(0);print(log.read()[-5000:]);raise
            finally:
                for process in processes:process.terminate()
                for process in processes:
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                node.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
