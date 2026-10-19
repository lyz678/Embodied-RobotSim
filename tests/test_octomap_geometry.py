"""Integration regression: XYZ-only odom cloud raycasts from the moving sensor."""
import os,struct,subprocess,tempfile,time,unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2,PointField
class GeometryOriginTests(unittest.TestCase):
 def test_translated_sensor_keeps_nearby_hit_instead_of_clipping_at_world_origin(self):
  rclpy.init();node=Node('geometry_origin_test');broadcaster=StaticTransformBroadcaster(node)
  tf=TransformStamped();tf.header.frame_id='octomap_test_map';tf.child_frame_id='octomap_test_lidar';tf.transform.translation.x=20.;tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
  pub=node.create_publisher(PointCloud2,'/octomap_test/points',10);received=[]
  sub=node.create_subscription(PointCloud2,'/octomap_test/voxels',received.append,10)
  with tempfile.TemporaryFile(mode='w+') as log:
   command=['ros2','run','semantic_voxel_mapping','semantic_voxel_node','--ros-args','-r','__node:=octomap_geometry_test','-r','/semantic_map/voxels:=/octomap_test/voxels','-r','/occupied_cells_vis_array:=/octomap_test/markers','-r','/octomap_full:=/octomap_test/full','-r','/projected_map:=/octomap_test/grid','-r','/semantic_map/diagnostics:=/octomap_test/diagnostics','-p','use_sim_time:=false','-p','geometry_only:=true','-p','sensor_frame:=octomap_test_lidar','-p','map_frame:=octomap_test_map','-p','cloud_topic:=/octomap_test/points','-p','max_range:=2.0','-p','publish_rate:=10.0']
   process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
   try:
    deadline=time.monotonic()+20
    while not received and time.monotonic()<deadline and process.poll() is None:
     msg=PointCloud2();msg.header.frame_id='octomap_test_map';msg.header.stamp=node.get_clock().now().to_msg();msg.height=1;msg.width=1;msg.is_dense=True;msg.point_step=12;msg.row_step=12
     msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))];msg.data=struct.pack('<fff',21.,0.,1.)
     pub.publish(msg);rclpy.spin_once(node,timeout_sec=.1)
    if not received:log.seek(0);self.fail('No geometry map received: '+log.read()[-3000:])
    cloud=received[-1];self.assertGreater(cloud.width,0);x=next(f.offset for f in cloud.fields if f.name=='x');xs=[struct.unpack_from('<f',cloud.data,i*cloud.point_step+x)[0] for i in range(cloud.width)]
    self.assertTrue(any(20.9<v<21.2 for v in xs),xs)
   finally:
    process.terminate()
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:process.kill();process.wait()
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main()
