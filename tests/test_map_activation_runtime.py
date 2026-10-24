"""Use an isolated ROS_DOMAIN_ID; starts only a synthetic map server."""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import rclpy

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/mapping/x_bot_mapping'))
from x_bot_mapping.map_activation_guard import MapActivationGuard


class MapActivationRuntimeTests(unittest.TestCase):
    def test_unmanaged_map_server_is_activated_and_publishes_grid(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            (folder/'map.pgm').write_bytes(b'P5\n4 4\n255\n'+bytes([254]*16))
            (folder/'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n')
            with (folder/'map_server.log').open('w+') as log:
                process=subprocess.Popen(['ros2','run','nav2_map_server','map_server','--ros-args','-p','use_sim_time:=false','-p',f'yaml_filename:={folder}/map.yaml'],stdout=log,stderr=subprocess.STDOUT)
                rclpy.init();node=MapActivationGuard()
                try:
                    self.assertTrue(node.run(timeout=25.,grace=0.))
                    self.assertTrue(node.received)
                finally:
                    node.destroy_node();rclpy.shutdown();process.terminate()
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:process.kill();process.wait()

if __name__=='__main__':unittest.main()
