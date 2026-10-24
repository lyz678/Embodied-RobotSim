import sys
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/manipulation/x_bot_manipulation/scripts'))
from local_map_session import LocalMapSession


class LocalMapSessionTests(unittest.TestCase):
    def make_session(self):
        node=NS(create_client=Mock(),create_subscription=Mock(),get_logger=lambda:NS(info=Mock(),error=Mock()),get_clock=lambda:NS(now=lambda:NS(nanoseconds=100*10**9)))
        return LocalMapSession(node)

    def test_search_resets_then_waits_for_current_camera_map(self):
        session=self.make_session();events=[];clock=[10.]
        session.clear='clear';session.enable='map';session.cloud='camera'
        def call(client,request):
            events.append((client,getattr(request,'data',None)));return True
        session.call=call
        def advance(dt):
            clock[0]+=dt
            # A latched snapshot from the previous location cannot authorize grasping.
            old=NS(width=1,height=1,header=NS(stamp=NS(sec=99,nanosec=0)))
            session.observe(old)
            self.assertEqual(session.received,-float('inf'))
            session.observe(NS(width=0,height=1,header=NS(stamp=NS(sec=0,nanosec=0))))
            self.assertEqual(session.received,-float('inf'))
            session.observe(NS(width=0,height=1,header=NS(stamp=NS(sec=100,nanosec=0))))
        with patch('local_map_session.time.monotonic',side_effect=lambda:clock[0]),patch('local_map_session.time.sleep',side_effect=advance):
            self.assertTrue(session.start())
        self.assertEqual(events,[('camera',False),('map',False),('clear',None),('map',True),('camera',True)])

    def test_timeout_pauses_both_inputs_and_retains_collision_snapshot(self):
        session=self.make_session();session.clear='clear';session.enable='map';session.cloud='camera';events=[];clock=[10.]
        session.call=lambda client,req:events.append((client,getattr(req,'data',None))) or True
        with patch('local_map_session.time.monotonic',side_effect=lambda:clock[0]),patch('local_map_session.time.sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            self.assertFalse(session.start(timeout=.2))
        self.assertEqual(events[-2:],[('camera',False),('map',False)])
        self.assertEqual(sum(client=='clear' for client,_ in events),1)

if __name__=='__main__':unittest.main()
