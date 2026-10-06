import copy
import sys
from pathlib import Path
import unittest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/x_bot/scripts'))
from physical_pick_verification import lifted, retained, in_bin

class PhysicalPickAcceptance(unittest.TestCase):
    def setUp(self):
        self.initial = {'objects': {'book': {'position': [.6, .3, .8]}, 'fr3_hand': {'position': [.5,.3,.9]}}, 'place_bounds': [[-.2,-1,0],[.2,-.6,.4]]}
    def test_fixture_keeps_leg_gap_open(self):
        path = Path(__file__).resolve().parents[1] / 'src/x_bot/config/manipulation_fixture.yaml'
        boxes = yaml.safe_load(path.read_text())['boxes']
        def inside(point, box):
            return all(abs(point[i]-box['position'][i]) <= box['size'][i]/2 for i in range(3))
        self.assertTrue(any(inside([.8,0,.77], b) for b in boxes))
        self.assertFalse(any(inside([.8,0,.4], b) for b in boxes))
        self.assertTrue(all(min(b['size']) > 0 for b in boxes))
    def test_pushed_object_is_not_a_grasp(self):
        pushed = copy.deepcopy(self.initial)
        pushed['objects']['book']['position'] = [.5,.3,.81]
        self.assertFalse(lifted(self.initial, pushed, 'book'))
    def test_lift_and_transport(self):
        raised = copy.deepcopy(self.initial)
        raised['objects']['book']['position'][2] = .92
        self.assertTrue(lifted(self.initial, raised, 'book'))
        dropped = copy.deepcopy(raised)
        dropped['objects']['book']['position'] = [0,-.8,0.1]
        self.assertFalse(retained(raised, dropped, 'book'))
    def test_rotation_and_equal_distance_slip(self):
        held = copy.deepcopy(self.initial)
        hand = held['objects']['fr3_hand']
        hand['position'] = [0,0,1]
        held['objects']['book']['position'] = [.1,0,1]
        turned = copy.deepcopy(held)
        turned['objects']['fr3_hand']['orientation_wxyz'] = [2**-.5,0,0,2**-.5]
        turned['objects']['book']['position'] = [0,.1,1]
        self.assertTrue(retained(held, turned, 'book'))
        turned['objects']['book']['position'] = [0,-.1,1]
        self.assertFalse(retained(held, turned, 'book'))
    def test_bin_requires_real_position_and_bounds(self):
        self.assertFalse(in_bin(self.initial, 'book'))
        landed = copy.deepcopy(self.initial)
        landed['objects']['book']['position'] = [0,-.8,.1]
        self.assertTrue(in_bin(landed, 'book'))
        landed.pop('place_bounds')
        self.assertFalse(in_bin(landed, 'book'))
