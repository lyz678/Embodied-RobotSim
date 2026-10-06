"""Separating retreat must never hide a new obstacle in the swept footprint."""

import source_packages
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/localization/x_bot_localization'))
from x_bot_control.contact_escape import separating_contacts


class ContactEscapeTests(unittest.TestCase):
    def scan(self):
        values = [math.inf]*720
        for i in range(720):
            angle = -math.pi+i*math.tau/720
            if abs(math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi))) < .6:
                values[i] = 4.
        values[360] = .35
        return values

    def evaluate(self, values, v=-.1, w=0.):
        return separating_contacts(values, -math.pi, math.tau/720, .1, 40., v, w)

    def test_front_contact_can_separate_without_mutating_scan(self):
        values = self.scan()
        self.assertEqual(self.evaluate(values), [360])
        self.assertEqual(values[360], .35)

    def test_new_rear_obstacle_blocks_even_without_initial_overlap(self):
        values = self.scan()
        values[0] = .6 # Outside current footprint, inside the reverse sweep.
        self.assertEqual(self.evaluate(values), [])

    def test_side_and_rear_contacts_block(self):
        for index in (0, 180, 540):
            values = self.scan(); values[index] = .35
            self.assertEqual(self.evaluate(values), [])

    def test_forward_spin_curved_reverse_and_fast_reverse_block(self):
        for v, w in ((.1, 0), (0, .2), (-.1, .01), (-.15, 0), (math.nan, 0)):
            self.assertEqual(self.evaluate(self.scan(), v, w), [])

    def test_blind_rear_and_visibility_gap_block(self):
        values = self.scan()
        for i in range(720):
            if i < 45 or i > 675: values[i] = math.inf
        self.assertEqual(self.evaluate(values), [])

    def test_clear_robot_does_not_exempt_any_scan_returns(self):
        values = self.scan(); values[360] = 1.
        self.assertEqual(self.evaluate(values), [])

    def test_sparse_mid360_rear_projection_keeps_both_sides_observed(self):
        values = self.scan()
        for i in range(720):
            angle = -math.pi+i*math.tau/720
            rear = math.atan2(math.sin(angle-math.pi), math.cos(angle-math.pi))
            if abs(rear) < .25:
                values[i] = math.inf
        self.assertEqual(self.evaluate(values), [360])


if __name__ == '__main__': unittest.main()
