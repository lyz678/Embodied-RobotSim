"""Action outcome handling without requiring a running ROS installation."""

import source_packages
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import TestCase
from unittest.mock import patch

class Future:
    def __init__(self, value=None, ready=True): self.value, self.ready = value, ready
    def done(self): return self.ready
    def result(self): return self.value

class Handle:
    accepted = True
    def __init__(self, outcome): self.outcome, self.cancels = outcome, 0
    def get_result_async(self): return self.outcome
    def cancel_goal_async(self): self.cancels += 1; return Future(NS())

class GripperAction(TestCase):
    def setUp(self):
        class Goal:
            def __init__(self): self.command = NS(position=0, max_effort=0)
        modules = {'action_msgs': NS(), 'action_msgs.msg': NS(GoalStatus=NS(STATUS_SUCCEEDED=4)),
                   'control_msgs': NS(), 'control_msgs.action': NS(GripperCommand=NS(Goal=Goal))}
        path = Path(__file__).resolve().parents[1]/'src/manipulation/x_bot_manipulation/scripts/gripper_action.py'
        spec = importlib.util.spec_from_file_location('tested_gripper_action', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', modules): spec.loader.exec_module(self.module)
        self.node = NS(get_logger=lambda: NS(info=lambda *a: None, error=lambda *a: None))

    def run_goal(self, target, status=4, reached=False, stalled=False, pending=False):
        handle = Handle(Future(NS(status=status, result=NS(position=.03, reached_goal=reached, stalled=stalled)), ready=not pending))
        goals = []
        client = NS(wait_for_server=lambda **k: True, send_goal_async=lambda goal: (goals.append(goal) or Future(handle)))
        tick = iter(i*.1 for i in range(1000))
        with patch.object(self.module.time, 'sleep'), patch.object(self.module.time, 'monotonic', side_effect=lambda: next(tick)):
            result = self.module.control_gripper(self.node, client, target, timeout=.5)
        return result, handle, goals

    def test_closing_contact_stall_is_accepted(self):
        self.assertTrue(self.run_goal(.005, stalled=True)[0])

    def test_stalled_open_is_not_success(self):
        self.assertFalse(self.run_goal(.06, stalled=True)[0])

    def test_aborted_action_is_not_success_even_if_stalled(self):
        self.assertFalse(self.run_goal(.005, status=6, stalled=True)[0])

    def test_full_open_uses_project_joint_limit(self):
        success, _, goals = self.run_goal(.06, reached=True)
        self.assertTrue(success); self.assertEqual(goals[0].command.position, .06)

    def test_timeout_cancels_active_goal(self):
        success, handle, _ = self.run_goal(.005, pending=True)
        self.assertFalse(success); self.assertEqual(handle.cancels, 1)

    def test_empty_success_flags_are_rejected(self):
        self.assertFalse(self.run_goal(.005)[0])
