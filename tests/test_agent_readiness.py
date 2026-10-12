"""Run after sourcing ROS and the workspace, using the Agent Python environment."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/embodied/llm_agent'))
from agent_server import LLMAgentNode, Qwen3Agent


class ReadinessTests(unittest.IsolatedAsyncioTestCase):
    def test_false_missing_and_stale_localization(self):
        state = SimpleNamespace(_localization_ready=True, _localization_received=100.)
        with patch('agent_server.time.monotonic', return_value=100.2):
            self.assertTrue(LLMAgentNode.localization_ready(state))
        with patch('agent_server.time.monotonic', return_value=100.6):
            self.assertFalse(LLMAgentNode.localization_ready(state))
        state._localization_received = -float('inf')
        self.assertFalse(LLMAgentNode.localization_ready(state))
        state._localization_ready = False
        self.assertFalse(LLMAgentNode.localization_ready(state))

    async def test_unready_actions_are_rejected_before_dispatch(self):
        robot = SimpleNamespace(_node=SimpleNamespace(localization_ready=lambda: False),
                                navigate=AsyncMock(), pick=AsyncMock(), move=AsyncMock(),
                                arm_go_home=AsyncMock(), stop=AsyncMock())
        agent = object.__new__(Qwen3Agent)
        agent._robot = robot
        for name, args in [('navigate_to_location', {'location':'厨房'}),
                           ('pick_object', {'object_name':'杯子'}),
                           ('move_robot', {'direction':'forward'}), ('arm_go_home', {})]:
            ok, message = await agent._execute_tool(name, args, AsyncMock())
            self.assertFalse(ok)
            self.assertIn('定位未就绪', message)
        for method in [robot.navigate, robot.pick, robot.move, robot.arm_go_home]:
            method.assert_not_called()
        self.assertTrue((await agent._execute_tool('stop_robot', {}, AsyncMock()))[0])
        robot.stop.assert_awaited_once()
        self.assertTrue((await agent._execute_tool('move_robot', {'direction':'stop'}, AsyncMock()))[0])
        robot.move.assert_awaited_once()

    async def test_ready_navigation_is_dispatched(self):
        robot = SimpleNamespace(_node=SimpleNamespace(localization_ready=lambda: True),
                                navigate=AsyncMock(return_value=(True, '模拟成功')))
        agent = object.__new__(Qwen3Agent)
        agent._robot = robot
        self.assertTrue((await agent._execute_tool('navigate_to_location', {'location':'厨房'}, AsyncMock()))[0])
        robot.navigate.assert_awaited_once_with('厨房')


if __name__ == '__main__':
    unittest.main()
