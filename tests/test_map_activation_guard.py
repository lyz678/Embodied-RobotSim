import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from lifecycle_msgs.msg import State,Transition
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/mapping/x_bot_mapping'))
from x_bot_mapping.map_activation_guard import MapActivationGuard,required_transition


class MapActivationTests(unittest.TestCase):
    def test_response_loss_rechecks_state_and_does_not_repeat_successful_configuration(self):
        current=[State.PRIMARY_STATE_UNCONFIGURED];transitions=[]
        node=NS(state='get',change='change',received=False,get_logger=lambda:NS(warn=Mock()))
        def call(client,request):
            if client=='get':return NS(current_state=NS(id=current[0],label=str(current[0])))
            transitions.append(request.transition.id)
            if request.transition.id==Transition.TRANSITION_CONFIGURE:current[0]=State.PRIMARY_STATE_INACTIVE
            else:current[0]=State.PRIMARY_STATE_ACTIVE
            return None  # The server changed state, but its response was lost.
        node.call=call
        for _ in range(3):MapActivationGuard.step(node)
        self.assertEqual(transitions,[Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE])
        self.assertEqual(current[0],State.PRIMARY_STATE_ACTIVE)

    def test_active_or_transitioning_server_is_not_reconfigured(self):
        for state in (State.PRIMARY_STATE_ACTIVE,State.TRANSITION_STATE_CONFIGURING,State.TRANSITION_STATE_ACTIVATING):
            self.assertIsNone(required_transition(state))

if __name__=='__main__':unittest.main()
