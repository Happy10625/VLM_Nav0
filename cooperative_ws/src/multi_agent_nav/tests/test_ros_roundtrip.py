from dataclasses import replace
import pytest
from multi_agent_nav.models import AgentState,Command,Observation,Pose2D
from multi_agent_nav.ros_codec import to_ros,from_ros

@pytest.mark.parametrize('value',[
    AgentState(robot_id='ranger',task_id='t',local_map_id='r',map_version='v',alignment_id='a',
               ready=True,deposited=True,resource_id='basket',permit_command_id='deposit',token='token',resource_status='ENTERED'),
    Command(task_id='t',recipient_id='go2',action='GRANT_RESOURCE',token='token',parent_command_id='deposit'),
    Command(task_id='t',recipient_id='go2',kind='EXIT',token='token',parent_command_id='deposit',common_goal=Pose2D(1,2,.8)),
    Observation(target_id='C',status='DETECTED',position_valid=False),
    Observation(target_id='B',status='API_ERROR',reason='timeout')])
def test_generated_messages_preserve_task_semantics(value):
    assert from_ros(to_ros(value))==value
