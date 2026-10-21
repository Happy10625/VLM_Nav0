from dataclasses import replace
import pytest
from multi_agent_nav.models import Command, AgentContext, Pose2D, Observation, validate_command
from multi_agent_nav.config import load_config, live_gates

def test_protocol_checks_identity_and_no_navigable_position():
    ctx = AgentContext('go2', 't', 'go2_map', 'v1', 'lab_ref', 'align', True)
    cmd = Command(task_id='t', command_id='c', recipient_id='go2', alignment_id='align',
                  kind='APPROACH_PICKUP', target_id='A', object_position=Pose2D(1, 2), position_valid=True)
    assert validate_command(cmd, ctx).accepted
    for change in ({'recipient_id': 'ranger'}, {'schema_version': 2}, {'task_id': 'old'},
                   {'common_frame_id': 'map'}, {'alignment_id': 'old'}, {'position_valid': False},
                   {'object_position': Pose2D(float('nan'), 0)}):
        assert not validate_command(replace(cmd, **change), ctx).accepted
    assert not Observation(target_id='A', status='DETECTED').navigable
    assert Observation(target_id='B', status='DETECTED', position_valid=True,
                       common_position=Pose2D(1, 2)).navigable

def test_broadcast_only_sync_and_live_templates_block():
    ctx = AgentContext('ranger', 't', 'ranger_map', 'v2', 'lab_ref', 'align', True)
    cmd = Command(task_id='t', recipient_id='*', alignment_id='align', action='SYNC')
    assert validate_command(cmd, ctx).accepted
    assert not validate_command(replace(cmd, action='EXECUTE'), ctx).accepted
    config = load_config()
    assert live_gates(config)
    assert config['alignment']['robots']['ranger']['transform'] is None
