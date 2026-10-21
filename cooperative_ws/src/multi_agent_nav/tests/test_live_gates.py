from copy import deepcopy
from multi_agent_nav.config import live_gates
from multi_agent_nav.mock import mock_config

def test_toggling_validated_flag_or_using_synthetic_scene_cannot_open_live():
    c=mock_config()
    assert live_gates(c)
    c['alignment']['simulation']=False;c['scene']['simulation']=False
    c['alignment']['report']={'validated':True}
    assert any('report' in e for e in live_gates(c))
    c['robot_go2']['path_source_verified']=False
    assert any('path' in e for e in live_gates(c))
