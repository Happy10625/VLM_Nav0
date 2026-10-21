"""Configuration gates retained alongside independent C completion."""


def test_camera_confirmation_is_evidence_not_startup_gate():
    from multi_agent_nav.config import load_config,robot_local_gates
    config=load_config();config['robot_ranger']['camera_calibrated']=False
    errors=robot_local_gates(config,'ranger',motion=True,check_map_files=False)
    assert not any('camera_calibrated' in error for error in errors)


def test_private_gates_do_not_require_resource_or_exit_geometry(monkeypatch):
    from multi_agent_nav.config import load_config,private_live_gates
    import multi_agent_nav.go2_exploration as core
    import multi_agent_nav.config as settings
    config=load_config();config['scene'].update(enabled=True,resources={},exit_points={},share_c_coordinates=False)
    config['robot_ranger'].update(exploration_strategy='generated_candidates_vlm',stop_margin=.2,
        camera_calibrated=False,costmap_source_verified=True,path_source_verified=True,
        peer_obstacle_verified=True,vision_validated=True)
    monkeypatch.setattr(core,'exploration_gates',lambda *args:[])
    monkeypatch.setattr(settings,'map_file_gates',lambda *args,**kwargs:[])
    assert private_live_gates(config,'ranger')==[]
