"""Shared free-space exploration without ROS nodes or provider requests."""
from copy import deepcopy
from pathlib import Path
import math

import numpy as np
from PIL import Image
import pytest
import yaml

from test_generated_candidates import generated_config, run_generated
from multi_agent_nav.generated_candidates import GeneratedCandidates, circle_free


@pytest.mark.parametrize('mode', ['missing', 'unset', 'empty'])
def test_optional_labels_keep_stable_free_candidates(generated_config, mode):
    options = generated_config['robot_go2']['generated_candidates']
    path = Path(options['semantic_yaml'])
    if mode == 'empty':
        doc = yaml.safe_load(path.read_text())
        doc['furniture'] = []
        path.write_text(yaml.safe_dump(doc))
    else:
        path.unlink()
        if mode == 'unset':
            del options['semantic_yaml']
    catalog = GeneratedCandidates(generated_config)
    assert catalog.points and not catalog.furniture
    assert catalog.points == GeneratedCandidates(generated_config).points
    assert all(p['id'].startswith('free/') and p['furniture_id'] == '' for p in catalog.points)
    for p in catalog.points:
        x, y = catalog.map_xy(*p['pose'][:2])
        assert circle_free(catalog.raw, x, y, catalog.radius, .1, lambda cells: cells >= 253)
    assert path.exists() == (mode == 'empty')


def test_furniture_supplements_free_space_and_excludes_occupied_geometry(generated_config):
    catalog = GeneratedCandidates(generated_config)
    furniture = [p for p in catalog.points if p['furniture_id']]
    free = [p for p in catalog.points if not p['furniture_id']]
    assert len(furniture) == 4 and free
    assert len({p['id'] for p in catalog.points}) == len(catalog.points)
    assert all(math.hypot(p['pose'][0] - f['pose'][0], p['pose'][1] - f['pose'][1]) >= .5
               for p in free for f in furniture)
    assert all(math.hypot(max(abs(p['pose'][0]) - 1., 0.), max(abs(p['pose'][1]) - 1., 0.)) > catalog.radius
               for p in free)


@pytest.mark.parametrize('distance', [.2, 2.])
def test_furniture_observation_distance_does_not_change_free_grid(generated_config, distance):
    robot = generated_config['robot_go2']
    robot['generated_candidates']['semantic_yaml'] = ''
    before = GeneratedCandidates(generated_config).points
    robot['generated_candidates']['observation_distance'] = distance
    assert GeneratedCandidates(generated_config).points == before


def test_optional_asset_creation_and_deletion_invalidate_selection(generated_config):
    path = Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
    content = path.read_bytes()
    path.unlink()
    catalog = GeneratedCandidates(generated_config)
    assert catalog.unchanged()
    path.write_bytes(content)
    assert not catalog.unchanged()
    catalog = GeneratedCandidates(generated_config)
    path.unlink()
    assert not catalog.unchanged()


def test_existing_invalid_labels_are_rejected(generated_config):
    from multi_agent_nav.go2_exploration import exploration_gates
    path = Path(generated_config['robot_go2']['generated_candidates']['semantic_yaml'])
    path.write_text('invalid: [')
    assert any('generated candidates invalid' in e for e in exploration_gates(generated_config))


@pytest.mark.parametrize('asset', ['image', 'yaml'])
def test_deleted_map_asset_invalidates_selection(generated_config, asset):
    catalog = GeneratedCandidates(generated_config)
    path = catalog.map_path if asset == 'yaml' else Path(catalog.source['image'])
    path.unlink()
    assert not catalog.unchanged()


def test_rotated_ranger_map_uses_own_footprint_and_grid_centers(generated_config):
    from multi_agent_nav.config import map_file_info
    cfg = generated_config
    robot = deepcopy(cfg['robot_go2'])
    robot['generated_candidates']['semantic_yaml'] = ''
    robot['footprint'] = [[-.8, -.4], [.8, -.4], [.8, .4], [-.8, .4]]
    path = Path(robot['map_path'])
    doc = yaml.safe_load(path.read_text())
    doc['origin'] = [100., 200., math.pi / 2]
    path.write_text(yaml.safe_dump(doc))
    info = map_file_info(path)
    robot['map_digest'] = info['map_digest']
    cfg['robot_ranger'] = robot
    cfg['alignment']['robots']['ranger'] = dict(local_map_id='ranger', map_version='sha256:' + info['map_digest'])
    catalog = GeneratedCandidates(cfg, 'ranger')
    assert catalog.points
    # A 1.8 m grid starts at local (0.95, 0.95), then applies the map rotation.
    assert catalog.points[0]['pose'] == pytest.approx([99.05, 200.95, math.pi / 2])
    for p in catalog.points:
        x, y = catalog.map_xy(*p['pose'][:2])
        assert circle_free(catalog.raw, x, y, catalog.radius, .1, lambda cells: cells >= 253)


@pytest.mark.parametrize('annotated', [False, True])
def test_vlm_shortlist_is_nearby_first_even_with_furniture(generated_config, tmp_path, annotated):
    if not annotated:
        generated_config['robot_go2']['generated_candidates']['semantic_yaml'] = ''
    adapter, events, requests, outcome = run_generated(generated_config, tmp_path, 'found')
    assert outcome == 'TARGET_FOUND' and len(requests) == 1
    candidates = requests[0]['candidates']
    assert len(candidates) == 12
    order = [(math.hypot(*p['pose'][:2]), p['id']) for p in candidates]
    assert order == sorted(order)
    assert len(requests[0]['images']) == 2 and adapter.scans == 1


def test_no_label_images_are_identical_and_leave_map_unchanged(generated_config, tmp_path):
    from multi_agent_nav.generated_candidates import render_selection
    from multi_agent_nav.models import Pose2D
    generated_config['robot_go2']['generated_candidates']['semantic_yaml'] = ''
    catalog = GeneratedCandidates(generated_config)
    original = Path(catalog.source['image']).read_bytes()
    images = render_selection(catalog, [dict(catalog.points[0], display_id='P1')], {}, Pose2D(), tmp_path / 'images')
    assert len(images) == 2
    assert np.array_equal(np.asarray(Image.open(images[0])), np.asarray(Image.open(images[1])))
    assert Path(catalog.source['image']).read_bytes() == original


def test_offline_report_uses_null_for_unconfigured_labels(generated_config, monkeypatch, capsys):
    import json
    from multi_agent_nav import go2_check
    generated_config['robot_go2']['generated_candidates']['semantic_yaml'] = ''
    monkeypatch.setattr(go2_check, 'load_config', lambda *_: generated_config)
    monkeypatch.setattr(go2_check, 'live_gates', lambda *_: [])
    monkeypatch.setattr(go2_check, 'run_ros', lambda *_: pytest.fail('offline initialized ROS'))
    assert go2_check.main(['--offline', '--strategy', 'generated_candidates_vlm']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['semantic_search']['semantic_yaml'] is None
    assert report['semantic_search']['points'] and not report['motion_sent']
