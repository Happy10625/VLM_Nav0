"""Map-aligned binary keepout masks, without ROS or motion."""
from pathlib import Path
import numpy as np
from PIL import Image
import pytest
import yaml

from vlm_nav.keepout import KeepoutMask, write_mask


def map_file(tmp_path,origin=(0.,0.,0.)):
    Image.new('L',(80,60),254).save(tmp_path/'map.pgm')
    path=tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=list(origin),
        mode='trinary',negate=0,free_thresh=.25,occupied_thresh=.65)))
    return path


def test_mask_uses_original_geometry_insets_boundary_and_keeps_map_untouched(tmp_path):
    source=map_file(tmp_path);before=source.read_bytes(),(tmp_path/'map.pgm').read_bytes()
    output=write_mask(source,[[1,1],[6,1],[6,5],[1,5]],tmp_path/'keepout_mask.yaml',.5)
    mask=KeepoutMask(source,output)
    assert mask.contains(3,3) and not mask.contains(.5,3)
    assert not mask.contains(1.2,3) and not mask.contains(6.1,3)
    assert not mask.contains(-.01,2) and not mask.contains(8.01,2)
    assert not mask.contains(float('nan'),0)
    assert set(np.unique(mask.pixels))=={0,255}
    assert mask.pixels.shape==(60,80)
    assert before==(source.read_bytes(),(tmp_path/'map.pgm').read_bytes())
    with pytest.raises(FileExistsError):write_mask(source,[[1,1],[6,1],[6,5],[1,5]],output,.5)


@pytest.mark.parametrize('fault',['size','resolution','origin','negate','mode','gray','source'])
def test_misaligned_or_ambiguous_mask_is_rejected(tmp_path,fault):
    source=map_file(tmp_path)
    output=write_mask(source,[[1,1],[6,1],[6,5],[1,5]],tmp_path/'keepout_mask.yaml',.2)
    data=yaml.safe_load(output.read_text())
    if fault=='size':Image.new('L',(40,30),255).save(tmp_path/data['image'])
    elif fault=='gray':Image.new('L',(80,60),128).save(tmp_path/data['image'])
    elif fault=='source':Image.new('L',(80,60),0).save(tmp_path/'map.pgm')
    else:data[fault]={'resolution':.2,'origin':[1,0,0],'negate':1,'mode':'scale'}[fault]
    output.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):KeepoutMask(source,output)


def test_concave_polygon_does_not_fill_its_notch(tmp_path):
    source=map_file(tmp_path)
    output=write_mask(source,[[1,1],[6,1],[6,5],[4,5],[4,3],[1,3]],tmp_path/'mask.yaml',0.)
    mask=KeepoutMask(source,output)
    assert mask.contains(2,2) and not mask.contains(2,4)


def test_polygon_rejects_self_intersections_and_outside_vertices(tmp_path):
    source=map_file(tmp_path)
    for polygon in ([[1,1],[4,4],[1,4],[4,1]],[[1,1],[90,1],[1,4]]):
        with pytest.raises(ValueError):write_mask(source,polygon,tmp_path/'mask.yaml',0.)


def test_mask_messages_must_match_pinned_geometry_and_all_cells(tmp_path):
    from array import array
    from types import SimpleNamespace as S
    source=map_file(tmp_path)
    output=write_mask(source,[[1,1],[6,1],[6,5],[1,5]],tmp_path/'mask.yaml',.2)
    mask=KeepoutMask(source,output)
    msg=S(header=S(frame_id='map'),info=S(width=80,height=60,resolution=.1,
        origin=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),
        data=array('b',np.where(mask.free,0,100).ravel()))
    assert mask.matches_grid(msg)
    msg.data[0]=0
    assert not mask.matches_grid(msg)
    msg.header.frame_id='odom'
    assert not mask.matches_grid(msg)
    info=S(type=0,base=0.,multiplier=1.,filter_mask_topic='/keepout_filter_mask')
    assert mask.matches_info(info)
    info.base=100.
    assert not mask.matches_info(info)


def test_keepout_launch_configures_both_costmaps_without_changing_navigation(tmp_path,monkeypatch):
    import importlib.util
    import json
    from launch.actions import IncludeLaunchDescription
    from launch import LaunchContext
    source=map_file(tmp_path)
    output=write_mask(source,[[1,1],[6,1],[6,5],[1,5]],tmp_path/'mask.yaml',.2)
    path=Path(__file__).resolve().parents[1]/'launch/navigation.launch.py'
    spec=importlib.util.spec_from_file_location('keepout_navigation_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'Node',lambda **kwargs:kwargs)
    c=LaunchContext()
    profile=Path(__file__).resolve().parents[1]/'config/nav2_ranger_local.yaml'
    c.launch_configurations.update(dict(localization_mode='amcl',overrides_file=str(profile),
        map=str(source),keepout_mask_yaml=str(output),keepout_mask_digest=KeepoutMask(source,output).digest))
    actions=module.configure_navigation(c)
    servers=[a for a in actions if isinstance(a,dict)]
    assert {'keepout_mask_server','costmap_filter_info_server','lifecycle_manager_keepout'}<={a['name'] for a in servers}
    params={a['name']:a['parameters'][0] for a in servers}
    assert params['keepout_mask_server']['topic_name']==params['costmap_filter_info_server']['mask_topic']
    assert params['costmap_filter_info_server']['type']==0
    assert params['lifecycle_manager_keepout']['node_names']==['keepout_mask_server','costmap_filter_info_server']
    file=next(a for a in servers if a['name']=='controller_server')['parameters'][0]
    merged=yaml.safe_load(Path(file).read_text())
    c.launch_configurations['keepout_mask_yaml']=''
    baseline=module.configure_navigation(c)
    baseline_file=next(a for a in baseline if isinstance(a,dict) and a['name']=='controller_server')['parameters'][0]
    original=yaml.safe_load(Path(baseline_file).read_text())
    c.launch_configurations['keepout_mask_yaml']=str(output)
    for name in ('global_costmap','local_costmap'):
        cfg=merged[name][name]['ros__parameters']
        assert cfg['filters']==['keepout_filter']
        assert cfg['keepout_filter']['enabled'] and cfg['keepout_filter']['plugin']=='nav2_costmap_2d::KeepoutFilter'
        for key,value in original[name][name]['ros__parameters'].items():assert cfg[key]==value
    assert merged['controller_server']['ros__parameters']['FollowPath']['max_vel_x']==.2
    c.launch_configurations['keepout_mask_digest']='old'
    with pytest.raises(ValueError,match='digest'):module.configure_navigation(c)


def test_mask_changes_invalidate_map_worker_evidence(tmp_path):
    from multi_agent_nav.config import map_file_info,map_file_evidence,file_signature,map_file_gates
    source=map_file(tmp_path)
    output=write_mask(source,[[1,1],[6,1],[6,5],[1,5]],tmp_path/'mask.yaml',.2)
    mask=KeepoutMask(source,output)
    cfg={'robot_ranger':dict(map_path=str(source),map_digest=map_file_info(source)['map_digest'],
        keepout_mask_yaml=str(output),keepout_mask_digest=mask.digest)}
    gates,paths,signature=map_file_evidence(cfg,'ranger')
    assert not gates and len(paths)==4 and file_signature(paths)==signature
    Image.new('L',(80,60),0).save(output.with_suffix('.pgm'))
    assert map_file_evidence(cfg,'ranger')[0] and 'keepout' in map_file_gates(cfg,'ranger')[0]


@pytest.mark.parametrize('frame',['map','odom'])
def test_polygon_document_cli_uses_own_map_and_refuses_other_frames(tmp_path,frame):
    import subprocess,sys
    source=map_file(tmp_path)
    cfg=tmp_path/'config';cfg.mkdir()
    (cfg/'robot_ranger.yaml').write_text(yaml.safe_dump(dict(map_path=str(source),footprint=[[-.2,-.1],[.2,-.1],[.2,.1],[-.2,.1]],footprint_padding=.03)))
    samples=[dict(header={'frame_id':frame},point=dict(x=x,y=y,z=0.)) for x,y in [[1.,1.],[6.,1.],[6.,5.],[1.,5.]]]
    document=tmp_path/'polygon.md';document.write_text('```txt\n'+yaml.safe_dump_all(samples)+'```\n')
    output=tmp_path/'mask'
    script=Path(__file__).resolve().parents[1]/'scripts/make_keepout_mask.py'
    result=subprocess.run([sys.executable,str(script),'--config-dir',str(cfg),'--polygon',str(document),'--output-dir',str(output)],capture_output=True,text=True)
    if frame=='odom':
        assert result.returncode!=0 and not output.exists()
    else:
        assert result.returncode==0,result.stderr
        data=yaml.safe_load(result.stdout)
        mask=KeepoutMask(source,output/'keepout_mask.yaml')
        assert mask.contains(1.2,3) and not mask.contains(.9,3)
        assert data['boundary_margin_m']==0.
        assert data['vertices']==4 and data['keepout_mask_digest']==mask.digest


def test_humble_keepout_rejects_rotated_mask_before_launching_nodes(tmp_path,monkeypatch):
    import importlib.util
    from launch import LaunchContext
    from types import SimpleNamespace as S
    import vlm_nav.keepout as keepout
    path=Path(__file__).resolve().parents[1]/'launch/navigation.launch.py'
    spec=importlib.util.spec_from_file_location('rotated_keepout_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(keepout,'KeepoutMask',lambda *args:S(origin=[0,0,.1]))
    c=LaunchContext();c.launch_configurations.update(dict(localization_mode='amcl',
        overrides_file=str(Path(__file__).resolve().parents[1]/'config/nav2_ranger_local.yaml'),
        map='/tmp/map.yaml',keepout_mask_yaml='/tmp/mask.yaml',keepout_mask_digest=''))
    with pytest.raises(ValueError,match='axis-aligned'):module.configure_navigation(c)
