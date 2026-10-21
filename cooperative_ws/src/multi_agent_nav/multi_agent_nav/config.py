from pathlib import Path
import math
import hashlib
import yaml

def load_config(directory=None):
    explicit = directory is not None
    directory = Path(directory) if directory else Path(__file__).resolve().parents[1] / 'config'
    if not directory.is_dir():
        if explicit:
            raise FileNotFoundError(f'configuration directory does not exist: {directory}')
        from ament_index_python.packages import get_package_share_directory
        directory=Path(get_package_share_directory('multi_agent_nav'))/'config'
    return {name: yaml.safe_load((directory / f'{name}.yaml').read_text())
            for name in ('alignment', 'scene', 'robot_go2', 'robot_ranger')}

def map_file_info(path):
    path=Path(path).expanduser().resolve()
    data=yaml.safe_load(path.read_text())
    image=(path.parent / data['image']).resolve()
    yaml_hash=hashlib.sha256(path.read_bytes()).hexdigest()
    image_hash=hashlib.sha256(image.read_bytes()).hexdigest()
    return {'yaml_path':str(path),'image_path':str(image),'yaml_sha256':yaml_hash,
        'image_sha256':image_hash,'map_digest':hashlib.sha256((yaml_hash+image_hash).encode()).hexdigest()}

def valid_numbers(value, length):
    return isinstance(value,(list,tuple)) and len(value)==length and all(
        isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in value)

def region_errors(regions):
    errors=[]
    if not isinstance(regions,dict):
        return ['resource polygons must be an explicit mapping']
    for name,entry in (regions.items() if isinstance(regions,dict) else []):
        polygon=entry.get('polygon') if isinstance(entry,dict) else None
        if not isinstance(polygon,list) or len(polygon)<3 or not all(valid_numbers(p,2) for p in polygon):
            errors.append(f'{name}: finite resource polygon missing')
        elif abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(polygon,polygon[1:]+polygon[:1])))<1e-8:
            errors.append(f'{name}: degenerate resource polygon')
    return errors

def map_file_gates(config,robot,check_files=True):
    cfg=config[f'robot_{robot}']
    if not check_files:
        return [] if cfg.get('map_digest') else [f'{robot}: map_digest missing']
    try:
        info=map_file_info(cfg['map_path'])
        return [] if cfg.get('map_digest')==info['map_digest'] else [f'{robot}: map_digest missing or differs from YAML/image files']
    except (OSError,KeyError,TypeError,ValueError):
        return [f'{robot}: readable map YAML/image missing']

def robot_local_gates(config, robot_id, motion=False, vision=False, check_map_files=True):
    """Independent local checks; never stand in for the cooperative live gates."""
    errors=[]
    alignment=config['alignment'];go2=config['robot_'+robot_id]
    if alignment.get('simulation') or config['scene'].get('simulation'):
        errors.append('synthetic configuration cannot run live')
    if robot_id=='go2' and config['scene'].get('share_c_coordinates',True) is not False and (alignment.get('common_frame_id')!='lab_ref' or alignment['robots']['go2'].get('transform')!=[0.,0.,0.]):
        errors.append('lab_ref must equal Go2 map numerically')
    entry=alignment['robots'][robot_id]
    if not entry.get('local_map_id') or not entry.get('map_version'):
        errors.append(f'{robot_id}: map identity/version missing')
    errors+=map_file_gates(config,robot_id,check_map_files)
    if motion:
        if config['scene'].get('share_c_coordinates',True) or config.get('coordinate_mode')=='local_only':
            errors+=region_errors(config['scene'].get('resources'))
        margin=go2.get('stop_margin')
        if not isinstance(margin,(int,float)) or not math.isfinite(margin) or margin<=0:
            errors.append('measured positive stop margin missing')
        if go2.get('costmap_source_verified') is not True:
            errors.append(f'{robot_id} costmap_source_verified field verification required')
        if not go2.get('costmap_topic'):errors.append(f'{robot_id} realtime costmap topic missing')
        footprint=go2.get('footprint')
        if not isinstance(footprint,list) or len(footprint)<3 or not all(valid_numbers(p,2) for p in footprint):
            errors.append(f'{robot_id} finite footprint missing')
        for field,default in (('stationary_linear_speed',.02),('stationary_angular_speed',.02),
            ('stationary_duration',.5),('odom_max_age',.5),('pose_max_age',1.),
            ('costmap_max_age',1.),('goal_costmap_max_age',1.),
            ('path_receive_timeout',5.),('path_start_timeout',1.),('cancel_timeout',5.)):
            value=go2.get(field,default)
            if not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value) or value<=0:
                errors.append(f'{robot_id} positive finite {field} required')
        blocked=go2.get('costmap_blocked_cost',253)
        if not isinstance(blocked,int) or not 1<=blocked<=253:
            errors.append(f'{robot_id} costmap_blocked_cost must be within 1..253')
    if vision and go2.get('vision_validated') is not True:
        errors.append(f'{robot_id} vision_validated field verification required')
    return errors

def go2_local_gates(config,motion=False,vision=False,check_map_files=True):
    return robot_local_gates(config,'go2',motion,vision,check_map_files)

def live_gates(config,local_robot='go2'):
    if config['scene'].get('share_c_coordinates',True) is False:
        return private_live_gates(config,local_robot)
    errors = go2_local_gates(config,motion=True,vision=True,check_map_files=local_robot=='go2')
    alignment, scene = config['alignment'], config['scene']
    if not isinstance(scene.get('resources'),dict) or 'basket' not in scene['resources']:
        errors.append('basket resource polygon missing')
    if alignment.get('validated') is not True or not alignment.get('report'):
        errors.append('validated alignment report missing')
    else:
        from .calibration import fit_alignment
        report=alignment['report']
        try:
            recomputed=fit_alignment(*(report[k] for k in ('source_points','common_points','held_out_pairs','thresholds')))
            identities=report['identities']
            valid=recomputed['validated'] and all(
                identities[r]=={k:alignment['robots'][r][k] for k in ('local_map_id','map_version')} for r in ('go2','ranger'))
            transform=alignment['robots']['ranger']['transform']
            valid=valid and valid_numbers(transform,3) and all(abs(a-b)<1e-8 for a,b in zip(transform,recomputed['transform']))
            if not valid:errors.append('alignment report/transform/map identities do not validate')
        except (KeyError,TypeError,ValueError):errors.append('alignment report cannot be independently recomputed')
    if scene.get('enabled') is not True:
        errors.append('scene disabled: field coordinates missing')
    for robot in ('go2', 'ranger'):
        entry = alignment['robots'][robot]
        if not entry.get('local_map_id') or not entry.get('map_version') or entry.get('transform') is None:
            errors.append(f'{robot}: map identity/transform missing')
        cfg = config[f'robot_{robot}']
        if not cfg.get('map_path'):
            errors.append(f'{robot}: map path missing')
        if robot!='go2':errors+=map_file_gates(config,robot,check_files=local_robot==robot)
        points=scene.get('viewpoints',{}).get(robot,[])
        if not points or any(not p.get('id') or not valid_numbers(p.get('pose'),3) for p in points):
            errors.append(f'{robot}: measured viewpoints missing/invalid')
        for field in ('wait_points','exit_points'):
            if not valid_numbers(scene.get(field,{}).get(robot),3):
                errors.append(f'{robot}: {field} missing/invalid')
    for name,entry in scene.get('resources',{}).items():
        if name!='basket':
            for robot in ('go2','ranger'):
                exits=entry.get('transit_exits',{}).get(robot,[])
                if not exits or not all(valid_numbers(p,3) for p in exits):
                    errors.append(f'{name}: {robot} transit exits missing/invalid')
    if any(not isinstance(scene.get('targets',{}).get(t),str) or not scene['targets'][t].strip()
           or '现场填写' in scene['targets'][t] for t in ('A','B','C')):
        errors.append('measured A/B/C target descriptions missing')
    go2=config['robot_go2']
    for flag in ('path_source_verified','peer_obstacle_verified'):
        if go2.get(flag) is not True:errors.append(f'Go2 {flag} field verification required')
    if not go2.get('path_topic'):
        errors.append('executing Nav2 path source missing')
    return errors


def private_live_gates(config,robot_id):
    from .go2_exploration import exploration_gates
    errors=exploration_gates(config,robot_id)
    errors+=robot_local_gates(config,robot_id,motion=True,vision=True)
    scene=config['scene'];robot=config['robot_'+robot_id]
    if scene.get('enabled') is not True:errors.append('scene disabled: field coordinates missing')
    for peer in ('go2','ranger'):
        entry=config['alignment']['robots'][peer]
        if not entry.get('local_map_id') or not entry.get('map_version'):errors.append(peer+': map identity/version missing')
    if robot.get('local_map_id') and robot['local_map_id']!=config['alignment']['robots'][robot_id]['local_map_id']:
        errors.append(robot_id+': local map identity mismatch')
    if robot.get('map_version') and robot['map_version']!=config['alignment']['robots'][robot_id]['map_version']:
        errors.append(robot_id+': local map version mismatch')
    for flag in ('path_source_verified','peer_obstacle_verified'):
        if robot.get(flag) is not True:errors.append(robot_id+' '+flag+' field verification required')
    if not robot.get('goal_costmap_topic'):errors.append(robot_id+': global costmap topic missing')
    if robot.get('exploration_strategy')!='generated_candidates_vlm':
        errors.append(robot_id+': cooperative generated_candidates_vlm strategy required')
    if any(not isinstance(scene.get('targets',{}).get(t),str) or not scene['targets'][t].strip() or
            '现场填写' in scene['targets'][t] for t in ('A','B','C')):errors.append('measured A/B/C target descriptions missing')
    return list(dict.fromkeys(errors))
