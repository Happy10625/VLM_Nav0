"""Local, preset-only furniture selection; this module never sends motion goals."""
import hashlib
import json
from pathlib import Path
import uuid

import yaml

from .config import map_file_info, valid_numbers


def load_catalog(config,robot_id='go2'):
    options = config['robot_'+robot_id]['semantic_search']
    path = Path(options['observation_points']).resolve()
    catalog = yaml.safe_load(path.read_text())
    if not isinstance(catalog, dict):
        raise ValueError('observation catalog must be a mapping')
    semantic_path = (path.parent / catalog['semantic_yaml']).resolve()
    image_path = (path.parent / catalog['annotated_image']).resolve()
    semantic = yaml.safe_load(semantic_path.read_text())
    if not isinstance(semantic, dict):
        raise ValueError('semantic map must be a mapping')
    identity = config['alignment']['robots'][robot_id]
    info = map_file_info(config['robot_'+robot_id]['map_path'])
    if (catalog.get('schema_version') != 1 or catalog.get('robot_id') != robot_id
            or catalog.get('frame_id') != 'map'
            or catalog.get('map_id') != identity['local_map_id']
            or catalog.get('map_version') != identity['map_version']
            or catalog['map_version'] != 'sha256:' + info['map_digest']
            or info['map_digest'] != config['robot_'+robot_id]['map_digest']):
        raise ValueError('observation catalog map identity mismatch')
    if (semantic.get('source_robot') != robot_id or semantic.get('frame_id') != 'map'
            or semantic.get('units') != 'meters' or semantic.get('schema_version') != 1):
        raise ValueError('invalid semantic map frame/schema')
    source = semantic['source_map']
    for key in ('yaml_sha256', 'image_sha256'):
        if source[key] != info[key]:
            raise ValueError('semantic map source digest mismatch')
    if (Path(source['yaml']).resolve() != Path(info['yaml_path'])
            or Path(source['image']).resolve() != Path(info['image_path'])):
        raise ValueError('semantic map source path mismatch')
    for asset, file in (('semantic_yaml', semantic_path), ('annotated_image', image_path)):
        if hashlib.sha256(file.read_bytes()).hexdigest() != catalog[asset + '_sha256']:
            raise ValueError('semantic asset digest mismatch: ' + asset)
    furniture = {}
    if not isinstance(semantic.get('furniture'), list):
        raise ValueError('semantic furniture list required')
    for item in semantic['furniture']:
        if not isinstance(item, dict):
            raise ValueError('furniture must be a mapping')
        fid = item.get('id')
        if not isinstance(fid, str) or not fid.strip() or fid in furniture:
            raise ValueError('invalid or duplicate furniture ID')
        furniture[fid] = {k: item.get(k) for k in ('id', 'category', 'name', 'room_id')}
    points = []
    ids = set()
    if not isinstance(catalog.get('viewpoints'), list):
        raise ValueError('observation viewpoints list required')
    for item in catalog['viewpoints']:
        if not isinstance(item, dict):
            raise ValueError('viewpoint must be a mapping')
        pid = item.get('viewpoint_id')
        if not isinstance(pid, str) or not pid.strip() or pid in ids:
            raise ValueError('invalid or duplicate viewpoint ID')
        ids.add(pid)
        if item.get('furniture_id') not in furniture:
            raise ValueError('unknown furniture for viewpoint: ' + pid)
        if type(item.get('enabled')) is not bool:
            raise ValueError('explicit enabled boolean required: ' + pid)
        if not item['enabled']:
            continue
        pose = item.get('pose') or {}
        if not isinstance(pose, dict):
            raise ValueError('pose mapping required: ' + pid)
        values = [pose.get(k) for k in ('x', 'y', 'yaw')]
        if not valid_numbers(values, 3):
            raise ValueError('finite local map pose required: ' + pid)
        points.append(dict(id=pid, furniture_id=item['furniture_id'], pose=values,
                           side=item.get('side')))
    if not points:
        raise ValueError('no enabled observation viewpoints')
    return dict(points=points, furniture=furniture, image=str(image_path),
                asset_paths=[str(path),str(semantic_path),str(image_path),info['yaml_path'],info['image_path']],
                map_id=catalog['map_id'], map_version=catalog['map_version'],
                semantic_yaml_sha256=catalog['semantic_yaml_sha256'],
                annotated_image_sha256=catalog['annotated_image_sha256'],
                current_room=options.get('current_room'))


def parse_selection(raw, candidates):
    def unique_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate selection field')
            result[key] = value
        return result
    try:
        payload = json.loads(raw, object_pairs_hook=unique_keys)
    except (TypeError, json.JSONDecodeError) as error:
        raise ValueError('furniture selection must be JSON') from error
    if (not isinstance(payload, dict)
            or set(payload) != {'action', 'furniture_id', 'reason'}
            or payload['action'] != 'inspect_furniture'
            or not isinstance(payload['furniture_id'], str)
            or payload['furniture_id'] not in candidates
            or not isinstance(payload['reason'], str)):
        raise ValueError('invalid furniture selection or unknown furniture ID')
    return payload


class SelectionState:
    def __init__(self, points):
        self.task_id = uuid.uuid4().hex
        self.revision = 0
        self.active = True
        self.records = {p['id']: dict(status='unobserved', retry_after=0.) for p in points}

    def eligible(self, now):
        return [pid for pid, row in self.records.items()
                if row['status'] not in ('observed', 'executing') and row['retry_after'] <= now]

    def update(self, pid, status, now, **details):
        self.records[pid] = dict(status=status, retry_after=now + 30. if status in
                                ('navigation_failed', 'observation_failed') else 0., **details)
        self.revision += 1

    def snapshot(self, candidates):
        return self.task_id, self.revision, tuple(candidates)

    def accepts(self, token, pid, now):
        return (self.active and token[:2] == (self.task_id, self.revision)
                and pid in token[2] and pid in self.eligible(now))

    def cancel(self):
        self.active = False
        self.revision += 1


class FixedViewpoints:
    def __init__(self, points):
        self.points = iter(points)

    def next_viewpoint(self, snapshot):
        return next(self.points, None)


class SemanticFurniture:
    def __init__(self, client):
        self.client = client

    def next_viewpoint(self, snapshot):
        reply = self.client.choose_furniture(snapshot)
        candidates = snapshot['candidates']
        # Validate again even when an injected client bypasses the API parser.
        reply = parse_selection(json.dumps(reply), [p['furniture_id'] for p in candidates])
        point = min((p for p in candidates if p['furniture_id'] == reply['furniture_id']),
                    key=lambda p: (p['path_length'], p['id']))
        return dict(point, selection_reason=reply['reason'])
