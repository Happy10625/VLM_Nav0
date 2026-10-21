"""Distance contract shared by single-robot and cooperative Go2 execution."""
import math
from pathlib import Path
import yaml


def target_distance(pose, target):
    if pose is None or target is None:
        return math.inf
    if not all(math.isfinite(v) for v in (*pose[:2], *target[:2])):
        return math.inf
    return math.hypot(pose[0]-target[0], pose[1]-target[1])


def stop_radius(success_radius, cancel_radius, retry):
    return success_radius if retry > 0 else cancel_radius


def arrival_decision(distance, success_radius, retry, retry_limit):
    if math.isfinite(distance) and distance <= success_radius:
        return 'SUCCESS'
    return 'RETRY' if retry < retry_limit else 'FAILED'


def load_contract(files):
    if not files:
        raise ValueError('original VLM arrival config files required')
    params = {}
    for file in files:
        params.update(yaml.safe_load(Path(file).read_text())['vlm_nav']['ros__parameters'])
    output = {k: params[k] for k in ('target_success_radius', 'approach_cancel_radius', 'arrival_retry_limit')}
    for key in ('target_success_radius', 'approach_cancel_radius'):
        v = output[key]
        if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
            raise ValueError('positive finite ' + key + ' required')
    if output['approach_cancel_radius'] < output['target_success_radius']:
        raise ValueError('cancel radius must not be inside success radius')
    if type(output['arrival_retry_limit']) is not int or output['arrival_retry_limit'] < 0:
        raise ValueError('nonnegative integer arrival_retry_limit required')
    return output
