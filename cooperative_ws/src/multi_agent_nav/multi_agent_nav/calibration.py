"""Offline SE(2) fit, no ROS, no TF and no live configuration updates."""
import argparse
from pathlib import Path
import math
import numpy as np
import yaml

def fit_alignment(source_points, common_points, held_out_pairs, thresholds):
    required = ('fit_max', 'validation_max', 'geometry_ratio_min')
    if any(k not in thresholds or not math.isfinite(thresholds[k]) or thresholds[k] <= 0 for k in required):
        raise ValueError('explicit positive error and geometry thresholds required')
    src, dst = np.asarray(source_points, float), np.asarray(common_points, float)
    report = dict(direction='common_from_ranger', validated=False, transform=None,
                  thresholds=thresholds, source_points=source_points, common_points=common_points,
                  held_out_pairs=held_out_pairs, reason='invalid reference geometry')
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2 or len(src) < 3:
        return report
    if not np.isfinite(src).all() or not np.isfinite(dst).all():
        return report
    a, b = src-src.mean(axis=0), dst-dst.mean(axis=0)
    ratios = []
    for points in (a, b):
        singular = np.linalg.svd(points, compute_uv=False)
        ratios.append(float(singular[-1]/singular[0]) if singular[0] > 1e-9 else 0.0)
    report['geometry_ratio'] = min(ratios)
    if min(ratios) < thresholds['geometry_ratio_min'] or len(np.unique(src, axis=0)) != len(src):
        return report
    u, _, vt = np.linalg.svd(a.T@b)
    correction = np.eye(2)
    correction[-1, -1] = np.linalg.det(vt.T@u.T)
    rotation = vt.T@correction@u.T
    translation = dst.mean(axis=0)-rotation@src.mean(axis=0)
    report['transform'] = [float(translation[0]), float(translation[1]),
                           float(math.atan2(rotation[1, 0], rotation[0, 0]))]
    errors = np.linalg.norm(src@rotation.T+translation-dst, axis=1)
    report['fit_max'] = float(errors.max())
    report['fit_rms'] = float(np.sqrt(np.mean(errors**2)))
    report['reason'] = 'independent validation missing or failed'
    if not held_out_pairs:
        return report
    hold = np.asarray(held_out_pairs, float)
    if hold.ndim != 3 or hold.shape[1:] != (2, 2) or not np.isfinite(hold).all():
        return report
    # A reused fitting point is not an independent check.
    if any(np.any(np.linalg.norm(src-p, axis=1) < 1e-6) for p in hold[:, 0]):
        return report
    errors = np.linalg.norm(hold[:, 0]@rotation.T+translation-hold[:, 1], axis=1)
    report['validation_max'] = float(errors.max())
    report['validated'] = bool(report['fit_max'] <= thresholds['fit_max'] and
                               report['validation_max'] <= thresholds['validation_max'])
    report['reason'] = 'ok' if report['validated'] else 'error limit exceeded'
    return report

def main():
    parser = argparse.ArgumentParser(description='Fit candidate Ranger→lab_ref; prints report only')
    parser.add_argument('references', help='YAML with source_points/common_points/held_out_pairs/thresholds and map identities')
    args = parser.parse_args()
    data = yaml.safe_load(Path(args.references).read_text())
    report = fit_alignment(*(data[k] for k in ('source_points', 'common_points', 'held_out_pairs', 'thresholds')))
    report['identities'] = data.get('identities', {})
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report['validated'] else 1
