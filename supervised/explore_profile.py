"""Versioned, evidence-gated configuration. This validates records, not human truth."""
import hashlib
import json
import math


def number(value, low, high, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite in [{low}, {high}]')
    return float(value)


def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_profile(p, ready=True):
    if not isinstance(p, dict) or p.get('version') != 1 or p.get('kind') != 'camera-explore':
        raise ValueError('A version 1 camera-explore profile is required')
    for key in ('configuration_id', 'surface'):
        if not isinstance(p.get(key), str) or not p[key].strip():
            raise ValueError(f'Missing {key}')
    camera = p.get('camera', {})
    for key in ('identity', 'mounting_id'):
        if not isinstance(camera.get(key), str) or not camera[key].strip():
            raise ValueError(f'Missing camera {key}')
    for key in ('device', 'focus', 'width', 'height', 'rotation_degrees'):
        if key not in camera: raise ValueError(f'Missing camera {key}')
    number(camera['width'], 160, 4096, 'width')
    number(camera['height'], 120, 4096, 'height')
    for key, shape in (('matrix', (3, 3)), ('image_to_ground', (3, 3))):
        a = camera.get(key)
        if not isinstance(a, list) or len(a) != shape[0] or any(not isinstance(row, list) or len(row) != shape[1] for row in a):
            raise ValueError(f'Missing camera {key}')
        for row in a:
            for v in row: number(v, -1e7, 1e7, key)
        det = (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) - a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0]) + a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))
        if abs(det) < 1e-12: raise ValueError(f'Singular {key}')
    if not isinstance(camera.get('distortion'), list) or len(camera['distortion']) not in (4,5,8,12,14):
        raise ValueError('Missing distortion coefficients')
    for v in camera['distortion']: number(v, -1e4, 1e4, 'distortion')
    bounds = {
        'width_m': (.05, 1), 'length_m': (.05, 1), 'track_m': (.05, 1),
        'camera_forward_m': (-1, 1), 'blind_forward_m': (0, 1),
        'upper_speed_m_s': (.001, .5), 'upper_turn_rad_s': (.001, 6.3),
        'fallback_timeout_s': (.01, 30), 'braking_m': (0, 1), 'fallback_stop_m': (.001, 2), 'margin_m': (.02, .5),
        'encoder_m_per_unit': (.00001, 1), 'max_translation_disagreement_m': (.005, .1),
        'max_yaw_disagreement_rad': (.01, .2), 'floor_probability': (.5, .999),
        'near_limit': (-1e6, 1e6), 'score_low': (-1e6, 1e6), 'score_high': (-1e6, 1e6),
        'min_sharpness': (1, 10000), 'min_features': (8, 200),
        'clearance_max_age_s': (.5, 30), 'max_reprojection_px': (.1, 3), 'max_geometry_error_m': (.001, .05),
    }
    for key, limits in bounds.items(): number(p.get(key), *limits, key)
    if p['fallback_stop_m'] + 1e-9 < p['upper_speed_m_s']*p['fallback_timeout_s'] + p['braking_m']:
        raise ValueError('Fallback clearance must include command-loss timeout travel plus braking')
    if not p['score_low'] < p['near_limit'] < p['score_high']:
        raise ValueError('Relative-depth thresholds must be ordered')
    if p.get('encoder_signs') not in ([1,1], [-1,-1], [1,-1], [-1,1]):
        raise ValueError('Measured encoder signs required')
    if p.get('motor_polarity') not in ([1,1], [-1,-1], [1,-1], [-1,1]):
        raise ValueError('Measured motor polarity required')
    if not isinstance(p.get('models'), dict) or not all(p['models'].get(k) for k in ('floor','depth','detector')):
        raise ValueError('Pinned model identities required')
    if ready:
        for name in ('motion', 'camera', 'watchdog', 'perception', 'latency'):
            evidence = p.get('validation', {}).get(name, {})
            if (not isinstance(evidence, dict) or evidence.get('passed') is not True
                    or not isinstance(evidence.get('evidence'), list) or not evidence['evidence']
                    or any(not isinstance(x,str) or not x for x in evidence['evidence'])):
                raise PermissionError(f'Exploration locked: {name} validation pending')
        if any(not isinstance(v, dict) or not isinstance(v.get('sha256'), str) or len(v['sha256']) != 64 for v in p['models'].values()):
            raise PermissionError('Pinned model hashes required')
        if camera.get('focus_verified') is not True:
            raise PermissionError('Exploration locked: stable camera focus not verified')
    return p


def camera_matches(profile, frame):
    camera = profile['camera']
    return all(frame.get(k) == camera.get(k) for k in ('device', 'identity', 'width', 'height', 'rotation_degrees', 'focus')) and frame.get('focus_stable') is True


def configuration_fingerprint(profile):
    return digest({k:v for k,v in profile.items() if k != 'validation'})


def verify_recorded_validation(profile, root):
    """Check machine results and file provenance; human motion evidence still needs review."""
    from pathlib import Path
    root = Path(root).resolve()
    for name, category in profile['validation'].items():
        records = []
        for evidence in category['evidence']:
            path = (root / evidence).resolve()
            if not (path.is_relative_to(root/'data') or path.is_relative_to(root/'records')) or not path.is_file():
                raise ValueError('Evidence must be an existing project data or records file')
            if path.suffix == '.json':
                value = json.loads(path.read_text())
                if isinstance(value,dict): records.append(value)
        kind = {'perception':'held-out-perception', 'latency':'stationary-navigation-latency'}.get(name)
        if kind and not any(r.get('kind') == kind and r.get('passed') is True
                            and r.get('profile_fingerprint') == configuration_fingerprint(profile)
                            for r in records):
            raise PermissionError(f'Missing passing {name} report for this exact configuration')
